/*
 * comm_radio.c
 *
 *  Created on: Dec 13, 2013
 *      Author: petera
 */

#include "comm_radio.h"
#include "comm.h"
#include "nrf905_impl.h"
#include "taskq.h"
#include "app.h"

#define COMRAD_RAD_CFG_PA       (1)
#define COMRAD_RAD_CFG_CH       (2)

typedef struct {
  u8_t data[COMM_RADIO_LNK_MAX_DATA];
  u8_t guard;
  comm_arg rx;
} pool_pkt;

static struct radio_s {
  comm stack;

  // last received sequence number
  u16_t last_seqno;
  // bit mask of 32 last received sequence numbers
  u32_t seq_mask;

  // last received packet
  comm_arg *cur_rx;

  // rx pkt pool ix
  u8_t pool_ix;
  // rx pkt pool
  pool_pkt pool[COMM_RADIO_POOLED_PACKETS];
  // used pool entries
  u8_t pool_used;

  // original communication stack nwk layer rx to upper layer report function
  comm_rx_fn post_nwk_comm_rx_up_f;
  task *comm_tick_task;
  task_timer comm_tick_timer;

  // radio debug conditions emulation
  u8_t bad_link_simul;
  // radio supervision ticker
  task *rad_tick_task;
  task_timer rad_tick_timer;
  // link quality
  u8_t lqual;
  // radio pending configuration
  u8_t rad_cfg;
  struct {
    // current radio pa setting
    nrf905_cfg_pa_pwr pa;
    // current radio channel freq setting
    u16_t ch_freq;
  } rad_config;
  // radio pa adjustment scheme vars
  bool paired;
  bool lost_pair;
  u8_t lqual_samples;
  u16_t sum_lqual;
  u32_t sum_client_rx_pkts;
} comrad;

//
// comm stack implementation defs
//

static int comrad_com_rx_pkt(comm *comm, comm_arg *rx,  unsigned short len, unsigned char *data);
static void comrad_com_ack_pkt(comm *comm, comm_arg *rx, unsigned short seqno, unsigned short len, unsigned char *data);
static void comrad_com_tra_inf(comm *comm, comm_arg *rx);
static void comrad_com_alert(comm *comm, comm_addr addr, unsigned char type, unsigned short len, unsigned char *data);

static void comrad_com_err(comm *comm, int err, unsigned short seqno, unsigned short len, unsigned char *data);

static comm_time comrad_com_get_tick_count(void);

static int comrad_com_tx_flush(comm *comm, comm_arg* tx);

static void comrad_com_alloc(comm *c, void **data, void **arg, unsigned int size_data, unsigned int size_arg);
static void comrad_com_free(comm *c, void *data, void *arg);

static int comrad_com_nwk_rx(comm *com, comm_arg *rx);

static void comrad_com_tick_task_f(u32_t arg, void *arg_p);

//
// radio funcs, stack implementation and other
//

static void comrad_rad_on_rx(u8_t *data, u8_t len);
static void comrad_rad_on_tx(int res);
static void comrad_rad_on_cfg(int res);
static void comrad_rad_on_err(nrf905_state state, int res);

static void comrad_rad_return(void);
static void comrad_rad_super_task_f(u32_t arg, void *arg_p);
static void comrad_rad_request_cfg_update(u8_t cfg);

//
// comm radio api
//

int COMRAD_send(const u8_t *data, u16_t len, bool ack) {
#ifdef SECONDARY
  return comm_tx(&comrad.stack, 1, len, (u8_t *)data, ack);
#else
  return comm_tx(&comrad.stack, 2, len, (u8_t *)data, ack);
#endif
}

int COMRAD_reply(const u8_t *data, u16_t len) {
  s32_t res = comm_reply(&comrad.stack, comrad.cur_rx, len, (u8_t *)data);
  if (res < R_COMM_OK) DBG(D_COMM, D_WARN, "COMrad reply failed %i\n", res);
  return res;
}

void COMRAD_report_paired(bool paired) {
  if (!paired && comrad.paired) {
    comrad.lost_pair = TRUE;
  }
  comrad.sum_client_rx_pkts = 0;
  comrad.paired = paired;
  comrad.last_seqno = 0;
  comrad.seq_mask = 0;
}

u8_t COMRAD_get_link_qual(void) {
  return comrad.lqual;
}

void COMRAD_dbg_pkt_drop_rate(u8_t percentage) {
  comrad.bad_link_simul = percentage;
}


void COMRAD_init(void) {
  int res;
  memset(&comrad, 0, sizeof(struct radio_s));

  //
  // comm stack setup
  //
  // skip preamble and crc in comm stack, this is done by radio nrf905
  comm_init(
    &comrad.stack,        // comm stack struct
    COMM_CONF_SKIP_PREAMPLE | COMM_CONF_SKIP_CRC,
#ifdef SECONDARY
    2,                    // this address
#else
    1,                    // this address
#endif
    0,                    // comm_phy_rx_char - called via radio irq
    0, //COMRAD_tx_char,          // comm_phy_tx_char
    0, //COMRAD_tx_buf,           // comm_phy_tx_buf
    comrad_com_tx_flush,          // comm_phy_tx_flush
    comrad_com_get_tick_count,    // comm_app_get_time
    comrad_com_rx_pkt,            // comm_app_user_rx
    comrad_com_ack_pkt,           // comm_app_user_ack
    comrad_com_err,               // comm_app_user_err
    comrad_com_tra_inf,           // comm_app_user_inf
    comrad_com_alert              // comm_app_alert
  );
  // using comm stacks allocation callback for pkt buffers
  comm_init_alloc(&comrad.stack, comrad_com_alloc, comrad_com_free);

  // rewire communication stacks network layer callback to go via taskq
  // save old upcall from network layer
  comrad.post_nwk_comm_rx_up_f = comrad.stack.nwk.up_rx_f;
  // set new upcall to invoke old upcall via task instead
  comrad.stack.nwk.up_rx_f = comrad_com_nwk_rx;

  // start comm stack ticker
  comrad.comm_tick_task = TASK_create(comrad_com_tick_task_f, TASK_STATIC);
  TASK_start_timer(comrad.comm_tick_task, &comrad.comm_tick_timer, 0, 0, 0, 7, "comm_tick");

  //
  // radio setup
  //
  comrad.rad_config.pa = NRF905_CFG_PA_PWR_10;
  comrad.rad_config.ch_freq = 128; // 435.227MHz
  // callbacks
  NRF905_IMPL_init(comrad_rad_on_rx, comrad_rad_on_tx, comrad_rad_on_cfg, comrad_rad_on_err);
  // config
  nrf905_config spybot_conf;
  spybot_conf.auto_retransmit = NRF905_CFG_AUTO_RETRAN_OFF;
  spybot_conf.channel_freq = comrad.rad_config.ch_freq;
  spybot_conf.crc_en = NRF905_CFG_CRC_ON;
  spybot_conf.crc_mode = NRF905_CFG_CRC_MODE_16BIT;
  spybot_conf.crystal_frequency = NRF905_CFG_XTAL_FREQ_16MHZ;
  spybot_conf.hfreq_pll = NRF905_CFG_HFREQ_433;
  spybot_conf.out_clk_enable = NRF905_CFG_OUT_CLK_OFF;
  spybot_conf.out_clk_freq = NRF905_CFG_OUT_CLK_FREQ_1MHZ;
  spybot_conf.pa_pwr= comrad.rad_config.pa;
#ifdef SECONDARY
  spybot_conf.rx_address = 0x9ce3aed2;
#else
  spybot_conf.rx_address = 0x631c512d;
#endif
  spybot_conf.rx_address_field_width = NRF905_CFG_ADDRESS_FIELD_WIDTH_4;
  spybot_conf.rx_payload_width = COMM_LNK_MAX_DATA;
  spybot_conf.rx_reduced_power = NRF905_CFG_RX_REDUCED_POWER_OFF;
  spybot_conf.tx_address_field_width = NRF905_CFG_ADDRESS_FIELD_WIDTH_4;
  spybot_conf.tx_payload_width = COMM_LNK_MAX_DATA;

  res = NRF905_IMPL_conf(&spybot_conf, TRUE);
  if (res != NRF905_OK) {
    DBG(D_COMM, D_WARN, "COMRAD init config failed %i\n", res);
  }
  SYS_hardsleep_ms(250); // ugly wait for conf to finish - todo
#ifdef SECONDARY
  u8_t tx_addr[4] = {0x63, 0x1c, 0x51, 0x2d};
#else
  u8_t tx_addr[4] = {0x9c, 0xe3, 0xae, 0xd2};
#endif
  res = NRF905_IMPL_conf_tx_addr(tx_addr, TRUE);
  if (res != NRF905_OK) {
    DBG(D_COMM, D_WARN, "COMRAD config tx addr failed %i\n", res);
  }
  SYS_hardsleep_ms(250); // ugly wait for conf to finish - todo
  comrad_rad_return();

  // start radio supervision ticker
  comrad.rad_tick_task = TASK_create(comrad_rad_super_task_f, TASK_STATIC);
  TASK_start_timer(comrad.rad_tick_task, &comrad.rad_tick_timer, 0, 0,
      COMRAD_RADIO_ADJUST_TICK_D, COMRAD_RADIO_ADJUST_TICK_D, "rad_tick");
}


//
// radio stack implementation functions
//

#ifdef CONFIG_SPYBOT_APP_CLIENT
static u8_t comrad_rad_client_rxed_packets_ratio(void) {
  if (comrad.paired) {
    u32_t pkt_ratio_per_supervision_cycle =
        (255*comrad.sum_client_rx_pkts) /
        (COMRAD_RADIO_ADJUST_TICK_D / COMM_OTHER_RECURRENCE);
    pkt_ratio_per_supervision_cycle = MIN(255, pkt_ratio_per_supervision_cycle);
    comrad.sum_client_rx_pkts = 0;
    return (u8_t)pkt_ratio_per_supervision_cycle;
  } else {
    // we're not paired, so pretend we have a lousy link
    // thus upping the PA until we get paired
    return 0;
  }
}
#endif


// updates given configuration, where cfg may be any of COMRAD_RAD_CFG_*
static void comrad_rad_update_cfg(u8_t cfg, bool force) {
  int res = NRF905_OK;
  if (cfg == COMRAD_RAD_CFG_PA) {
    res = NRF905_IMPL_conf_pa(comrad.rad_config.pa, force);
  }
  if (cfg == COMRAD_RAD_CFG_CH) {
    res = NRF905_IMPL_conf_channel(comrad.rad_config.ch_freq, force);
  }
  if (res == NRF905_OK) {
    comrad.rad_cfg &= ~(1<<cfg); // clear radio setting request
    IF_DBG(D_COMM, D_INFO) {
      if (cfg == COMRAD_RAD_CFG_PA) {
        switch (comrad.rad_config.pa) {
        case NRF905_CFG_PA_PWR_10:
          DBG(D_COMM, D_INFO, "comRAD: pa = 10dB\n");
          break;
        case NRF905_CFG_PA_PWR_6:
          DBG(D_COMM, D_INFO, "comRAD: pa = 6dB\n");
          break;
        case NRF905_CFG_PA_PWR_m2:
          DBG(D_COMM, D_INFO, "comRAD: pa = -2dB\n");
          break;
        case NRF905_CFG_PA_PWR_m10:
          DBG(D_COMM, D_INFO, "comRAD: pa = -10dB\n");
          break;
        default:
          DBG(D_COMM, D_INFO, "comRAD: pa = UNKNOWN\n");
          break;
        }
      }
      else if (cfg == COMRAD_RAD_CFG_CH) {
        // (422.4 + CH_NOd /10)*(1+HFREQ_PLLd)
        u32_t freq = 4224 + comrad.rad_config.ch_freq;
        DBG(D_COMM, D_INFO, "comRAD: ch freq = %i.%i\n", freq/10, freq % 10);
      }
    }
  } else {
    DBG(D_COMM, D_WARN, "comRAD failed config request for cfg %i, err %i\n", cfg, res);
  }
}

static void comrad_rad_update_pending_cfg(bool force) {
  if (comrad.rad_cfg == 0) return;
  int i;
  for (i = 0; i < 8; i++) {
    if (comrad.rad_cfg & (1<<i)) {
      comrad_rad_update_cfg(i, force);
      return;
    }
  }
}

static void comrad_rad_return(void) {
  if (comrad.rad_cfg) {
    comrad_rad_update_pending_cfg(TRUE);
  } else {
    int res = NRF905_IMPL_rx();
    if (res != NRF905_OK) {
      DBG(D_COMM, D_WARN, "COMRAD goto rx failed %i\n", res);
    }
  }
}

static void comrad_rad_on_rx(u8_t *data, u8_t len) {
  // pretend to be phy and feed comm link layer comm_link_rx
  IF_DBG(D_RADIO, D_DEBUG) {
    print("COMRAD got packet, radio len %i\n", len);
    printbuf(IOSTD, data, len);
  }
  //comrad.stack.phy.up_rx_f(&comrad.stack, len-1, NULL);

  if (comrad.bad_link_simul) {
    if ((rand_next() & 0xff) <= comrad.bad_link_simul) {
      DBG(D_COMM, D_DEBUG, "COMRAD rx: dropped packet as debug\n");
      comrad_rad_return();
      return;
    }
  }

  int i;
  int res;
  // first byte is packet length - 1, so add 1 + 1 more for length byte
  for (i = 0; i < data[0]+1+1; i++) {
    res = comrad.stack.phy.up_rx_f(&comrad.stack, data[i], NULL);
    if (res != R_COMM_OK) {
      DBG(D_COMM, D_WARN, "COMRAD rx: lnk err %i\n", res);
      comrad_rad_return();
      return;
    }
  }
}

static void comrad_rad_on_tx(int res) {
  // goto rx after tx
  comrad_rad_return();
}

static void comrad_rad_on_cfg(int res) {
  // goto rx after config
  comrad_rad_return();
}

static void comrad_rad_on_err(nrf905_state state, int res) {
  // TODO
  // handle
  comrad_rad_return();
}

// radio supervision task, adjusts PA according to scheme
static void comrad_rad_super_task_f(u32_t arg, void *arg_p) {
  // the master is sending packets all the time, so here we use the comm stacks
  // build in link quality
#ifdef CONFIG_SPYBOT_APP_MASTER
  comrad.lqual = comm_squal(&comrad.stack);
  comm_clear_stats(&comrad.stack);
#endif
  // the client is receiving packets on a definite time slot when paired, so here
  // we calculate it manually
#ifdef CONFIG_SPYBOT_APP_CLIENT
  comrad.lqual = comrad_rad_client_rxed_packets_ratio();
#endif

  comrad.sum_lqual += comrad.lqual;
  comrad.lqual_samples++;
  // adjust PA according to link quality and pair loss
  if (comrad.lqual_samples >= COMRAD_RADIO_ADJUST_TICKS) {
    u16_t avg_lqual = comrad.sum_lqual / COMRAD_RADIO_ADJUST_TICKS;
    DBG(D_COMM, D_INFO, "COMRAD avg lqual: %i\n", avg_lqual);
    comrad.sum_lqual = 0;
    comrad.lqual_samples = 0;
#ifdef CONFIG_SPYBOT_ADJUST_PA
    // check if bad
    if (avg_lqual >= COMM_RADIO_LQUAL_PERFECT) {
      comrad.lost_pair = FALSE; // never mind, good enough
    }
    if (avg_lqual < (COMM_RADIO_LQUAL_GOOD + COMM_RADIO_LQUAL_PERFECT)/2 || comrad.lost_pair) {
      // need to amplify more
      DBG(D_COMM, D_WARN, "link is bad (%i), try to up PA (%i)\n", avg_lqual, comrad.pa);
      if (comrad.pa != NRF905_CFG_PA_PWR_10) {
        comrad.pa++;
        comrad_rad_request_cfg_update(COMRAD_RAD_CFG_PA);
      }
    }
    else if (avg_lqual > COMM_RADIO_LQUAL_PERFECT) {
      // amplify less
      DBG(D_COMM, D_WARN, "link is perfect (%i), try to lower PA (%i)\n", avg_lqual, comrad.pa);
      if (comrad.pa != NRF905_CFG_PA_PWR_m10) {
        comrad.pa--;
        comrad_rad_request_cfg_update(COMRAD_RAD_CFG_PA);
      }
    }
    comrad.lost_pair = FALSE;
#endif
  }
}

// flags that given radio setting is dirty and need to be changed
// as soon as radio is idle
// cfg may be any of COMRAD_RAD_CFG_*
static void comrad_rad_request_cfg_update(u8_t cfg) {
  comrad.rad_cfg |= (1<<cfg);
  if (NRF905_IMPL_listening()) {
    comrad_rad_update_pending_cfg(TRUE);
  }
}

//
// comm stack implementation functions
//

static void comrad_com_tick_task_f(u32_t arg, void *arg_p) {
  comm_tick(&comrad.stack, SYS_get_time_ms());
}

// called from nwk layer via task queue
// for further comm stack handling
static void comrad_com_task_on_pkt(u32_t arg, void* arg_p) {
  comm_arg *rx = (comm_arg *)arg_p;
  int res = comrad.post_nwk_comm_rx_up_f(&comrad.stack, rx);
  if (res != R_COMM_OK) {
    comrad_rad_return();
  }
}

static int comrad_com_nwk_rx(comm *com, comm_arg *rx) {
  // got packet from network layer, further processing in task context
  task *pkt_task = TASK_create(comrad_com_task_on_pkt, 0);
  ASSERT(pkt_task);
  TASK_run(pkt_task, 0, rx);
  return R_COMM_OK;
}

static int comrad_com_tx_flush(comm *comm, comm_arg* tx) {
  // acks do not need to check LBT
  if ((tx->flags & COMM_FLAG_ISACK_BIT) == 0) {
    // not an ack, check lbt
    if (!NRF905_IMPL_lbt_check_rts(COMM_RADIO_LBT_MS)) {
      // got carrier, return LBT failure
      DBG(D_COMM, D_DEBUG, "COMRAD tx: LBT fail\n");
      comrad_rad_return();
      return R_COMM_PHY_TRY_LATER;
    }
  }

  if (comrad.bad_link_simul) {
    if ((rand_next() & 0xff) <= comrad.bad_link_simul) {
      DBG(D_COMM, D_DEBUG, "COMRAD tx: dropped packet as debug\n");
      comrad_rad_return();
      return R_COMM_OK;
    }
  }

  unsigned char buf[COMM_RADIO_LNK_MAX_DATA];
  buf[0] = tx->len;
  memcpy(&buf[1], tx->data, tx->len+1);

  int res = NRF905_IMPL_tx(buf, tx->len+1);

  if (res == NRF905_OK) {
    return R_COMM_OK;
  } else if (res == NRF905_ERR_BUSY) {
    DBG(D_COMM, D_DEBUG, "COMRAD tx: rf905 busy\n");
    comrad_rad_return();
    return R_COMM_PHY_TRY_LATER;
  } else {
    DBG(D_COMM, D_DEBUG, "COMRAD tx: rf905 err %i\n", res);
    comrad_rad_return();
    return R_COMM_PHY_FAIL;
  }
}

//typedef comm_time (*comm_app_get_time_fn)(void);
static comm_time comrad_com_get_tick_count(void) {
  return SYS_get_time_ms();
}

static s16_t comrad_com_seq_delta(u16_t seqno_current, u16_t seqno_last_registered) {
  if (seqno_current < 0x100 && seqno_last_registered > 0xeff) {
    return seqno_current + 0x1000 - seqno_last_registered;
  } else if (seqno_current > 0xeff && seqno_last_registered < 0x100) {
    return -(seqno_last_registered + 0x1000 - seqno_current);
  } else {
    return seqno_current - seqno_last_registered;
  }
}

/* received stuff from rx->src, in here one might to call comm_app_reply  */
//typedef int (*comm_app_user_rx_fn)(comm *comm, comm_arg *rx,  unsigned short len, unsigned char *data);
static int comrad_com_rx_pkt(comm *comm, comm_arg *rx,  unsigned short len, unsigned char *data) {
  s32_t res = R_COMM_OK;
  comrad.cur_rx = rx;

  // detect packets that were resent but already received
  // keep track of latest 32 received packet sequence numbers
  s16_t seqd = comrad_com_seq_delta(rx->seqno, comrad.last_seqno);
  u8_t already_received = FALSE;

  // check if this packet already was received
  if (seqd <= 0) {
    if (-seqd < 32) {
      already_received = comrad.seq_mask & (1 << (-seqd));
      // register this packet as received
      comrad.seq_mask |= (1 << (-seqd));
    }
  } else {
    // shift received packet mask
    comrad.seq_mask <<= seqd;
    // register this packet as received
    comrad.seq_mask |= 1;
  }

  DBG(D_COMM, D_DEBUG, "COMrad rx: seq:%03x len:%02x src:%01x flg:%02x last:%03x delta:%i %s\n",
      rx->seqno, rx->len, rx->src, rx->flags, comrad.last_seqno, seqd, already_received ? "ALREADY RECEIVED": "FRESH");

  if (seqd > 0) {
    // got a new packet, register latest seqno
    comrad.last_seqno = rx->seqno;
  }

  if ((rx->flags & COMM_FLAG_REQACK_BIT) == 0) {
    // no ack required here, so return to rx directly
    comrad_rad_return();
  }
  // else, ack is required and stack will tx ack, which will end up in radio

  if (!already_received && (rx->flags & COMM_FLAG_RESENT_BIT) == 0) {
    comrad.sum_client_rx_pkts++;
  }

  // up to app
  APP_comrad_rx(rx, len, data, already_received);

  return res;
}

static void comrad_com_ack_pkt(comm *comm, comm_arg *rx, unsigned short seqno, unsigned short len, unsigned char *data) {
  DBG(D_COMM, D_DEBUG, "COMrad ack seqno:0x%03x\n", seqno);
  comrad_rad_return();
  APP_comrad_ack(rx, seqno, len, data);
}

/* invoked on transport info */
//typedef void (*comm_app_user_inf_fn)(comm *comm, comm_arg *rx);
static void comrad_com_tra_inf(comm *comm, comm_arg *rx) {
  DBG(D_DEBUG, D_COMM, "COMrad inf %02x\n", rx->data[0]);
}

static void comrad_com_alert(comm *comm, comm_addr addr, unsigned char type, unsigned short len, unsigned char *data) {
  DBG(D_DEBUG, D_COMM, "COMrad node alert addr:%02x type:%02x\n", addr, type);
}

/* invoked on error */
//typedef void (*comm_app_user_err_fn)(comm *comm, int err, unsigned short seqno, unsigned short len, unsigned char *data);
static void comrad_com_err(comm *comm, int err, unsigned short seqno, unsigned short len, unsigned char *data) {
  DBG(D_COMM, D_WARN, "COMrad ERR %i seqno:0x%03x\n", err, seqno);
  APP_comrad_err(seqno, err);
}

static void comrad_com_alloc(comm *c, void **data, void **arg, unsigned int size_data, unsigned int size_arg) {
  *data = &comrad.pool[comrad.pool_ix].data[0];
  *arg = &comrad.pool[comrad.pool_ix].rx;
  comrad.pool_ix++;
  comrad.pool_used++;
  if (comrad.pool_ix >= COMM_RADIO_POOLED_PACKETS) {
    comrad.pool_ix = 0;
  }
}

static void comrad_com_free(comm *c, void *data, void *arg) {
}

