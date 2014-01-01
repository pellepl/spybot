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
// radio stack implementation api
//

static void comrad_rad_rx(u8_t *data, u8_t len);
static void comrad_rad_tx(int res);
static void comrad_rad_cfg(int res);
static void comrad_rad_err(nrf905_state state, int res);

static void comrad_rad_goto_rx(void);

//
// comm radio api
//

int COMRAD_send(u8_t *data, u16_t len, bool ack) {
#ifdef SECONDARY
  return comm_tx(&comrad.stack, 1, len, data, ack);
#else
  return comm_tx(&comrad.stack, 2, len, data, ack);
#endif
}

int COMRAD_reply(u8_t *data, u16_t len) {
  s32_t res = comm_reply(&comrad.stack, comrad.cur_rx, len, data);
  if (res < R_COMM_OK) DBG(D_COMM, D_WARN, "COMrad reply failed %i\n", res);
  return res;
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
  // callbacks
  NRF905_IMPL_init(comrad_rad_rx, comrad_rad_tx, comrad_rad_cfg, comrad_rad_err);
  res = NRF905_IMPL_conf(NULL, TRUE);
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
  comrad_rad_goto_rx();
}


static void comrad_rad_goto_rx(void) {
  int res = NRF905_IMPL_rx();
  if (res != NRF905_OK) {
    DBG(D_COMM, D_WARN, "COMRAD goto rx failed %i\n", res);
  }
}

//
// radio stack implementation functions
//

static void comrad_rad_rx(u8_t *data, u8_t len) {
  // pretend to be phy and feed comm link layer comm_link_rx
  IF_DBG(D_RADIO, D_DEBUG) {
    print("COMRAD got packet, len %i\n", len);
    int i;
    for (i = 0; i < len; i++) {
      print("%02x", data[i]);
    }
    print("\n");
  }
  comrad.stack.phy.up_rx_f(&comrad.stack, len-1, NULL);
  int i;
  int res;
  for (i = 0; i < len; i++) {
    res = comrad.stack.phy.up_rx_f(&comrad.stack, data[i], NULL);
    if (res != R_COMM_OK) {
      DBG(D_COMM, D_WARN, "COMRAD rx: lnk err %i\n", res);
      comrad_rad_goto_rx();
      return;
    }
  }
}

static void comrad_rad_tx(int res) {
  // goto rx after tx
  comrad_rad_goto_rx();
}

static void comrad_rad_cfg(int res) {
  // goto rx after config
  comrad_rad_goto_rx();
}

static void comrad_rad_err(nrf905_state state, int res) {
  // TODO
  // handle
  comrad_rad_goto_rx();
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
    comrad_rad_goto_rx();
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
      return R_COMM_PHY_TRY_LATER;
    }
  }

  unsigned char buf[COMM_RADIO_LNK_MAX_DATA+1];
  buf[0] = tx->len;
  memcpy(&buf[1], tx->data, tx->len+1);

  int res = NRF905_IMPL_tx(buf, tx->len+1);

  if (res == NRF905_OK) {
    return R_COMM_OK;
  } else if (res == NRF905_ERR_BUSY) {
    return R_COMM_PHY_TRY_LATER;
  } else {
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

  DBG(D_COMM, D_DEBUG, "COMrad pkt: seq:%03x len:%02x src:%01x flg:%02x last:%03x delta:%i %s\n",
      rx->seqno, rx->len, rx->src, rx->flags, comrad.last_seqno, seqd, already_received ? "ALREADY RECEIVED": "FRESH");

  if (seqd > 0) {
    // got a new packet, register latest seqno
    comrad.last_seqno = rx->seqno;
  }

  if ((rx->flags & COMM_FLAG_REQACK_BIT) == 0) {
    // no ack required here, so return to rx directly
    comrad_rad_goto_rx();
  }
  // else, ack is required and stack will tx ack, which will end up in radio
  // going to rx

  // todo
  // up to app

  return res;
}

static void comrad_com_ack_pkt(comm *comm, comm_arg *rx, unsigned short seqno, unsigned short len, unsigned char *data) {
  DBG(D_COMM, D_DEBUG, "COMrad ack seqno:0x%03x\n", seqno);
  comrad_rad_goto_rx();
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

