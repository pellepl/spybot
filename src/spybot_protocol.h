/*
 * spybot_protocol.h
 *
 *  Created on: Dec 19, 2013
 *      Author: petera
 */

#ifndef SPYBOT_PROTOCOL_H_
#define SPYBOT_PROTOCOL_H_

typedef enum {
  CMD_DBG = 0,
  CMD_STATUS,
  CMD_MOTOR,
  CMD_ENTER_CONFIG,
  CMD_SET_CONFIG,
  CMD_STORE_CONFIG,
  CMD_LOAD_CONFIG,

} spybot_cmd;


#endif /* SPYBOT_PROTOCOL_H_ */
