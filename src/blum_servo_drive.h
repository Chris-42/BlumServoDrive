#ifndef BLUM_DERVO_DRIVE_H
#define BLUM_DERVO_DRIVE_H

#include <Arduino.h>
#include "RF24.h"

#define SEND_INTERVAL 500

class Peer;

class BlumServoDrive {
private:
  enum PayloadTypes {
    BLUM_C4_ACK,
    BLUM_C4_ACT_REQ,
    BLUM_C4_POS_REQ,
    BLUM_C4_SYNC_BUTTON,
    BLUM_C4_SYNC_ACK,
    BLUM_C4_RESET,
    BLUM_C4_POWERON,
    BLUM_C10_DISCOVER,
    BLUM_C10_DISCOVER_ACK
  };

  enum States {
    BLUM_ACTIVE,
    BLUM_SYNC,
    BLUM_SYNC_WAIT_DISCOVER,
    BLUM_SYNC_WAIT_ANNOUNCE,
    BLUM_SYNC_WAIT_SEND_ACK,
    BLUM_SYNC_WAIT_REC_ACK,
    BLUM_MOVE,
    BLUM_MOVE_WAIT_ACK
  };

  typedef struct Payload {
    unsigned char data[16];
    uint8_t len;
  } Payload;

  uint64_t sync_cfg_addr = 0x534D554C42LL;
  //uint64_t switch_cfg_addr = 0x544D554C42LL;

  //                                             MyID____________                          seq_        PeerID__________  ??
  const unsigned char ch4_ack[13]             = {0x00, 0x00, 0x00, 0x00, 0x01, 0xA5, 0x00, 0x91, 0xdd, 0x00, 0x00, 0x00, 0x01}; 
  const unsigned char ch4_power_on[13]        = {0x00, 0x00, 0x00, 0x00, 0x01, 0x23, 0x00, 0xCE, 0xDD, 0xFC, 0x72, 0x42, 0x6C};
  const unsigned char ch4_activation_cmd[13]  = {0x00, 0x00, 0x00, 0x00, 0x01, 0x25, 0x00, 0x91, 0xdd, 0x00, 0x00, 0x00, 0x00};
  const unsigned char ch4_position_cmd[13]    = {0x00, 0x00, 0x00, 0x00, 0x01, 0x26, 0x00, 0x91, 0xdd, 0x00, 0x00, 0x00, 0x01}; // template set to close
  const unsigned char ch4_sync_button_cmd[13] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x27, 0x00, 0x91, 0xdd, 0x00, 0x00, 0x00, 0x01};
  const unsigned char ch4_sync_ack[14]        = {0x00, 0x00, 0x00, 0x02, 0x01, 0x03, 0x00, 0x91, 0xdd, 0x00, 0x00, 0x00, 0x01, 0x93};
  const unsigned char ch4_reset[13]           = {0x00, 0x00, 0x00, 0x00, 0x01, 0x42, 0x00, 0x91, 0xdd, 0x00, 0x00, 0x00, 0x01};
  //                                             MyID____________                          seq_                     ??
  const unsigned char ch10_discover[12]       = {0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x21, 0xdd, 0x02, 0x01, 0xe7};
  const unsigned char ch10_discover_ack[12]   = {0x00, 0x00, 0x00, 0x00, 0x01, 0x82, 0x00, 0x21, 0xdd, 0x02, 0x01, 0xe7};
  
  void payloadToHex(unsigned char* input, char* output, uint8_t inputSize );
  void printPayload(Payload &payload);
  uint64_t addressFromID(uint32_t id, unsigned char idx = 1);
  bool preparePayload(PayloadTypes type, Payload* payload, uint32_t peer);
  bool sendPacket(PayloadTypes type, Payload &ackPayload, uint32_t peerID, uint8_t pipe = 2);
  bool sendPacket(Payload &sendPayload, Payload &ackPayload, uint64_t address);
  void setAutoAckPayload(PayloadTypes type, uint32_t peerID, uint8_t pipe = 2);
  bool sendDiscover(Payload &ackPayload);
  bool handleDiscoverAck(Payload &ackPayload);
  void handleDiscover(Payload &recPayload);
  bool sendSynAck();
  void handleSynAck(Payload &recPayload);
  bool sendMovePos();
  void addPeer(uint32_t peerID);
  void removePeer(uint32_t peerID);
  Peer* findPeer(uint32_t peerID);
  int findPeerIdx(uint32_t peerID);
  static void radioLoopTask(void * parameters);
  void radioLoop();
  RF24* _radio;
  uint32_t _id;
  std::vector<Peer> _peers;
  TaskHandle_t _loophandle;
  uint32_t _last_state_transition;
  uint32_t _next_ack_send;
  uint8_t _sequence;
  //Payload _payload;
  uint8_t _pipe;
  uint32_t _currentPeerID;
  bool _requested_open;
  uint8_t _devID;
  uint8_t _retries;
  int _state;
public:
  enum event_t {SYNC_END, SYNC_IDENTIFY, OPEN, CLOSE, UNLINK};
  BlumServoDrive(RF24* radio, uint32_t id = 0xBAABCE);
  bool begin(uint8_t level = RF24_PA_MAX);
  bool storeConfig();
  bool loadConfig();
  int getPeerCount();
  uint8_t getState() {return _state;};
  uint32_t getPeerID(int idx);
  String getPeerName(int idx);
  void setPeerName(int idx, String name);
  bool sendMove(int idx, bool open);
  bool sendSyncIdentify(int idx);
  bool removePeerIdx(int idx);
  bool toggleState(int idx);
  bool pollState(int idx, uint8_t &state);
  uint8_t getState(int idx, uint8_t &state);
  bool startSync();
  void setCallback(void (*callback)(event_t event, int peer_idx));
private:
  void (*_event_callback)(event_t event, int peer_idx);
};

#endif