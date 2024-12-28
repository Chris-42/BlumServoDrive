#ifndef BLUM_DERVO_DRIVE_H
#define BLUM_DERVO_DRIVE_H

#include <Arduino.h>
#include "RF24.h"

#define SEND_INTERVAL 500

class Peer {
  private:
    String _name;
    uint8_t _devID;
    uint32_t _id;
    uint8_t _state;
  public:
    Peer(uint32_t id, String name = "") {
      _id = id;
      _name = name;
      _devID = 1;
      _state = 0;
    }

    ~Peer() {}

    uint32_t getID() const {
      return _id;
    }
    
    void setID(uint32_t id) {
      _id = id;
    }
    
    String getName() const {
      return _name;
    }
    
    void setName(String name) {
      _name = name;
    }

    uint8_t getState() {
      return _state;
    }

    void setState(uint8_t state) {
      _state = state;
    }
};

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
  enum event_t {SYNC_END, SYNC_IDENTIFY, OPEN, CLOSE, NEWPEER, UNLINK};
  BlumServoDrive(RF24* radio, uint32_t id = 0xBAABCE);
  bool begin(uint8_t level = RF24_PA_MAX);
  bool storeConfig();
  bool loadConfig();
  int getPeerCount();
  int findPeerIdx(uint32_t peerID);
  uint32_t getPeerID(uint idx);
  uint8_t getState() {return _state;};
  String getPeerName(uint idx);
  void setPeerName(uint idx, String name);
  bool sendMove(uint idx, bool open);
  bool sendSyncIdentify(uint idx);
  bool removePeerIdx(uint idx);
  bool toggleState(uint idx);
  bool pollPeerState(uint idx, uint8_t &state);
  uint8_t getPeerState(uint idx);
  bool startSync();
  void setCallback(void (*callback)(event_t event, int peer_idx));
private:
  void (*_event_callback)(event_t event, int peer_idx);
};

#endif