#include "blum_servo_drive.h"
#include <Preferences.h>

#ifdef BLUM_DEBUG
 #define Serialprint(...) Serial.print(__VA_ARGS__)
 #define Serialprintln(...) Serial.println(__VA_ARGS__)
 #define Serialprintf(...) Serial.printf(__VA_ARGS__)
#else
 #define Serialprint(...)
 #define Serialprintln(...)
 #define Serialprintf(...)
#endif

#define SYNC_WAIT_TIMEOUT 20000

BlumServoDrive::BlumServoDrive(RF24* radio, uint32_t id) {
  _radio = radio;
  _id = id;
  _devID = 0x94;
  _event_callback = nullptr;
}

bool BlumServoDrive::begin(uint8_t level) {
  _radio->setPALevel(level);  // RF24_PA_MAX is default.
  _radio->setDataRate(RF24_1MBPS);
  _radio->enableDynamicPayloads();
  _radio->disableCRC();
  _radio->enableAckPayload();
  _radio->stopListening();
  _radio->setChannel(4);
  //_radio->openReadingPipe(0, addressFromID(_id, 0));
  _radio->closeReadingPipe(0);
  _radio->openReadingPipe(1, addressFromID(_id, 1));
  _radio->openReadingPipe(2, 0x02);
  _radio->startListening();

  loadConfig();

  _sequence = 1;
  _state = BLUM_ACTIVE;
  _last_state_transition = millis();
  xTaskCreatePinnedToCore(this->radioLoopTask, "BlumLoop", 4000, (void*)this, 5, &_loophandle, ARDUINO_RUNNING_CORE);

  for(int i = 0; i < _peers.size(); ++i) {
    uint8_t state;
    pollPeerState(i, state);
  }
  
  return true;
}

//function to convert ascii char[] to hex-string (char[])
void BlumServoDrive::payloadToHex(unsigned char* input, char* output, uint8_t inputSize ) {
  int loop=0;
  int i=0;
  while (loop < inputSize) {
    sprintf((char*)(output + i), "%02X ", input[loop]);
    loop += 1;
    i += 3;
  }
}

void BlumServoDrive::printPayload(Payload &payload) {
  char str[payload.len*3+2];
  str[0] = ' ';
  str[payload.len*3] = '\0';
  payloadToHex(payload.data, &str[1], payload.len);
  Serialprintln(str);
}

uint64_t BlumServoDrive::addressFromID(uint32_t id, unsigned char idx) {
  // 0xDD LL NN MM 0X
  return 0xdd00000000LL | ((uint64_t)(id & 0xFF)<<24)
                        | ((uint64_t)((id >> 8) & 0xFF)<<16)
                        | ((uint64_t)((id >> 16) & 0xFF)<<8)
                        | (uint64_t)idx;
}

bool BlumServoDrive::preparePayload(PayloadTypes type, Payload* payload, uint32_t peerID) {
  switch(type) {
    case BLUM_C4_ACK:
      payload->len = sizeof(ch4_ack);
      memcpy(payload->data, ch4_ack, payload->len);
      break;
    case BLUM_C4_SYNC_ACK:
      payload->len = sizeof(ch4_sync_ack);
      memcpy(payload->data, ch4_sync_ack, payload->len);
      payload->data[13] = _devID;
      break;
    case BLUM_C4_ACT_REQ:
      payload->len = sizeof(ch4_activation_cmd);
      memcpy(payload->data, ch4_activation_cmd, payload->len);
      break;
    case BLUM_C4_POS_REQ:
      payload->len = sizeof(ch4_position_cmd);
      memcpy(payload->data, ch4_position_cmd, payload->len);
      break;
    case BLUM_C4_SYNC_BUTTON:
      payload->len = sizeof(ch4_sync_button_cmd);
      memcpy(payload->data, ch4_sync_button_cmd, payload->len);
      payload->data[12] = _devID;
      break;
    case BLUM_C10_DISCOVER:
      payload->len = sizeof(ch10_discover);
      memcpy(payload->data, ch10_discover, payload->len);
      break;
    case BLUM_C10_DISCOVER_ACK:
      payload->len = sizeof(ch10_discover_ack);
      memcpy(payload->data, ch10_discover_ack, payload->len);
      break;
    case BLUM_C4_RESET:
      payload->len = sizeof(ch4_reset);
      memcpy(payload->data, ch4_reset, payload->len);
      break;
    case BLUM_C4_POWERON:
      payload->len = sizeof(ch4_power_on);
      memcpy(payload->data, ch4_power_on, payload->len);
      break;
    default:
      Serialprintf(F("payload type %d not handled\r\n"), type);
      return false;
  }
  payload->data[0] = (_id >> 16) & 0xFF;
  payload->data[1] = (_id >> 8) & 0xFF;
  payload->data[2] = _id & 0xFF;
  if(type == BLUM_C4_POWERON) {
    return true;
  }
  payload->data[7] = _sequence++;
  if(payload->len > 12) {
    payload->data[9] = (peerID >> 16) & 0XFF;
    payload->data[10] = (peerID >> 8) & 0XFF;
    payload->data[11] = peerID & 0xFF;
  }
  return true;
}

bool BlumServoDrive::sendPacket(Payload &sendPayload, Payload &ackPayload, uint64_t address) {
  printPayload(sendPayload);
  Serialprintf(F("send : %10llX(%d)\r\n"), address, millis());
  _radio->stopListening();
  _radio->openWritingPipe(address);
  bool success = _radio->write(sendPayload.data, sendPayload.len);      // transmit 
  if(success) {
    Serialprint("got send ack");
    delay(1);
    ackPayload.len = _radio->getDynamicPayloadSize(); // see if there was a ack payload
    if(ackPayload.len) {
       _radio->read(ackPayload.data, ackPayload.len);
       Serialprint(" with ack payload");
       printPayload(ackPayload);
    } else {
      Serialprintln(" without payload");
    }
  } else {
    Serialprintln("no ack");
  }
  _radio->startListening();
  return success;
}

bool BlumServoDrive::sendPacket(PayloadTypes type, Payload &ackPayload, uint32_t peerID, uint8_t pipe) {
  Payload payload;
  uint64_t address;
  preparePayload(type, &payload, peerID);
  if(type == BLUM_C10_DISCOVER) {
    Serialprintf(F("send payload to sync_cfg"));
    address = sync_cfg_addr;
  } else {
    Serialprintf(F("send payload to %06X.%d"), peerID, pipe);
    address = addressFromID(peerID, pipe);
  }
  return sendPacket(payload, ackPayload, address);
}

void BlumServoDrive::setAutoAckPayload(PayloadTypes type, uint32_t peerID, uint8_t pipe) {
  Payload autoAckPayload;
  preparePayload(type, &autoAckPayload, peerID);
  Serialprintf("set autoack payload on %d:", pipe);
  printPayload(autoAckPayload);
  _radio->stopListening();
  _radio->writeAckPayload(pipe, autoAckPayload.data, autoAckPayload.len);
  _radio->startListening();
}

bool BlumServoDrive::loadConfig() {
  Preferences prefs;
  if(prefs.begin("config")) {
    size_t len = prefs.getBytesLength("PeerIDs");
    if(len > 0) {
      uint32_t peerids[len/4];
      prefs.getBytes("PeerIDs", peerids, len);
      for(int i = 0; i < len/4; ++i) {
        char n[16];
        sprintf(n, "name_%04X", peerids[i]);
        String name = prefs.getString(n);
        if(name.isEmpty()) {
          _peers.push_back(Peer(peerids[i]));
        } else {
          _peers.push_back(Peer(peerids[i], name));
        }
      }
    }
    prefs.end();
  } else {
    return false;
  }
  return true;
}

bool BlumServoDrive::storeConfig() {
  Preferences prefs;
  int len = _peers.size();
  if(!prefs.begin("config")) {
    return false;
  }  
  prefs.clear();
  uint32_t peerIDs[len];
  for(int i = 0; i < _peers.size(); ++i) {
    peerIDs[i] = _peers.at(i).getID();
  }
  if(prefs.putBytes("PeerIDs", peerIDs, len*4) != len*4) {
    prefs.clear();
    prefs.end();
    return false;
  }
  for(int i = 0; i < len; ++i) {
    char n[16];
    sprintf(n, "name_%04X", peerIDs[i]);
    String name = _peers.at(i).getName();
    if(!name.isEmpty()) {
      if(!prefs.putString(n, name)) {
        prefs.clear();
        prefs.end();
        return false;
      }
    }
  }
  prefs.end();
  return true;
}

int BlumServoDrive::getPeerCount(){
  return _peers.size();
}

uint32_t BlumServoDrive::getPeerID(uint idx) {
  if(idx > _peers.size()) {
    return 0;
  }
  return _peers.at(idx).getID();
}

void BlumServoDrive::setPeerName(uint idx, String name) {
  if(idx >= _peers.size()) {
    return;
  }
  _peers.at(idx).setName(name);
}

String BlumServoDrive::getPeerName(uint idx) {
  if(idx >= _peers.size()) {
    return String("out of range");
  }
  return _peers.at(idx).getName();
}

bool BlumServoDrive::removePeerIdx(uint idx) {
  if(idx >= _peers.size()) {
    return false;
  }
  Payload ackPayload;
  Serialprintf(F("send reset to %06X.2"),  _peers.at(idx).getID());
  sendPacket(BLUM_C4_RESET, ackPayload, _peers.at(idx).getID(), 2);
  removePeer(_peers.at(idx).getID());
  return true;
}

bool BlumServoDrive::sendSyncIdentify(uint idx) {
  if(idx >= _peers.size()) {
    return false;
  }
  Payload ackPayload;
  Serialprintf(F("send sync identify to %06X.2"),  _peers.at(idx).getID());
  return sendPacket(BLUM_C4_SYNC_BUTTON, ackPayload, _peers.at(idx).getID(), 2);
}

bool BlumServoDrive::pollPeerState(uint idx, uint8_t &state) {
  if(idx >= _peers.size()) {
    return false;
  }
  Payload ackPayload;
  Serialprintf(F("send power on to %06X.2"),  _peers.at(idx).getID());
  bool success = sendPacket(BLUM_C4_POWERON, ackPayload, _peers.at(idx).getID(), 2);
  if(success && ackPayload.len) {
    _peers.at(idx).setState(ackPayload.data[0]);
    return true;
  }
  return false;
}

uint8_t BlumServoDrive::getPeerState(uint idx) {
  if(idx >= _peers.size()) {
    return 0;
  }
  return _peers.at(idx).getState();
}

bool BlumServoDrive::sendMovePos() {
  Payload ackPayload, sendPayload;
  preparePayload(BLUM_C4_POS_REQ, &sendPayload, _currentPeerID);
  Serialprintf("open: %d\r\n", _requested_open);
  if(_requested_open) {
    sendPayload.data[12] = 0x10;
  }
  bool success = sendPacket(sendPayload, ackPayload, addressFromID(_currentPeerID, 2));
  if(success) {
    findPeer(_currentPeerID)->setState(sendPayload.data[12]);
  }
  return success;
}


bool BlumServoDrive::sendMove(uint idx, bool open) {
  if(idx >= _peers.size()) {
    return false;
  }
  _currentPeerID = _peers.at(idx).getID();
  _requested_open = open;
  _retries = 5;
  _state = BLUM_MOVE;
  _last_state_transition = millis();
  return true;
}

bool BlumServoDrive::toggleState(uint idx) {
  if(idx >= _peers.size()) {
    return false;
  }
  uint8_t state = _peers.at(idx).getState();
  if(state == 0x01) {
    return sendMove(idx, true);
  } else if(state == 0x10) {
    return sendMove(idx, false);
  }
  return true;
}

bool BlumServoDrive::startSync() {
  _state = BLUM_SYNC;
  _retries = 5;
  _last_state_transition = millis();
  return true;
}

void BlumServoDrive::setCallback(void (*callback)(event_t event, int peer_idx)) {
  _event_callback = callback;
}

bool BlumServoDrive::sendDiscover(Payload &ackPayload) {
  _radio->stopListening();
  _radio->flush_rx();
  _radio->flush_tx();
  _radio->setChannel(10);
  _radio->openReadingPipe(0, sync_cfg_addr);
  Serialprintln(F("Sending discover"));
  return sendPacket(BLUM_C10_DISCOVER, ackPayload, 0);
}

bool BlumServoDrive::handleDiscoverAck(Payload &ackPayload) {
  Serialprint(F("got ack "));
  if(!ackPayload.len) {
    Serialprintln(F("without payload, retry"));
    return false;
  }
  Serialprintln(F("with payload"));
  if((ackPayload.len < 11) || (ackPayload.data[5] != 0x82)) {
    Serialprintln(F("no 82 in payload, retry"));
    return false;
  }
  uint32_t peerID = (uint32_t)ackPayload.data[0]<<16 | (uint32_t)ackPayload.data[1]<<8 | ackPayload.data[2];
  addPeer(peerID);
  _currentPeerID = peerID;
  return true;
}

void BlumServoDrive::handleDiscover(Payload &recPayload) {
  Serialprintln(F("sync data"));
  _currentPeerID = (uint32_t)recPayload.data[0]<<16 | (uint32_t)recPayload.data[1]<<8 | recPayload.data[2];
  _radio->stopListening();
  _radio->flush_tx();
  _radio->flush_rx();
  _radio->setChannel(4);
  _radio->closeReadingPipe(0);
  _radio->startListening();
}

bool BlumServoDrive::sendSynAck() {
  if(!_currentPeerID) {
    Serialprintln(F("no peer, not sending syn ack"));
    return false;
  }
  Payload ackPayload;
  Serialprintln(F("send syn ack packet"));
  bool success = sendPacket(BLUM_C4_SYNC_ACK, ackPayload, _currentPeerID, 2);
  if(!success) {
    return false;
  }
  if(success && (ackPayload.len > 5) && (ackPayload.data[0] != ackPayload.data[1])) {
    //_sent_sync_ack = true;
    return true;
  }
  if(success) {
    return sendPacket(BLUM_C4_SYNC_BUTTON, ackPayload,  _currentPeerID, 2);
  }
  return false;
}

void BlumServoDrive::handleSynAck(Payload &recPayload) {
  Serialprintln(F("sync_ack data"));
  if(recPayload.len) {
    uint32_t peerID = (uint32_t)recPayload.data[0]<<16 | (uint32_t)recPayload.data[1]<<8 | recPayload.data[2];
    _currentPeerID = peerID;
    if(recPayload.data[5] == 0x03) { // seems to be syn ack
      addPeer(peerID);
    }
  }
}

int BlumServoDrive::findPeerIdx(uint32_t peerID) {
  auto p = std::find_if(_peers.begin(), _peers.end(), [&peerID] (const Peer& p) { return p.getID() == peerID; });
  if(p == _peers.end()) {
    return -1;
  }
  return std::distance(_peers.begin(), p);
}

Peer* BlumServoDrive::findPeer(uint32_t peerID) {
  int idx = findPeerIdx(peerID);
  if(idx < 0) {
    return nullptr;
  }
  return &(_peers.at(idx));
}

void BlumServoDrive::addPeer(uint32_t peerID) {
  if(findPeer(peerID)) {
    Serialprintln(F("Peer already paired"));
  } else {
    Serialprint(F("new Peer "));
    Serialprintln(peerID, HEX);
    Peer* p = new Peer(peerID);
    _peers.push_back(*p);
    storeConfig();
    if(_event_callback) {
      _event_callback(NEWPEER, findPeerIdx(peerID));
    }
  }
}

void BlumServoDrive::removePeer(uint32_t peerID) {
  if(findPeer(peerID)) {
    std::erase_if(_peers, [&peerID] (const Peer& p) { return p.getID() == peerID; });
    storeConfig();
    if(_event_callback) {
      _event_callback(UNLINK, findPeerIdx(peerID));
    }
  } else {
    Serialprintln(F("Peer not found"));
  }
}

void BlumServoDrive::radioLoop() {
  //unsigned char payload[32];
  bool got_packet = false;
  uint8_t pipe;
  Payload recPayload;
  if(_radio->available(&pipe)) { // is there a payload? get the pipe number that recieved it
    got_packet = true;
    recPayload.len = _radio->getDynamicPayloadSize(); // get the size of the payload
    _radio->read(recPayload.data, recPayload.len);    // fetch payload from FIFO
    Serialprintf(F("received on .%d (%lu) "), pipe, millis());
    printPayload(recPayload);
  }
  Payload ack_payload;
  static uint32_t last_send = 0;
  uint32_t ti = millis();
  switch(_state) {
    case BLUM_SYNC:
      if((ti - last_send) > SEND_INTERVAL) {
        last_send = ti;
        if(sendDiscover(ack_payload)) {
          Serialprint(F("got ack "));
          if(ack_payload.len) {
            Serialprintln(F("with payload"));
            if((ack_payload.len > 11) && (ack_payload.data[5] == 0x82)) {
              Serialprintln(F(" 82 in payload, adding peer"));
              uint32_t peerID = (uint32_t)ack_payload.data[0]<<16 | (uint32_t)ack_payload.data[1]<<8 | ack_payload.data[2];
              addPeer(peerID);
              _state = BLUM_SYNC_WAIT_SEND_ACK;
              _last_state_transition = ti;
            } else {
              Serialprintln(F("no 82 in payload, listen"));
              _radio->stopListening();
              _radio->setChannel(4);
              _radio->closeReadingPipe(0);
              _radio->openReadingPipe(1, addressFromID(_id, 1));
              _radio->openReadingPipe(2, 0x02);
              setAutoAckPayload(BLUM_C4_SYNC_ACK, _currentPeerID, 2);
              _state = BLUM_SYNC_WAIT_REC_ACK;
              _last_state_transition = ti;
            }
          } else {
            Serialprintln(F("with empty payload, retry"));
          }
        } else {
          if(!_retries) {
            Serialprintln(F("got no ack, wait for discover on ch10"));
            setAutoAckPayload(BLUM_C10_DISCOVER_ACK, 0, 0);
            _state = BLUM_SYNC_WAIT_DISCOVER;
            _last_state_transition = millis();
          }
          _retries--;
        }
      }
      break;
    case BLUM_SYNC_WAIT_DISCOVER:
      if(got_packet && (_pipe == 0)) {
        handleDiscover(recPayload);
        got_packet = false;
        _state = BLUM_SYNC_WAIT_REC_ACK;
        _retries = 5;
        last_send = ti;
        _last_state_transition = millis();
      }
      if((millis() - _last_state_transition) > SYNC_WAIT_TIMEOUT) {
        Serialprintln("Sync timeout");
        _radio->stopListening();
        _radio->flush_tx();
        _radio->flush_rx();
        _radio->setChannel(4);
        _radio->closeReadingPipe(0);
        _radio->openReadingPipe(1, addressFromID(_id, 1));
        _radio->openReadingPipe(2, 0x02);
        _radio->startListening();
        _state = BLUM_ACTIVE;
        _last_state_transition = millis();
        if(_event_callback) {
          _event_callback(SYNC_END, -1);
        }
      }
      break;
    case BLUM_SYNC_WAIT_REC_ACK:
      if(got_packet) {
        Serialprint("got SYN Ack: ");
        printPayload(recPayload);
        handleSynAck(recPayload);
        if(_event_callback) {
          _event_callback(SYNC_END, findPeerIdx(_currentPeerID));
        }
        got_packet = false;
        _state = BLUM_ACTIVE;
        _last_state_transition = ti;
      }
      break;
    case BLUM_SYNC_WAIT_SEND_ACK:
      if((ti - last_send) > SEND_INTERVAL) {
        last_send = ti;
        if(_retries) {
          _retries--;
          if(sendSynAck()) {
            Serialprintln("sync ack ok");
            if(_event_callback) {
              _event_callback(SYNC_END, findPeerIdx(_currentPeerID));
            }
            _state = BLUM_ACTIVE;
          }
        } else {
          Serialprintln("sync ack retries exceeded");
            if(_event_callback) {
              _event_callback(SYNC_END, -1);
            }
          _state = BLUM_ACTIVE;
        }
      }
      break;
    case BLUM_MOVE_WAIT_ACK:
      if((ti - last_send) > SEND_INTERVAL) {
        last_send = ti;
        if(_retries) {
          _retries--;
          if(sendMovePos()) {
            Serialprintln("move pos ack ok");
            _state = BLUM_ACTIVE;
            if(_event_callback) {
              _event_callback(_requested_open ? OPEN : CLOSE, findPeerIdx(_currentPeerID));
            }
          }
        } else {
          Serialprintln("move retries exceeded");
          _state = BLUM_ACTIVE;
        }
      }
      break;
    case BLUM_MOVE:
      if((ti - last_send) > SEND_INTERVAL) {
        last_send = ti;
        if(_retries) {
          _retries--;
          if(sendPacket(BLUM_C4_ACT_REQ, ack_payload, _currentPeerID, 2)) {
            Serialprintln("move wake ack ok");
            _retries = 5;
            last_send -= SEND_INTERVAL;
            _state = BLUM_MOVE_WAIT_ACK;
          }
        } else {
          Serialprintln("move retries exceeded");
          _state = BLUM_ACTIVE;
        }
      }
      break;
    case BLUM_ACTIVE:
      if(got_packet) {
        if((recPayload.len = 13) && (recPayload.data[5] == 0xA5)) {
          Serialprintln("got A5 ack");
          got_packet = false;
          break;
        }
        uint32_t peerID = (uint32_t)recPayload.data[0]<<16 | (uint32_t)recPayload.data[1]<<8 | recPayload.data[2];
        addPeer(peerID);
        Payload ackPayload;
        Serialprintln(F("send c4 ack"));
        bool success = sendPacket(BLUM_C4_ACK, ackPayload, peerID, 2);
        if (success && (recPayload.data[5] == 0x25)) { // Sync Move Wakeup 
          Serialprintln(F("Sync Move wake received. "));
          got_packet = false;
        } else if (recPayload.data[5] == 0x26) {
          Serialprint(F("Sync Move requested from: "));
          Serialprintln(peerID, HEX);
          Serialprint(F(" to: "));
          Serialprintln(recPayload.data[12], HEX);
          findPeer(peerID)->setState(recPayload.data[12]);
          if(_event_callback) {
            _event_callback(recPayload.data[12] == 01 ? OPEN : CLOSE, findPeerIdx(peerID));
          }
          got_packet = false;
        } else if (recPayload.data[5] == 0x27) {
          Serialprint(F("Sync Status requested from."));
          Serialprintln(peerID, HEX);
          addPeer(peerID);
          if(_event_callback) {
            _event_callback(SYNC_IDENTIFY, findPeerIdx(peerID));
          }
          got_packet = false;
        } else if (recPayload.data[5] == 0x42) {
          Serialprint("erasing peer");
          Serialprintln(peerID, HEX);
          removePeer(peerID);
          got_packet = false;
        }
      }
      break;
    default:
        Serialprintf("!!! unhandled state %d\r\n", _state);
      break;
  }
  if(got_packet) {
    Serialprint("unhandled packet: ");
    printPayload(recPayload);
  }

  static int last_state = 0;
  if(_state != last_state) {
    Serialprintf("state change %d -> %d ch:%d\r\n", last_state, _state, _radio->getChannel());
    _last_state_transition = millis();
    last_state = _state;
  }
}

void BlumServoDrive::radioLoopTask(void * parameters) {
  BlumServoDrive* p = static_cast<BlumServoDrive*>(parameters);
  while(1) {
    p->radioLoop();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}