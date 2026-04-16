#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <SerialTransfer.h>
#include <type_traits>

/**
 * @brief Bidirectional UART link helper built on top of SerialTransfer.
 *
 * The class is designed for two ESP32 boards (controller/peer) that exchange small,
 * high frequency packets such as joystick positions or button states.  It
 * provides framing, CRC validation (via SerialTransfer), heartbeats to detect
 * link loss, and an optional callback interface so applications can react to
 * incoming packets or error conditions from their main loop without blocking.
 */
class Comms {
 public:
  /** Maximum payload bytes that can be stored in a single packet. */
  static constexpr size_t kMaxPayloadSize = 160;  //!< Supports full ASCII RPC lines

  /**
   * @brief Frame types sent across the wire.
   */
  enum class FrameType : uint8_t {
    kData = 0x01,       //!< Application data payload
    kHeartbeat = 0x7E,  //!< Periodic keep-alive frame
    kAck = 0x7F         //!< Reserved for future acknowledgement support
  };

  /**
   * @brief High-level link status derived from heartbeats and serial health.
   */
  enum class LinkState : uint8_t {
    kIdle,          //!< Link has been initialised but no heartbeat seen yet
    kActive,        //!< Heartbeats and payload packets are flowing
    kTimedOut,      //!< Heartbeats stalled longer than the configured timeout
    kSerialError    //!< SerialTransfer reported a framing/CRC failure
  };

  /**
   * @brief Error categories surfaced to the application layer.
   */
  enum class Error : uint8_t {
    kNone = 0,          //!< No error detected
    kPayloadTooLarge,   //!< Caller attempted to send a packet bigger than the TX buffer
    kInvalidPayload,    //!< Caller supplied an invalid payload pointer/size combination
    kHeartbeatLost,     //!< Heartbeat timeout elapsed without receiving a frame
    kSerialTransfer     //!< SerialTransfer reported a framing or CRC failure
  };

  /**
   * @brief Container for an incoming packet.
   */
  struct Packet {
    uint8_t channel = 0;          //!< Logical channel identifier (0-255)
    uint8_t size = 0;             //!< Number of bytes stored in data[]
    uint8_t data[kMaxPayloadSize];  //!< Packet payload bytes
  };

  /** Callback invoked when a data packet arrives. */
  using PacketCallback = void (*)(const Packet& packet, void* context);
  /** Callback invoked when a heartbeat frame is received. */
  using HeartbeatCallback = void (*)(void* context);
  /** Callback invoked when an error state is detected. */
  using ErrorCallback = void (*)(Error error, int8_t rawStatus, void* context);

  /**
   * @brief Optional callbacks that the application can register.
   */
  struct Callbacks {
    PacketCallback onPacket = nullptr;       //!< Called for application data packets
    HeartbeatCallback onHeartbeat = nullptr; //!< Called whenever a heartbeat is received
    ErrorCallback onError = nullptr;         //!< Called when the link enters an error state
    void* context = nullptr;                 //!< User supplied context pointer
  };

  /**
   * @brief Statistics counters that help monitor link health.
   */
  struct Stats {
    uint32_t packetsTx = 0;        //!< Number of payload frames transmitted
    uint32_t packetsRx = 0;        //!< Number of payload frames received
    uint32_t heartbeatsTx = 0;     //!< Number of heartbeat frames transmitted
    uint32_t heartbeatsRx = 0;     //!< Number of heartbeat frames received
    uint32_t crcErrors = 0;        //!< Count of CRC or framing errors from SerialTransfer
    uint32_t payloadErrors = 0;    //!< Packets discarded because of malformed headers
  };

  Comms();

  /**
   * @brief Configure the link and underlying UART.
   *
   * @param serial  HardwareSerial instance to use.
   * @param rxPin   GPIO used for UART RX on the ESP32.
   * @param txPin   GPIO used for UART TX on the ESP32.
   * @param baud    UART baudrate (defaults to 115200).
   * @return true if the serial interface and SerialTransfer layer were initialised.
   */
  bool begin(HardwareSerial& serial, int rxPin, int txPin, uint32_t baud = 115200);

  /**
   * @brief Stop the link and release the UART.
   */
  void end();

  /**
   * @brief Install user callbacks for packet, heartbeat and error events.
   */
  void setCallbacks(const Callbacks& callbacks);

  /**
   * @brief Set the heartbeat transmit interval.
   *
   * The default is 50ms which keeps latency low without saturating the UART.
   */
  void setHeartbeatInterval(uint32_t intervalMs);

  /**
   * @brief Set the heartbeat timeout.  When no heartbeat or payload arrives
   * within this time the link transitions to kTimedOut and the error callback
   * is fired with kHeartbeatLost.
   */
  void setHeartbeatTimeout(uint32_t timeoutMs);

  /**
   * @brief Send a pre-built packet structure.
   */
  bool send(const Packet& packet);

  /**
   * @brief Send a payload with an explicit channel identifier.
   */
  bool send(uint8_t channel, const uint8_t* data, size_t length);

  /**
   * @brief Convenience helper that serialises a POD struct as packet payload.
   */
  template <typename T>
  bool sendStruct(uint8_t channel, const T& payload) {
    static_assert(std::is_trivially_copyable<T>::value,
                  "Payload structs must be trivially copyable");
    return send(channel, reinterpret_cast<const uint8_t*>(&payload), sizeof(T));
  }

  /**
   * @brief Poll the link for incoming frames and manage heartbeats.
   *
   * This function must be called regularly from loop() on both boards to keep
   * the protocol fully non-blocking.
   */
  void update();

  /**
   * @return The current high-level link state.
   */
  LinkState linkState() const { return linkState_; }

  /**
   * @return true if the link is exchanging heartbeats within the timeout window.
   */
  bool isActive() const { return linkState_ == LinkState::kActive; }

  /**
   * @return Milliseconds since boot that the last packet was transmitted.
   */
  uint32_t lastTxTime() const { return lastTxMs_; }

  /**
   * @return Milliseconds since boot that the last packet was received.
   */
  uint32_t lastRxTime() const { return lastRxMs_; }

  /**
   * @return Copy of the current statistics counters.
   */
  Stats stats() const { return stats_; }

  /**
   * @return The last error seen by the link.
   */
  Error lastError() const { return lastError_; }

  /**
   * @brief Clear the stored error state.
   */
  void clearError();

 private:
  static constexpr size_t kFrameOverhead = 3;  //!< bytes for type/channel/length header

  bool sendFrame(FrameType type, uint8_t channel, const uint8_t* payload, size_t length);
  void handleIncoming(uint16_t size);
  void handleErrorStatus(int16_t status);
  void sendHeartbeat(uint32_t now);
  void updateLinkState(uint32_t now);
  void recordSuccessfulTransfer();

  HardwareSerial* serial_ = nullptr;
  SerialTransfer transfer_;
  Callbacks callbacks_{};
  bool started_ = false;
  uint32_t heartbeatIntervalMs_ = 50;
  uint32_t heartbeatTimeoutMs_ = 250;
  uint32_t lastHeartbeatSentMs_ = 0;
  uint32_t lastHeartbeatSeenMs_ = 0;
  uint32_t lastTxMs_ = 0;
  uint32_t lastRxMs_ = 0;
  Stats stats_{};
  Error lastError_ = Error::kNone;
  int16_t lastTransferStatus_ = 0;
  LinkState linkState_ = LinkState::kIdle;
};

