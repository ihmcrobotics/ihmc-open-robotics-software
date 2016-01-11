package us.ihmc.valkyrie.treadmill;

public class MAVLinkTypes
{

   public static final byte MAVLINK_HEADER = (byte) 0xFE;
   public static final int MAVLINK_MAX_PAYLOAD_LEN = 255; ///< Maximum payload length
   public static final int MAVLINK_CORE_HEADER_LEN = 5; ///< Length of core header (of the comm. layer): message length (1 byte) + message sequence (1 byte) + message system id (1 byte) + message component id (1 byte) + message type id (1 byte)
   public static final int MAVLINK_NUM_HEADER_BYTES = (MAVLINK_CORE_HEADER_LEN + 1); ///< Length of all header bytes, including core and checksum
   public static final int MAVLINK_NUM_CHECKSUM_BYTES = 2;
   public static final int MAVLINK_NUM_NON_PAYLOAD_BYTES = (MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES);
   public static final int MAVLINK_MAX_PACKET_LEN = (MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES); ///< Maximum packet length
   public static final int MAVLINK_MSG_ID_EXTENDED_MESSAGE = 255;
   public static final int MAVLINK_EXTENDED_HEADER_LEN = 14;
   public static final int MAVLINK_MAX_EXTENDED_PACKET_LEN = 65507;
   public static final int MAVLINK_MAX_EXTENDED_PAYLOAD_LEN = (MAVLINK_MAX_EXTENDED_PACKET_LEN - MAVLINK_EXTENDED_HEADER_LEN - MAVLINK_NUM_NON_PAYLOAD_BYTES);
   public static final int MAVLINK_MAX_FIELDS = 64;

   public static enum MAV_MESSAGE_TYPE
   {
      HEARTBEAT(0),
      SYS_STATUS(1),
      SYSTEM_TIME(2),
      PING(4),
      CHANGE_OPERATOR_CONTROL(5),
      CHANGE_OPERATOR_CONTROL_ACK(6),
      AUTH_KEY(7),
      SET_MODE(11),
      PARAM_REQUEST_READ(20),
      PARAM_REQUEST_LIST(21),
      PARAM_VALUE(22),
      PARAM_SET(23),
      GPS_RAW_INT(24),
      GPS_STATUS(25),
      SCALED_IMU(26),
      RAW_IMU(27),
      RAW_PRESSURE(28),
      SCALED_PRESSURE(29),
      ATTITUDE(30),
      ATTITUDE_QUATERNION(31),
      LOCAL_POSITION_NED(32),
      GLOBAL_POSITION_INT(33),
      RC_CHANNELS_SCALED(34),
      RC_CHANNELS_RAW(35),
      SERVO_OUTPUT_RAW(36),
      MISSION_REQUEST_PARTIAL_LIST(37),
      MISSION_WRITE_PARTIAL_LIST(38),
      MISSION_ITEM(39),
      MISSION_REQUEST(40),
      MISSION_SET_CURRENT(41),
      MISSION_CURRENT(42),
      MISSION_REQUEST_LIST(43),
      MISSION_COUNT(44),
      MISSION_CLEAR_ALL(45),
      MISSION_ITEM_REACHED(46),
      MISSION_ACK(47),
      SET_GPS_GLOBAL_ORIGIN(48),
      GPS_GLOBAL_ORIGIN(49),
      SET_LOCAL_POSITION_SETPOINT(50),
      LOCAL_POSITION_SETPOINT(51),
      GLOBAL_POSITION_SETPOINT_INT(52),
      SET_GLOBAL_POSITION_SETPOINT_INT(53),
      SAFETY_SET_ALLOWED_AREA(54),
      SAFETY_ALLOWED_AREA(55),
      SET_ROLL_PITCH_YAW_THRUST(56),
      SET_ROLL_PITCH_YAW_SPEED_THRUST(57),
      ROLL_PITCH_YAW_THRUST_SETPOINT(58),
      ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT(59),
      SET_QUAD_MOTORS_SETPOINT(60),
      SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST(61),
      NAV_CONTROLLER_OUTPUT(62),
      SET_QUAD_SWARM_LED_ROLL_PITCH_YAW_THRUST(63),
      STATE_CORRECTION(64),
      RC_CHANNELS(65),
      REQUEST_DATA_STREAM(66),
      DATA_STREAM(67),
      MANUAL_CONTROL(69),
      RC_CHANNELS_OVERRIDE(70),
      VFR_HUD(74);

      public int value = 0;

      private MAV_MESSAGE_TYPE(int mID)
      {
         this.value = mID;
      }

      public static MAV_MESSAGE_TYPE get(int mID)
      {
         for (MAV_MESSAGE_TYPE m : MAV_MESSAGE_TYPE.values())
            if (m.value == mID)
               return m;
         return null;
      }
   }

   public static enum MAV_FIELD_TYPE
   {
      MAVLINK_TYPE_CHAR,
      MAVLINK_TYPE_UINT8_T,
      MAVLINK_TYPE_INT8_T,
      MAVLINK_TYPE_UINT16_T,
      MAVLINK_TYPE_INT16_T,
      MAVLINK_TYPE_UINT32_T,
      MAVLINK_TYPE_INT32_T,
      MAVLINK_TYPE_UINT64_T,
      MAVLINK_TYPE_INT64_T,
      MAVLINK_TYPE_FLOAT,
      MAVLINK_TYPE_DOUBLE;
   }

   public static enum MAV_PARSE_STATE
   {
      UNINIT, IDLE, GOT_STX, GOT_SEQ, GOT_LENGTH, GOT_SYSID, GOT_COMPID, GOT_MSGID, GOT_PAYLOAD, GOT_CRC1;
   }

   public static class MAV_MESSAGE
   {
      protected byte len; // Length of payload
      protected byte seq; // Sequence of packet
      protected byte sID; // System ID
      protected byte cID; // Componnent ID
      protected byte mID; // Message ID
      protected byte payload[] = new byte[MAVLINK_MAX_PAYLOAD_LEN]; // Message Payload 
      protected byte checksumLow = 0; // last-1 byte of packet 
      protected byte checksumHigh = 0; // last byte of packet  
      protected boolean initialized;

      //Setters
      public void setLength(int len)
      {
         this.len = (byte) len;
      }

      public void setSequence(int seq)
      {
         this.seq = (byte) seq;
      }

      public void setSystemID(int sID)
      {
         this.sID = (byte) sID;
      }

      public void setComponentID(int cID)
      {
         this.cID = (byte) cID;
      }

      public void setMessageID(int mID)
      {
         this.mID = (byte) mID;
      }

      public void setPayloadByte(int i, byte val)
      {
         this.payload[i] = val;
      }

      public void setChecksumLow(byte checksumLow)
      {
         this.checksumLow = checksumLow;
      }

      public void setChecksumHigh(byte checksumHigh)
      {
         this.checksumHigh = checksumHigh;
      }

      //Getters
      public byte getLength()
      {
         return this.len;
      }

      public byte getSequence()
      {
         return this.seq;
      }

      public byte getSystemID()
      {
         return this.sID;
      }

      public byte getComponentID()
      {
         return this.cID;
      }

      public byte getMessageID()
      {
         return this.mID;
      }

      public byte getPayloadByte(int i)
      {
         return this.payload[i];
      }

      public byte[] getPayload()
      {
         return this.payload;
      }

      public byte getChecksumLow()
      {
         return this.checksumLow;
      }

      public byte getChecksumHigh()
      {
         return this.checksumHigh;
      }

      /**
       * Packs the {@link MAVLinkMessage} into a byte array to be transmitted.
       * This method must return a byte array containing the following structure:<br>
       * { systemID(1 byte), componentID(1 byte), messageID(1 byte), payload(n bytes) }<br>
       * @return an array of size lebgth only containing systemID, componentID, and messageID..
       */
      public byte[] pack()
      {
         char msgLength = (char) (MAVLINK_NUM_NON_PAYLOAD_BYTES + this.len);
         byte[] msg = new byte[msgLength];
         msg[0] = MAVLINK_HEADER;
         msg[1] = this.len;
         msg[2] = this.seq;
         msg[3] = this.sID;
         msg[4] = this.cID;
         msg[5] = this.mID;
         for (int i = 0; i < this.len; i++)
            msg[i + 6] = this.payload[i];
         this.generateCRC();
         msg[msgLength - 2] = this.checksumLow;
         msg[msgLength - 1] = this.checksumHigh;
         return msg;
      }

      private void generateCRC()
      {
         char crc = CheckSum.crc_init();
         crc = CheckSum.crc_accumulate(this.len, crc);
         crc = CheckSum.crc_accumulate(this.seq, crc);
         crc = CheckSum.crc_accumulate(this.sID, crc);
         crc = CheckSum.crc_accumulate(this.cID, crc);
         crc = CheckSum.crc_accumulate(this.mID, crc);
         for (int i = 0; i < this.len; i++)
            crc = CheckSum.crc_accumulate(this.payload[i], crc);
         this.checksumLow = (byte) crc;
         this.checksumHigh = (byte) (crc >> 8);
      }

      public char getCRC()
      {
         return (char) (((checksumHigh & 0x0FF) << 8) | (checksumLow & 0x0FF));
      }
   }

   public static class MAV_STATUS
   {
      public byte msg_received = 0; ///< Number of received messages
      public byte buffer_overrun = 0; ///< Number of buffer overruns
      public byte parse_error = 0; ///< Number of parse errors
      public MAV_PARSE_STATE parse_state = MAV_PARSE_STATE.IDLE; ///< Parsing state machine
      public byte packet_idx = 0; ///< Index in current packet
      public byte current_rx_seq = 0; ///< Sequence number of last packet received
      public byte current_tx_seq = 0; ///< Sequence number of last packet sent
      public char packet_rx_success_count = 0; ///< Received packets
      public char packet_rx_drop_count = 0; ///< Number of packet drops

      /**
      * Copies the parameters of the msg into this one;
      * @param msg-message to be copied
      */
      public void clone(MAV_STATUS sts)
      {
         msg_received = sts.msg_received;
         buffer_overrun = sts.buffer_overrun;
         parse_error = sts.parse_error;
         parse_state = sts.parse_state;
         packet_idx = sts.packet_idx;
         current_rx_seq = sts.current_rx_seq;
         current_tx_seq = sts.current_tx_seq;
         packet_rx_success_count = sts.packet_rx_success_count;
         packet_rx_drop_count = sts.packet_rx_drop_count;
      }
   }

}