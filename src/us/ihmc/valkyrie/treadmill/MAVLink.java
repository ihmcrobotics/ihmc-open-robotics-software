package us.ihmc.valkyrie.treadmill;
/* Java implementation of the MAVLink
 * communication protocols
 *
 *
 */

import us.ihmc.valkyrie.treadmill.MAVLinkTypes.MAV_MESSAGE;
import us.ihmc.valkyrie.treadmill.MAVLinkTypes.MAV_PARSE_STATE;
import us.ihmc.valkyrie.treadmill.MAVLinkTypes.MAV_STATUS;

public class MAVLink
{

   public static byte HEADER = (byte) 0xFE;

   public MAV_MESSAGE lastMsg;
   private MAV_MESSAGE currentMsg;
   private MAV_STATUS status = new MAV_STATUS();

   public float getDropRate()
   {
      float dropped = status.packet_rx_drop_count;
      float total = status.packet_rx_drop_count + status.packet_rx_success_count;
      return dropped / total * 100f;
      //(float) (pckCount / (inSerial + 1) * 100) - 100;
   }

   public float getSuccessCount()
   {
      return status.packet_rx_success_count;
   }

   public void reset()
   {
      currentMsg = new MAV_MESSAGE();
      status.packet_idx = 0;
      status.parse_state = MAV_PARSE_STATE.IDLE;
   }

   public boolean parseChar(byte data)
   {

      //    System.out.println(status.parse_state.toString());

      switch (status.parse_state)
      {
      case UNINIT:
      case IDLE:
         if ((HEADER == data))
         {
            currentMsg = new MAV_MESSAGE();
            status.packet_idx++;
            status.parse_state = MAV_PARSE_STATE.GOT_STX;
         }
         return false;

      case GOT_STX:
         currentMsg.len = data;
         status.packet_idx++;
         status.parse_state = MAV_PARSE_STATE.GOT_LENGTH;
         return false;

      case GOT_LENGTH:
         currentMsg.seq = data;
         status.packet_idx++;
         status.parse_state = MAV_PARSE_STATE.GOT_SEQ;
         return false;

      case GOT_SEQ:
         currentMsg.sID = data;
         status.packet_idx++;
         status.parse_state = MAV_PARSE_STATE.GOT_SYSID;
         return false;

      case GOT_SYSID:
         currentMsg.cID = data;
         status.packet_idx++;
         status.parse_state = MAV_PARSE_STATE.GOT_COMPID;
         return false;

      case GOT_COMPID:
         currentMsg.mID = data;
         status.packet_idx++;
         status.parse_state = MAV_PARSE_STATE.GOT_MSGID;
         if (currentMsg.len == 0)
            status.parse_state = MAV_PARSE_STATE.GOT_PAYLOAD;
         return false;

      case GOT_MSGID:
         currentMsg.payload[status.packet_idx - 6] = data;
         status.packet_idx++;
         if (status.packet_idx == (currentMsg.len + 6))
         {
            status.parse_state = MAV_PARSE_STATE.GOT_PAYLOAD;
         }
         return false;

      case GOT_PAYLOAD:
         currentMsg.checksumLow = data;
         status.packet_idx++;
         status.parse_state = MAV_PARSE_STATE.GOT_CRC1;
         return false;

      case GOT_CRC1:
         currentMsg.checksumHigh = data;
         char receivedCRC = (char) (((currentMsg.checksumHigh & 0x00FF) << 8) | (currentMsg.checksumLow & 0x00FF));
         char calculatedCRC = currentMsg.getCRC();
         if (calculatedCRC != receivedCRC)
         {
            status.packet_rx_drop_count++;
            reset();
            status.parse_state = MAV_PARSE_STATE.IDLE;
            return false;
         }
         status.parse_state = MAV_PARSE_STATE.IDLE;
         currentMsg.initialized = true;
      }

      status.current_rx_seq = currentMsg.seq;
      status.packet_rx_success_count++;
      status.msg_received = currentMsg.mID;
      lastMsg = currentMsg;
      reset();
      return true;
   }

}