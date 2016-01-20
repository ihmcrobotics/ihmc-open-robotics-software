package us.ihmc.valkyrie.treadmill;

import us.ihmc.valkyrie.treadmill.MAVLinkTypes.MAV_MESSAGE;


public class HeartBeat extends MAV_MESSAGE
{

   public HeartBeat(int sysID, int compID)
   {
      this.mID = (byte) NetworkMsgType.Heartbeat.ordinal();
      this.len = 0;
      this.sID = (byte) sysID;
      this.cID = (byte) compID;
   }

   public HeartBeat(MAV_MESSAGE msg)
   {
      this.mID = msg.getMessageID();
      this.len = msg.getLength();
      this.sID = msg.getSystemID();
      this.cID = msg.getComponentID();
      this.payload = msg.getPayload();
   }
}
