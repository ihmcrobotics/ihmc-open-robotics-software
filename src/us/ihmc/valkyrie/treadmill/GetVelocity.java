package us.ihmc.valkyrie.treadmill;

import us.ihmc.valkyrie.treadmill.MAVLinkTypes.MAV_MESSAGE;


public class GetVelocity extends MAV_MESSAGE
{

   public GetVelocity(int sysID, int compID)
   {
      this.mID = (byte) NetworkMsgType.GetVelocity.ordinal();
      this.len = 0;
      this.sID = (byte) sysID;
      this.cID = (byte) compID;
   }
}
