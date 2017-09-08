package us.ihmc.valkyrie.treadmill;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import us.ihmc.valkyrie.treadmill.MAVLinkTypes.MAV_MESSAGE;


public class SetVelocity extends MAV_MESSAGE
{

   public SetVelocity(int sysID, int compID, double v)
   {
      this.mID = (byte) NetworkMsgType.SetVelocity.ordinal();
      this.len = 4;
      this.sID = (byte) sysID;
      this.cID = (byte) compID;
      int c = 0;
      
      byte[] byteArray = ByteBuffer.allocate(8).putDouble(v).array();
      ByteBuffer.wrap(byteArray).order(ByteOrder.LITTLE_ENDIAN); //Ensure its little engine.
      
      for (byte b : byteArray)
      {
         this.payload[c++] = b;
      }
   }

   public SetVelocity(MAV_MESSAGE msg)
   {
      this.mID = msg.getMessageID();
      this.len = msg.getLength();
      this.sID = msg.getSystemID();
      this.cID = msg.getComponentID();
      this.payload = msg.getPayload();
   }

   public double getVelocity()
   {
      return ByteBuffer.wrap(payload).getDouble();
   }
}
