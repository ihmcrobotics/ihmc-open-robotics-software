package us.ihmc.valkyrie.treadmill;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import us.ihmc.valkyrie.treadmill.MAVLinkTypes.MAV_MESSAGE;


public class SetPWM extends MAV_MESSAGE
{

   public SetPWM(int sysID, int compID, double duty)
   {
      this.mID = (byte) NetworkMsgType.SetPWM.ordinal();
      this.len = 4;
      this.sID = (byte) sysID;
      this.cID = (byte) compID;
      int c = 0;
      
      byte[] byteArray = ByteBuffer.allocate(8).putDouble(duty).array();
      ByteBuffer.wrap(byteArray).order(ByteOrder.LITTLE_ENDIAN); //Ensure its little engine.
      
      for (byte b : byteArray)
      {
         this.payload[c++] = b;
      }
   }

   public SetPWM(MAV_MESSAGE msg)
   {
      this.mID = msg.getMessageID();
      this.len = msg.getLength();
      this.sID = msg.getSystemID();
      this.cID = msg.getComponentID();
      this.payload = msg.getPayload();
   }

   public double getPWM()
   {
//      return ByteTools.byteArray2Double(0, payload);
      return ByteBuffer.wrap(payload).getDouble();
   }
}
