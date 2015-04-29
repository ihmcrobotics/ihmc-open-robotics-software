package us.ihmc.robotiq.communication;

import net.wimpi.modbus.msg.WriteMultipleRegistersRequest;
import net.wimpi.modbus.procimg.SimpleRegister;

public class RobotiqWriteRequestFactory
{
   private WriteMultipleRegistersRequest jamodRequest;
   private RobotiqWriteRequest robotiqRequest;
   
   public RobotiqWriteRequestFactory()
   {
      jamodRequest = new WriteMultipleRegistersRequest();
      jamodRequest.setProtocolID(0);
      jamodRequest.setReference(0);
      jamodRequest.setDataLength(8);
      
      robotiqRequest = new RobotiqWriteRequest();
   }
   
   public WriteMultipleRegistersRequest createActivationRequest()
   {
      // create appropriate Robotiq request
      
      // stuff into Jamod object
      
      //return Jamod object
      return jamodRequest;
   }
   
   public WriteMultipleRegistersRequest createOpenRequest()
   {
      // create appropriate Robotiq request
      
      // stuff into Jamod object
      
      //return Jamod object
      return jamodRequest;
   }
   
}
