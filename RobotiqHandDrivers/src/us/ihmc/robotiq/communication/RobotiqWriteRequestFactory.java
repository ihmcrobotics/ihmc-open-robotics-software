package us.ihmc.robotiq.communication;

import net.wimpi.modbus.msg.WriteMultipleRegistersRequest;
import net.wimpi.modbus.procimg.SimpleRegister;

public class RobotiqWriteRequestFactory
{
   private WriteMultipleRegistersRequest request;
   private SimpleRegister[] registers = new SimpleRegister[8];
   
   public RobotiqWriteRequestFactory()
   {
      request = new WriteMultipleRegistersRequest();
      request.setProtocolID(0);
      request.setReference(0);
      request.setDataLength(8);
      
      for(int i = 0; i < registers.length; i++)
      {
         registers[i] = new SimpleRegister(0);
      }
   }
   
   public WriteMultipleRegistersRequest createActivationRequest()
   {
      registers[0].setValue(new byte[]{0x01, 0x00});
      
      request.setRegisters(registers);
      
      return request;
   }
   
   
   
}
