package us.ihmc.robotiq.communication;

import net.wimpi.modbus.msg.WriteMultipleRegistersRequest;
import net.wimpi.modbus.procimg.SimpleRegister;
import us.ihmc.robotiq.communication.registers.ActionRequestRegister;
import us.ihmc.robotiq.communication.registers.GripperOptionRegister;
import us.ihmc.robotiq.communication.registers.RobotiqRegister;

public class RobotiqWriteRequestFactory
{
   private WriteMultipleRegistersRequest request;
   private SimpleRegister[] registers = new SimpleRegister[8];
   private RobotiqRegister actionRequestRegister, gripperOptionRegister, fingerAPostionRequestRegister, fingerASpeedRegister,
                           fingerAForceRegister, fingerBPositionRequestRegister, fingerBSpeedRegister, fingerBForceRegister,
                           fingerCPositionRequestRegister, fingerCSpeedRegister, fingerCForceRegister, scissorPositionRequestRegister,
                           scissorSpeedRegister, scissorForceRegister;
   
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
   
   private void initialize()
   {
      actionRequestRegister = new ActionRequestRegister();
      gripperOptionRegister = new GripperOptionRegister();
   }
   
   public WriteMultipleRegistersRequest createActivationRequest()
   {
      registers[0].setValue(new byte[]{0x01, 0x00});
      
      request.setRegisters(registers);
      
      return request;
   }
   
   
   
}
