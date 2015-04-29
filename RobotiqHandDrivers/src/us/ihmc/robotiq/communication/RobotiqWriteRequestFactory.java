package us.ihmc.robotiq.communication;

import net.wimpi.modbus.msg.WriteMultipleRegistersRequest;
import net.wimpi.modbus.procimg.Register;
import net.wimpi.modbus.procimg.SimpleRegister;

public class RobotiqWriteRequestFactory
{
   private WriteMultipleRegistersRequest jamodRequest;
   private SimpleRegister[] registers = new SimpleRegister[8];
   private RobotiqWriteRequest robotiqRequest;
   
   public RobotiqWriteRequestFactory()
   {
      jamodRequest = new WriteMultipleRegistersRequest();
      jamodRequest.setProtocolID(0);
      jamodRequest.setReference(0);
      jamodRequest.setDataLength(8);
      
      robotiqRequest = new RobotiqWriteRequest();
      
      for(int i = 0; i < registers.length; i++)
      {
         registers[i] = new SimpleRegister();
      }
   }
   
   public WriteMultipleRegistersRequest createActivationRequest()
   {
      // create appropriate Robotiq request
      
      // stuff into Jamod object
      packRequest();
      
      //return Jamod object
      return jamodRequest;
   }
   
   public WriteMultipleRegistersRequest createOpenRequest()
   {
      // create appropriate Robotiq request
      
      // stuff into Jamod object
      packRequest();
      
      //return Jamod object
      return jamodRequest;
   }
   
   private void packRequest()
   {
      registers[0].setValue((robotiqRequest.getActionRequest().getRegisterValue() << 8) | robotiqRequest.getGripperOption().getRegisterValue());
      registers[1].setValue((0x00 << 8) | robotiqRequest.getFingerAPositionRequest().getRegisterValue());
      registers[2].setValue((robotiqRequest.getFingerASpeed().getRegisterValue() << 8) | robotiqRequest.getFingerAForce().getRegisterValue());
      registers[3].setValue((robotiqRequest.getFingerBPositionRequest().getRegisterValue() << 8) | robotiqRequest.getFingerASpeed().getRegisterValue());
      registers[4].setValue((robotiqRequest.getFingerBForce().getRegisterValue() << 8) | robotiqRequest.getFingerCPositionRequest().getRegisterValue());
      registers[5].setValue((robotiqRequest.getFingerCSpeed().getRegisterValue() << 8) | robotiqRequest.getFingerCForce().getRegisterValue());
      registers[6].setValue((robotiqRequest.getScissorPositionRequest().getRegisterValue() << 8) | robotiqRequest.getScissorSpeed().getRegisterValue());
      registers[7].setValue((robotiqRequest.getScissorForce().getRegisterValue() << 8) | 0x00);
      
      jamodRequest.setRegisters(registers);
   }
}
