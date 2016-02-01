package us.ihmc.robotiq.communication;

import net.wimpi.modbus.procimg.InputRegister;

public class RobotiqReadResponseFactory
{
   private final RobotiqReadResponse robotiqResponse = new RobotiqReadResponse();
   
   public void updateRobotiqResponse(InputRegister[] registers)
   {
      robotiqResponse.getGripperStatus().setRegisterValue(registers[0].toBytes()[0]);
      robotiqResponse.getObjectDetection().setRegisterValue(registers[0].toBytes()[1]);
      robotiqResponse.getFaultStatus().setRegisterValue(registers[1].toBytes()[0]);
      robotiqResponse.getFingerAPositionEcho().setRegisterValue(registers[1].toBytes()[1]);
      robotiqResponse.getFingerAPosition().setRegisterValue(registers[2].toBytes()[0]);
      robotiqResponse.getFingerACurrent().setRegisterValue(registers[2].toBytes()[1]);
      robotiqResponse.getFingerBPositionEcho().setRegisterValue(registers[3].toBytes()[0]);
      robotiqResponse.getFingerBPosition().setRegisterValue(registers[3].toBytes()[1]);
      robotiqResponse.getFingerBCurrent().setRegisterValue(registers[4].toBytes()[0]);
      robotiqResponse.getFingerCPositionEcho().setRegisterValue(registers[4].toBytes()[1]);
      robotiqResponse.getFingerCPosition().setRegisterValue(registers[5].toBytes()[0]);
      robotiqResponse.getFingerCCurrent().setRegisterValue(registers[5].toBytes()[1]);
      robotiqResponse.getScissorPositionEcho().setRegisterValue(registers[6].toBytes()[0]);
      robotiqResponse.getScissorPosition().setRegisterValue(registers[6].toBytes()[1]);
      robotiqResponse.getScissorCurrent().setRegisterValue(registers[7].toBytes()[0]);
   }
   
   public RobotiqReadResponse getResponse()
   {
      return robotiqResponse;
   }
}
