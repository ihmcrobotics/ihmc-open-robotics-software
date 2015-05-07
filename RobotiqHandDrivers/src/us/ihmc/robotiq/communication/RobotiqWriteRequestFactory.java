package us.ihmc.robotiq.communication;

import net.wimpi.modbus.procimg.Register;
import net.wimpi.modbus.procimg.SimpleRegister;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.robotiq.RobotiqGraspMode;
import us.ihmc.robotiq.communication.registers.ActionRequestRegister.rACT;
import us.ihmc.robotiq.communication.registers.ActionRequestRegister.rGTO;

public class RobotiqWriteRequestFactory
{
   private SimpleRegister[] registers = new SimpleRegister[8];
   private RobotiqWriteRequest robotiqRequest;
   
   public RobotiqWriteRequestFactory()
   {
      robotiqRequest = new RobotiqWriteRequest();
      
      for(int i = 0; i < registers.length; i++)
      {
         registers[i] = new SimpleRegister(0);
      }
   }
   
   public Register[] createActivationRequest()
   {
      robotiqRequest.getActionRequest().setRact(rACT.ACTIVATE_GRIPPER);

      packRequest();
      
      return registers;
   }
   
   public Register[] createDeactivationRequest()
   {
      robotiqRequest.getActionRequest().setRact(rACT.DEACTIVATE_GRIPPER);
      robotiqRequest.getActionRequest().setRgto(rGTO.STOP);
      
      packRequest();
      
      return registers;
   }
   
   public Register[] createFingerPositionRequest(RobotiqGraspMode graspMode, FingerState fingerState)
   {
      robotiqRequest.getActionRequest().setRgto(rGTO.GO_TO);
      robotiqRequest.getFingerAPositionRequest().setFingerPosition(graspMode, fingerState);
      robotiqRequest.getFingerBPositionRequest().setFingerPosition(graspMode, fingerState);
      robotiqRequest.getFingerCPositionRequest().setFingerPosition(graspMode, fingerState);
      robotiqRequest.getScissorPositionRequest().setFingerPosition(graspMode, fingerState);
      
      packRequest();
      
      return registers;
   }
   
   private void packRequest()
   {
      registers[0].setValue((robotiqRequest.getActionRequest().getRegisterValue() << 8) | (robotiqRequest.getGripperOption().getRegisterValue() & 0xFF));
      registers[1].setValue((0x00 << 8) | (robotiqRequest.getFingerAPositionRequest().getRegisterValue() & 0xFF));
      registers[2].setValue((robotiqRequest.getFingerASpeed().getRegisterValue() << 8) | (robotiqRequest.getFingerAForce().getRegisterValue() & 0xFF));
      registers[3].setValue((robotiqRequest.getFingerBPositionRequest().getRegisterValue() << 8) | (robotiqRequest.getFingerBSpeed().getRegisterValue() & 0xFF));
      registers[4].setValue((robotiqRequest.getFingerBForce().getRegisterValue() << 8) | (robotiqRequest.getFingerCPositionRequest().getRegisterValue() & 0xFF));
      registers[5].setValue((robotiqRequest.getFingerCSpeed().getRegisterValue() << 8) | (robotiqRequest.getFingerCForce().getRegisterValue() & 0xFF));
      registers[6].setValue((robotiqRequest.getScissorPositionRequest().getRegisterValue() << 8) | (robotiqRequest.getScissorSpeed().getRegisterValue() & 0xFF));
      registers[7].setValue((robotiqRequest.getScissorForce().getRegisterValue() << 8) | 0x00);
   }
}
