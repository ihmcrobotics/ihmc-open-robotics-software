package us.ihmc.robotiq.communication;

import net.wimpi.modbus.procimg.Register;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotiq.RobotiqGraspMode;
import us.ihmc.robotiq.communication.registers.ActionRequestRegister.rACT;
import us.ihmc.robotiq.communication.registers.ActionRequestRegister.rGTO;
import us.ihmc.robotiq.communication.registers.ByteSettableSimpleRegister;

public class RobotiqWriteRequestFactory
{
   private ByteSettableSimpleRegister[] registers = new ByteSettableSimpleRegister[8];
   private RobotiqWriteRequest robotiqRequest;
   
   public RobotiqWriteRequestFactory()
   {
      robotiqRequest = new RobotiqWriteRequest();
      
      for(int i = 0; i < registers.length; i++)
      {
         registers[i] = new ByteSettableSimpleRegister(0);
      }
   }
   
   public Register[] createActivationRequest()
   {
      robotiqRequest.resetRequestFields();
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
   
   public Register[] createWholeHandPositionRequest(RobotiqGraspMode graspMode, HandConfiguration fingerState)
   {
      robotiqRequest.getActionRequest().setRgto(rGTO.GO_TO);
      robotiqRequest.getFingerAPositionRequest().setFingerPosition(graspMode, fingerState);
      robotiqRequest.getFingerBPositionRequest().setFingerPosition(graspMode, fingerState);
      robotiqRequest.getFingerCPositionRequest().setFingerPosition(graspMode, fingerState);
      robotiqRequest.getScissorPositionRequest().setFingerPosition(graspMode, fingerState);
      
      packRequest();
      
      return registers;
   }
   
   public Register[] createFingersPositionRequest(RobotiqGraspMode graspMode, HandConfiguration fingerState)
   {
      robotiqRequest.getActionRequest().setRgto(rGTO.GO_TO);
      robotiqRequest.getFingerBPositionRequest().setFingerPosition(graspMode, fingerState);
      robotiqRequest.getFingerCPositionRequest().setFingerPosition(graspMode, fingerState);
      robotiqRequest.getScissorPositionRequest().setFingerPosition(graspMode, fingerState);
      
      packRequest();
      
      return registers;
   }
   
   public Register[] createThumbPositionRequest(RobotiqGraspMode graspMode, HandConfiguration fingerState)
   {
      robotiqRequest.getActionRequest().setRgto(rGTO.GO_TO);
      robotiqRequest.getFingerAPositionRequest().setFingerPosition(graspMode, fingerState);
      
      packRequest();
      
      return registers;
   }
   
   private void packRequest()
   {
      registers[0].setValue(robotiqRequest.getActionRequest().getRegisterValue(), robotiqRequest.getGripperOption().getRegisterValue());
      registers[1].setValue((byte)0x00, robotiqRequest.getFingerAPositionRequest().getRegisterValue());
      registers[2].setValue(robotiqRequest.getFingerASpeed().getRegisterValue(), robotiqRequest.getFingerAForce().getRegisterValue());
      registers[3].setValue(robotiqRequest.getFingerBPositionRequest().getRegisterValue(), robotiqRequest.getFingerBSpeed().getRegisterValue());
      registers[4].setValue(robotiqRequest.getFingerBForce().getRegisterValue(), robotiqRequest.getFingerCPositionRequest().getRegisterValue());
      registers[5].setValue(robotiqRequest.getFingerCSpeed().getRegisterValue(), robotiqRequest.getFingerCForce().getRegisterValue());
      registers[6].setValue(robotiqRequest.getScissorPositionRequest().getRegisterValue(), robotiqRequest.getScissorSpeed().getRegisterValue());
      registers[7].setValue(robotiqRequest.getScissorForce().getRegisterValue(), (byte)0x00);
   }
}
