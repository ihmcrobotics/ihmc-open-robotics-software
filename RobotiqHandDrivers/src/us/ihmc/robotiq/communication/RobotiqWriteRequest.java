package us.ihmc.robotiq.communication;

import us.ihmc.robotiq.communication.registers.ActionRequestRegister;
import us.ihmc.robotiq.communication.registers.ActionRequestRegister.rACT;
import us.ihmc.robotiq.communication.registers.ActionRequestRegister.rATR;
import us.ihmc.robotiq.communication.registers.ActionRequestRegister.rGTO;
import us.ihmc.robotiq.communication.registers.ActionRequestRegister.rMOD;
import us.ihmc.robotiq.communication.registers.FingerForceRegister;
import us.ihmc.robotiq.communication.registers.FingerPositionRequestRegister;
import us.ihmc.robotiq.communication.registers.FingerSpeedRegister;
import us.ihmc.robotiq.communication.registers.GripperOptionRegister;
import us.ihmc.robotiq.communication.registers.GripperOptionRegister.rICF;
import us.ihmc.robotiq.communication.registers.GripperOptionRegister.rICS;

public class RobotiqWriteRequest
{
   private final ActionRequestRegister actionRequest;
   private final GripperOptionRegister gripperOption;
   private final FingerPositionRequestRegister fingerAPositionRequest;
   private final FingerSpeedRegister fingerASpeed;
   private final FingerForceRegister fingerAForce;
   private final FingerPositionRequestRegister fingerBPositionRequest;
   private final FingerSpeedRegister fingerBSpeed;
   private final FingerForceRegister fingerBForce;
   private final FingerPositionRequestRegister fingerCPositionRequest;
   private final FingerSpeedRegister fingerCSpeed;
   private final FingerForceRegister fingerCForce;
   private final FingerPositionRequestRegister scissorPositionRequest;
   private final FingerSpeedRegister scissorSpeed;
   private final FingerForceRegister scissorForce;
   
   public RobotiqWriteRequest()
   {
      actionRequest = new ActionRequestRegister(rACT.DEACTIVATE_GRIPPER, rMOD.BASIC_MODE, rGTO.STOP, rATR.NORMAL);
      gripperOption = new GripperOptionRegister(rICF.INDIVIDUAL_FINGER_CONTROL, rICS.INDIVIDUAL_SCISSOR_CONTROL);
      fingerAPositionRequest = new FingerPositionRequestRegister(Finger.FINGER_A);
      fingerASpeed = new FingerSpeedRegister(Finger.FINGER_A);
      fingerAForce = new FingerForceRegister(Finger.FINGER_A);
      fingerBPositionRequest = new FingerPositionRequestRegister(Finger.FINGER_B);
      fingerBSpeed = new FingerSpeedRegister(Finger.FINGER_B);
      fingerBForce = new FingerForceRegister(Finger.FINGER_B);
      fingerCPositionRequest = new FingerPositionRequestRegister(Finger.FINGER_C);
      fingerCSpeed = new FingerSpeedRegister(Finger.FINGER_C);
      fingerCForce = new FingerForceRegister(Finger.FINGER_C);
      scissorPositionRequest = new FingerPositionRequestRegister(Finger.SCISSOR);
      scissorSpeed = new FingerSpeedRegister(Finger.SCISSOR);
      scissorForce = new FingerForceRegister(Finger.SCISSOR);
   }
   
   public void resetRequestFields()
   {
      actionRequest.resetRegister();
      gripperOption.resetRegister();
      fingerAPositionRequest.resetRegister();
      fingerASpeed.resetRegister();
      fingerAForce.resetRegister();
      fingerBPositionRequest.resetRegister();
      fingerBSpeed.resetRegister();
      fingerBForce.resetRegister();
      fingerCPositionRequest.resetRegister();
      fingerCSpeed.resetRegister();
      fingerCForce.resetRegister();
      scissorPositionRequest.resetRegister();
      scissorSpeed.resetRegister();
      scissorForce.resetRegister();
   }

   public ActionRequestRegister getActionRequest()
   {
      return actionRequest;
   }

   public GripperOptionRegister getGripperOption()
   {
      return gripperOption;
   }

   public FingerPositionRequestRegister getFingerAPositionRequest()
   {
      return fingerAPositionRequest;
   }

   public FingerSpeedRegister getFingerASpeed()
   {
      return fingerASpeed;
   }

   public FingerForceRegister getFingerAForce()
   {
      return fingerAForce;
   }

   public FingerPositionRequestRegister getFingerBPositionRequest()
   {
      return fingerBPositionRequest;
   }

   public FingerSpeedRegister getFingerBSpeed()
   {
      return fingerBSpeed;
   }

   public FingerForceRegister getFingerBForce()
   {
      return fingerBForce;
   }

   public FingerPositionRequestRegister getFingerCPositionRequest()
   {
      return fingerCPositionRequest;
   }

   public FingerSpeedRegister getFingerCSpeed()
   {
      return fingerCSpeed;
   }

   public FingerForceRegister getFingerCForce()
   {
      return fingerCForce;
   }

   public FingerPositionRequestRegister getScissorPositionRequest()
   {
      return scissorPositionRequest;
   }

   public FingerSpeedRegister getScissorSpeed()
   {
      return scissorSpeed;
   }

   public FingerForceRegister getScissorForce()
   {
      return scissorForce;
   }
}
