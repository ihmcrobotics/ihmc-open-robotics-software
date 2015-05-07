package us.ihmc.robotiq.communication;

import us.ihmc.robotiq.communication.registers.ActualFingerPositionRegister;
import us.ihmc.robotiq.communication.registers.FaultStatusRegister;
import us.ihmc.robotiq.communication.registers.FaultStatusRegister.gFLT;
import us.ihmc.robotiq.communication.registers.FingerCurrentRegister;
import us.ihmc.robotiq.communication.registers.FingerPositionRequestEchoRegister;
import us.ihmc.robotiq.communication.registers.GripperStatusRegister;
import us.ihmc.robotiq.communication.registers.GripperStatusRegister.gACT;
import us.ihmc.robotiq.communication.registers.GripperStatusRegister.gGTO;
import us.ihmc.robotiq.communication.registers.GripperStatusRegister.gIMC;
import us.ihmc.robotiq.communication.registers.GripperStatusRegister.gMOD;
import us.ihmc.robotiq.communication.registers.GripperStatusRegister.gSTA;
import us.ihmc.robotiq.communication.registers.ObjectDetectionRegister;
import us.ihmc.robotiq.communication.registers.ObjectDetectionRegister.gDTA;
import us.ihmc.robotiq.communication.registers.ObjectDetectionRegister.gDTB;
import us.ihmc.robotiq.communication.registers.ObjectDetectionRegister.gDTC;
import us.ihmc.robotiq.communication.registers.ObjectDetectionRegister.gDTS;

public class RobotiqReadResponse
{
   private GripperStatusRegister gripperStatus;
   private ObjectDetectionRegister objectDetection;
   private FaultStatusRegister faultStatus;
   private FingerPositionRequestEchoRegister fingerAPositionEcho;
   private ActualFingerPositionRegister fingerAPosition;
   private FingerCurrentRegister fingerACurrent;
   private FingerPositionRequestEchoRegister fingerBPositionEcho;
   private ActualFingerPositionRegister fingerBPosition;
   private FingerCurrentRegister fingerBCurrent;
   private FingerPositionRequestEchoRegister fingerCPositionEcho;
   private ActualFingerPositionRegister fingerCPosition;
   private FingerCurrentRegister fingerCCurrent;
   private FingerPositionRequestEchoRegister scissorPositionEcho;
   private ActualFingerPositionRegister scissorPosition;
   private FingerCurrentRegister scissorCurrent;
   
   public RobotiqReadResponse()
   {
      gripperStatus = new GripperStatusRegister(gACT.GRIPPER_RESET, gMOD.BASIC_MODE, gGTO.STOPPED, gIMC.GRIPPER_IN_RESET, gSTA.ALL_FINGERS_STOPPED);
      objectDetection = new ObjectDetectionRegister(gDTA.FINGER_A_AT_REQUESTED_POSITION, gDTB.FINGER_B_AT_REQUESTED_POSITION, gDTC.FINGER_C_AT_REQUESTED_POSITION, gDTS.SCISSOR_AT_REQUESTED_POSITION);
      faultStatus = new FaultStatusRegister(gFLT.NO_FAULT);
      fingerAPositionEcho = new FingerPositionRequestEchoRegister(Finger.FINGER_A);
      fingerAPosition = new ActualFingerPositionRegister(Finger.FINGER_A);
      fingerACurrent = new FingerCurrentRegister(Finger.FINGER_A);
      fingerBPositionEcho = new FingerPositionRequestEchoRegister(Finger.FINGER_B);
      fingerBPosition = new ActualFingerPositionRegister(Finger.FINGER_B);
      fingerBCurrent = new FingerCurrentRegister(Finger.FINGER_B);
      fingerCPositionEcho = new FingerPositionRequestEchoRegister(Finger.FINGER_C);
      fingerCPosition = new ActualFingerPositionRegister(Finger.FINGER_C);
      fingerCCurrent = new FingerCurrentRegister(Finger.FINGER_C);
      scissorPositionEcho = new FingerPositionRequestEchoRegister(Finger.SCISSOR);
      scissorPosition = new ActualFingerPositionRegister(Finger.SCISSOR);
      scissorCurrent = new FingerCurrentRegister(Finger.SCISSOR);
   }
   
   public GripperStatusRegister getGripperStatus()
   {
      return gripperStatus;
   }

   public void setGripperStatus(GripperStatusRegister gripperStatus)
   {
      this.gripperStatus = gripperStatus;
   }

   public ObjectDetectionRegister getObjectDetection()
   {
      return objectDetection;
   }

   public void setObjectDetection(ObjectDetectionRegister objectDetection)
   {
      this.objectDetection = objectDetection;
   }

   public FaultStatusRegister getFaultStatus()
   {
      return faultStatus;
   }

   public void setFaultStatus(FaultStatusRegister faultStatus)
   {
      this.faultStatus = faultStatus;
   }

   public FingerPositionRequestEchoRegister getFingerAPositionEcho()
   {
      return fingerAPositionEcho;
   }

   public void setFingerAPositionEcho(FingerPositionRequestEchoRegister fingerAPositionEcho)
   {
      this.fingerAPositionEcho = fingerAPositionEcho;
   }

   public ActualFingerPositionRegister getFingerAPosition()
   {
      return fingerAPosition;
   }

   public void setFingerAPosition(ActualFingerPositionRegister fingerAPosition)
   {
      this.fingerAPosition = fingerAPosition;
   }

   public FingerCurrentRegister getFingerACurrent()
   {
      return fingerACurrent;
   }

   public void setFingerACurrent(FingerCurrentRegister fingerACurrent)
   {
      this.fingerACurrent = fingerACurrent;
   }

   public FingerPositionRequestEchoRegister getFingerBPositionEcho()
   {
      return fingerBPositionEcho;
   }

   public void setFingerBPositionEcho(FingerPositionRequestEchoRegister fingerBPositionEcho)
   {
      this.fingerBPositionEcho = fingerBPositionEcho;
   }

   public ActualFingerPositionRegister getFingerBPosition()
   {
      return fingerBPosition;
   }

   public void setFingerBPosition(ActualFingerPositionRegister fingerBPosition)
   {
      this.fingerBPosition = fingerBPosition;
   }

   public FingerCurrentRegister getFingerBCurrent()
   {
      return fingerBCurrent;
   }

   public void setFingerBCurrent(FingerCurrentRegister fingerBCurrent)
   {
      this.fingerBCurrent = fingerBCurrent;
   }

   public FingerPositionRequestEchoRegister getFingerCPositionEcho()
   {
      return fingerCPositionEcho;
   }

   public void setFingerCPositionEcho(FingerPositionRequestEchoRegister fingerCPositionEcho)
   {
      this.fingerCPositionEcho = fingerCPositionEcho;
   }

   public ActualFingerPositionRegister getFingerCPosition()
   {
      return fingerCPosition;
   }

   public void setFingerCPosition(ActualFingerPositionRegister fingerCPosition)
   {
      this.fingerCPosition = fingerCPosition;
   }

   public FingerCurrentRegister getFingerCCurrent()
   {
      return fingerCCurrent;
   }

   public void setFingerCCurrent(FingerCurrentRegister fingerCCurrent)
   {
      this.fingerCCurrent = fingerCCurrent;
   }

   public FingerPositionRequestEchoRegister getScissorPositionEcho()
   {
      return scissorPositionEcho;
   }

   public void setScissorPositionEcho(FingerPositionRequestEchoRegister scissorPositionEcho)
   {
      this.scissorPositionEcho = scissorPositionEcho;
   }

   public ActualFingerPositionRegister getScissorPosition()
   {
      return scissorPosition;
   }

   public void setScissorPosition(ActualFingerPositionRegister scissorPosition)
   {
      this.scissorPosition = scissorPosition;
   }

   public FingerCurrentRegister getScissorCurrent()
   {
      return scissorCurrent;
   }

   public void setScissorCurrent(FingerCurrentRegister scissorCurrent)
   {
      this.scissorCurrent = scissorCurrent;
   }
}
