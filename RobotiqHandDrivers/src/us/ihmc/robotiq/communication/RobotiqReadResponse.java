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
   
   public void setAll(RobotiqReadResponse response)
   {
      this.gripperStatus.setRegisterValue(response.getGripperStatus().getRegisterValue());
      this.objectDetection.setRegisterValue(response.getObjectDetection().getRegisterValue());
      this.faultStatus.setRegisterValue(response.getFaultStatus().getRegisterValue());
      this.fingerAPositionEcho.setRegisterValue(response.getFingerAPositionEcho().getRegisterValue());
      this.fingerAPosition.setRegisterValue(response.getFingerAPosition().getRegisterValue());
      this.fingerACurrent.setRegisterValue(response.getFingerACurrent().getRegisterValue());
      this.fingerBPositionEcho.setRegisterValue(response.getFingerBPositionEcho().getRegisterValue());
      this.fingerBPosition.setRegisterValue(response.getFingerBPosition().getRegisterValue());
      this.fingerBCurrent.setRegisterValue(response.getFingerBCurrent().getRegisterValue());
      this.fingerCPositionEcho.setRegisterValue(response.getFingerCPositionEcho().getRegisterValue());
      this.fingerCPosition.setRegisterValue(response.getFingerCPosition().getRegisterValue());
      this.fingerCCurrent.setRegisterValue(response.getFingerCCurrent().getRegisterValue());
      this.scissorPositionEcho.setRegisterValue(response.getScissorPositionEcho().getRegisterValue());
      this.scissorPosition.setRegisterValue(response.getScissorPosition().getRegisterValue());
      this.scissorCurrent.setRegisterValue(response.getScissorCurrent().getRegisterValue());
   }
   
   @Override
   public String toString()
   {
      String ret = "";
      
      ret += "Gripper Status = " + this.gripperStatus.getRegisterValue() + "\n";
      ret += "Object Detection = " + this.objectDetection.getRegisterValue() + "\n";
      ret += "Fault Status = " + this.faultStatus.getRegisterValue() + "\n";
      ret += "Finger A Requested Position = " + this.fingerAPositionEcho.getRegisterValue() + "\n";
      ret += "Finger A Position = " + this.fingerAPosition.getRegisterValue() + "\n";
      ret += "Finger A Current = " + this.fingerACurrent.getRegisterValue() + "\n";
      ret += "Finger B Requested Position = " + this.fingerBPositionEcho.getRegisterValue() + "\n";
      ret += "Finger B Position = " + this.fingerBPosition.getRegisterValue() + "\n";
      ret += "Finger B Current = " + this.fingerBCurrent.getRegisterValue() + "\n";
      ret += "Finger C Requested Position = " + this.fingerCPositionEcho.getRegisterValue() + "\n";
      ret += "Finger C Position = " + this.fingerCPosition.getRegisterValue() + "\n";
      ret += "Finger C Current = " + this.fingerCCurrent.getRegisterValue() + "\n";
      ret += "Scissor Requested Position = " + this.scissorPositionEcho.getRegisterValue() + "\n";
      ret += "Scissor Position = " + this.scissorPosition.getRegisterValue() + "\n";
      ret += "Scissor Current = " + this.scissorCurrent.getRegisterValue() + "\n";
      
      return ret;
   }
   
   @Override
   public boolean equals(Object other)
   {
      if(other instanceof RobotiqReadResponse)
      {
         boolean ret = true;
         
         other = (RobotiqReadResponse) other;
         
         ret &= this.gripperStatus.equals(((RobotiqReadResponse) other).getGripperStatus());
         ret &= this.objectDetection.equals(((RobotiqReadResponse) other).getObjectDetection());
         ret &= this.faultStatus.equals(((RobotiqReadResponse) other).getFaultStatus());
         ret &= this.fingerAPositionEcho.equals(((RobotiqReadResponse) other).getFingerAPositionEcho());
         ret &= this.fingerAPosition.equals(((RobotiqReadResponse) other).getFingerAPosition());
         ret &= this.fingerACurrent.equals(((RobotiqReadResponse) other).getFingerACurrent());
         ret &= this.fingerBPositionEcho.equals(((RobotiqReadResponse) other).getFingerBPositionEcho());
         ret &= this.fingerBPosition.equals(((RobotiqReadResponse) other).getFingerBPosition());
         ret &= this.fingerBCurrent.equals(((RobotiqReadResponse) other).getFingerBCurrent());
         ret &= this.fingerCPositionEcho.equals(((RobotiqReadResponse) other).getFingerCPositionEcho());
         ret &= this.fingerCPosition.equals(((RobotiqReadResponse) other).getFingerCPosition());
         ret &= this.fingerCCurrent.equals(((RobotiqReadResponse) other).getFingerCCurrent());
         ret &= this.scissorPositionEcho.equals(((RobotiqReadResponse) other).getScissorPositionEcho());
         ret &= this.scissorPosition.equals(((RobotiqReadResponse) other).getScissorPosition());
         ret &= this.scissorCurrent.equals(((RobotiqReadResponse) other).getScissorCurrent());
         
         return ret;
      }
      else
         return false;
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
