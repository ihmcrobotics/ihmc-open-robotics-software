package us.ihmc.robotiq.communication;

import us.ihmc.robotiq.communication.registers.ActualFingerPositionRegister;
import us.ihmc.robotiq.communication.registers.FaultStatusRegister;
import us.ihmc.robotiq.communication.registers.FingerCurrentRegister;
import us.ihmc.robotiq.communication.registers.FingerPositionRequestEchoRegister;
import us.ihmc.robotiq.communication.registers.GripperStatusRegister;
import us.ihmc.robotiq.communication.registers.ObjectDetectionRegister;

public class RobotiqWriteResponse
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
