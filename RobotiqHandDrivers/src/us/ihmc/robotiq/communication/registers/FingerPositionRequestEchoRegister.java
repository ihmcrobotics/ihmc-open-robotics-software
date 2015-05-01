package us.ihmc.robotiq.communication.registers;

import us.ihmc.robotiq.communication.Finger;
import us.ihmc.robotiq.communication.InvalidFingerException;

public class FingerPositionRequestEchoRegister implements RobotiqRegister
{
   private final int index;
   private byte positionEcho;
   
   public FingerPositionRequestEchoRegister(Finger finger)
   {
      switch(finger)
      {
         case FINGER_A:
            index = 3;
            break;
         case FINGER_B:
            index = 6;
            break;
         case FINGER_C:
            index = 9;
            break;
         case SCISSOR:
            index = 12;
            break;
         default:
            throw new InvalidFingerException(finger);
      }
   }
   
   public void setPosition(byte positionEcho)
   {
      this.positionEcho = positionEcho;
   }

   @Override
   public byte getRegisterValue()
   {
      return positionEcho;
   }

   @Override
   public int getRegisterIndex()
   {
      return index;
   }

}
