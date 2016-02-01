package us.ihmc.robotiq.communication.registers;

import us.ihmc.robotiq.communication.Finger;
import us.ihmc.robotiq.communication.InvalidFingerException;

public class FingerPositionRequestEchoRegister extends RobotiqInputRegister
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
   
   @Override
   public byte getRegisterValue()
   {
      return positionEcho;
   }
   
   @Override
   public void setRegisterValue(byte value)
   {
      positionEcho = value;
   }

   @Override
   public int getRegisterIndex()
   {
      return index;
   }

}
