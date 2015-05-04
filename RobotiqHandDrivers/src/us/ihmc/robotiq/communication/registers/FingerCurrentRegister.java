package us.ihmc.robotiq.communication.registers;

import us.ihmc.robotiq.communication.Finger;
import us.ihmc.robotiq.communication.InvalidFingerException;

public class FingerCurrentRegister extends RobotiqInputRegister
{
   private final int index;
   private byte current;
   
   public FingerCurrentRegister(Finger finger)
   {
      switch(finger)
      {
         case FINGER_A:
            index = 5;
            break;
         case FINGER_B:
            index = 8;
            break;
         case FINGER_C:
            index = 11;
            break;
         case SCISSOR:
            index = 14;
            break;
         default:
            throw new InvalidFingerException(finger);
      }
   }

   @Override
   public byte getRegisterValue()
   {
      return current;
   }
   
   @Override
   public void setRegisterValue(byte value)
   {
      current = value;
   }

   @Override
   public int getRegisterIndex()
   {
      return index;
   }

}
