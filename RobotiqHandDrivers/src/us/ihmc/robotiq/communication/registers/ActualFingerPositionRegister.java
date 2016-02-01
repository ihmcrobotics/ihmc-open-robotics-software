package us.ihmc.robotiq.communication.registers;

import us.ihmc.robotiq.communication.Finger;
import us.ihmc.robotiq.communication.InvalidFingerException;

public class ActualFingerPositionRegister extends RobotiqInputRegister
{
   private final int index;
   private byte actualPosition;
   
   public ActualFingerPositionRegister(Finger finger)
   {
      switch(finger)
      {
         case FINGER_A:
            index = 4;
            break;
         case FINGER_B:
            index = 7;
            break;
         case FINGER_C:
            index = 10;
            break;
         case SCISSOR:
            index = 13;
            break;
         default:
            throw new InvalidFingerException(finger);
      
      }
   }

   @Override
   public byte getRegisterValue()
   {
      return actualPosition;
   }
   
   @Override
   public void setRegisterValue(byte value)
   {
      this.actualPosition = value;
   }

   @Override
   public int getRegisterIndex()
   {
      return index;
   }
}
