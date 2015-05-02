package us.ihmc.robotiq.communication.registers;

import us.ihmc.robotiq.communication.Finger;
import us.ihmc.robotiq.communication.InvalidFingerException;

public class ActualFingerPositionRegister implements RobotiqRegister
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

   public void setActualPosition(byte actualPosition)
   {
      this.actualPosition = actualPosition;
   }
   
   @Override
   public byte getRegisterValue()
   {
      return actualPosition;
   }

   @Override
   public int getRegisterIndex()
   {
      return index;
   }
}
