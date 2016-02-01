package us.ihmc.robotiq.communication.registers;

import us.ihmc.robotiq.communication.Finger;
import us.ihmc.robotiq.communication.InvalidFingerException;

public class FingerForceRegister extends RobotiqOutputRegister
{
   private final byte MAX_FORCE = (byte) 0xFF;
   private final byte MIN_FORCE = (byte) 0x00;
   
   private final int index;
   private byte force;
   
   public FingerForceRegister(Finger finger)
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
      
      force = MAX_FORCE;
   }
   
   public void setForce(byte force)
   {
      this.force = force;
   }
   
   @Override
   public byte getRegisterValue()
   {
      return force;
   }
   
   @Override
   public int getRegisterIndex()
   {
      return index;
   }
   
   @Override
   public void resetRegister()
   {
      force = MAX_FORCE;
   }
}
