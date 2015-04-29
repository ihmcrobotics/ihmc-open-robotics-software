package us.ihmc.robotiq.communication.registers;

import us.ihmc.robotiq.communication.Finger;

public class FingerForceRegister implements RobotiqRegister
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
            throw new RuntimeException(getClass().getSimpleName() + ": " + finger.name() + " is not recognized as a Robotiq finger");
      }
      
      force = (byte)0x00;
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
}
