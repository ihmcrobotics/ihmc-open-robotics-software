package us.ihmc.robotiq.communication.registers;

public class GipperOptionRegister implements RobotiqRegister
{
   @Override
   public byte getRegisterValue()
   {
      // TODO Auto-generated method stub
      return 0;
   }
   
   @Override
   public int getRegisterIndex()
   {
      return 1;
   }
   
   enum rICF implements RobotiqRegisterComponent
   {
      NORMAL((byte)0x0), INDIVIDUAL_FINGER_CONTROL((byte)0x1);
      
      private byte value;
      
      private rICF(byte value)
      {
         this.value = value;
      }
      
      @Override
      public byte getValue()
      {
         return value;
      }
   }
   
   enum rICS implements RobotiqRegisterComponent
   {
      NORMAL((byte)0x0), INDIVIDUAL_SCISSOR_CONTROL((byte)0x1);
      
      private byte value;
      
      private rICS(byte value)
      {
         this.value = value;
      }

      @Override
      public byte getValue()
      {
         return value;
      }
      
   }
}
