package us.ihmc.robotiq.communication.registers;

public class GripperOptionRegister implements RobotiqRegister
{
   private rICF ricf;
   private rICS rics;
   
   public GripperOptionRegister(rICF ricf, rICS rics)
   {
      this.ricf = ricf;
      this.rics = rics;
   }
   
   public rICF getRicf()
   {
      return ricf;
   }

   public void setRicf(rICF ricf)
   {
      this.ricf = ricf;
   }

   public rICS getRics()
   {
      return rics;
   }

   public void setRics(rICS rics)
   {
      this.rics = rics;
   }

   @Override
   public byte getRegisterValue()
   {
      byte ret = (byte)0x00;
      
      ret |= rics.getValue() << 3;
      ret |= ricf.getValue() << 2;
      
      return ret;
   }
   
   @Override
   public int getRegisterIndex()
   {
      return 1;
   }
   
   public enum rICF implements RobotiqRegisterComponent
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
   
   public enum rICS implements RobotiqRegisterComponent
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
