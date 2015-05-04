package us.ihmc.robotiq.communication.registers;

public class GripperOptionRegister extends RobotiqOutputRegister
{
   private rICF ricf;
   private rICS rics;
   
   private final rICF ricfInitial;
   private final rICS ricsInitial;
   
   public GripperOptionRegister(rICF ricf, rICS rics)
   {
      this.ricf = ricf;
      this.rics = rics;
      
      this.ricfInitial = ricf;
      this.ricsInitial = rics;
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
   
   @Override
   public void resetRegister()
   {
      ricf = ricfInitial;
      rics = ricsInitial;
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
