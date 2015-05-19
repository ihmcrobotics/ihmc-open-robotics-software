package us.ihmc.robotiq.communication.registers;

public class FaultStatusRegister extends RobotiqInputRegister
{
   private gFLT gflt;
   
   public FaultStatusRegister(gFLT gflt)
   {
      this.gflt = gflt;
   }

   public gFLT getGflt()
   {
      return gflt;
   }

   public void setGflt(gFLT gflt)
   {
      this.gflt = gflt;
   }

   @Override
   public byte getRegisterValue()
   {
      return gflt.getValue();
   }
   
   @Override
   public void setRegisterValue(byte value)
   {
      for(gFLT g : gFLT.values())
      {
         if(g.getValue() == value)
            this.gflt = g;
      }
   }

   @Override
   public int getRegisterIndex()
   {
      return 2;
   }
   
   @Override
   public boolean equals(Object other)
   {
      boolean ret = super.equals(other);
      
      if(ret)
         ret &= this.gflt.equals(((FaultStatusRegister) other).getGflt());
      
      return ret;
   }

   public enum gFLT implements RobotiqRegisterComponent
   {
      NO_FAULT((byte)0x0), ACTION_DELAYED_BY_ACTIVATION((byte)0x5), ACTION_DELAYED_BY_MODE_CHANGE((byte)0x6),
      ACTIVATION_BIT_NOT_SET((byte)0x7), COMM_CHIP_NOT_READY((byte)0x9), MINOR_SCISSOR_INTERFERENCE((byte)0xA),
      AUTO_RELEASE_IN_PROGRESS((byte)0xB), ACTIVATION_FAULT((byte)0xD), MAJOR_SCISSOR_INTERFERENCE((byte)0xE),
      AUTO_RELEASE_COMPLETED((byte)0xF);
      
      private byte value;
      
      private gFLT(byte value)
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
