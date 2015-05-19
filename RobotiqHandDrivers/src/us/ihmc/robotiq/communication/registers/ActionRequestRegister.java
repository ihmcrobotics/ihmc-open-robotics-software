package us.ihmc.robotiq.communication.registers;

public class ActionRequestRegister extends RobotiqOutputRegister
{
   private rACT ract;
   private rMOD rmod;
   private rGTO rgto;
   private rATR ratr;

   private rACT ractInitial;
   private rMOD rmodInitial;
   private rGTO rgtoInitial;
   private rATR ratrInitial;
   
   public ActionRequestRegister(rACT ract, rMOD rmod, rGTO rgto, rATR ratr)
   {
      this.ract = ract;
      this.rmod = rmod;
      this.rgto = rgto;
      this.ratr = ratr;

      this.ractInitial = ract;
      this.rmodInitial = rmod;
      this.rgtoInitial = rgto;
      this.ratrInitial = ratr;
   }
   
   public rACT getRact()
   {
      return ract;
   }

   public void setRact(rACT ract)
   {
      this.ract = ract;
   }

   public rMOD getRmod()
   {
      return rmod;
   }

   public void setRmod(rMOD rmod)
   {
      this.rmod = rmod;
   }

   public rGTO getRgto()
   {
      return rgto;
   }

   public void setRgto(rGTO rgto)
   {
      this.rgto = rgto;
   }

   public rATR getRatr()
   {
      return ratr;
   }

   public void setRatr(rATR ratr)
   {
      this.ratr = ratr;
   }

   @Override
   public byte getRegisterValue()
   {
      byte ret = (byte)0x00;
      
      ret |= ratr.getValue() << 4;
      ret |= rgto.getValue() << 3;
      ret |= rmod.getValue() << 1;
      ret |= ract.getValue();
      
      return ret;
   }
   
   @Override
   public int getRegisterIndex()
   {
      return 0;
   }
   
   @Override
   public void resetRegister()
   {
      ract = ractInitial;
      rmod = rmodInitial;
      rgto = rgtoInitial;
      ratr = ratrInitial;
   }
   
   @Override
   public boolean equals(Object other)
   {
      boolean ret = super.equals(other);
      
      if(ret)
      {
         ret &= this.ract.equals(((ActionRequestRegister) other).getRact());
         ret &= this.rmod.equals(((ActionRequestRegister) other).getRmod());
         ret &= this.rgto.equals(((ActionRequestRegister) other).getRgto());
         ret &= this.ratr.equals(((ActionRequestRegister) other).getRatr());
      }
      
      return ret;
   }

   public enum rACT implements RobotiqRegisterComponent
   {
      DEACTIVATE_GRIPPER((byte)0x0), ACTIVATE_GRIPPER((byte)0x1);
      
      private byte value;
      
      private rACT(byte value)
      {
         this.value = value;
      }
      
      @Override
      public byte getValue()
      {
         return value;
      }
   }
   
   public enum rMOD implements RobotiqRegisterComponent
   {
      BASIC_MODE((byte)0x0), PINCH_MODE((byte)0x1), WIDE_MODE((byte)0x2), SCISSOR_MODE((byte)0x3);
      
      private byte value;
      
      private rMOD(byte value)
      {
         this.value = value;
      }

      @Override
      public byte getValue()
      {
         return value;
      }
   }
   
   public enum rGTO implements RobotiqRegisterComponent
   {
      STOP((byte)0x0), GO_TO((byte)0x1);
      
      private byte value;
      
      private rGTO(byte value)
      {
         this.value = value;
      }
      
      @Override
      public byte getValue()
      {
         return value;
      }
   }
   
   public enum rATR implements RobotiqRegisterComponent
   {
      NORMAL((byte)0x0), EMERGENCY_ATORELEASE((byte)0x1);
      
      private byte value;
      
      private rATR(byte value)
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
