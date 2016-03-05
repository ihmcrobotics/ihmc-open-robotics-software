package us.ihmc.robotiq.communication.registers;

public class ObjectDetectionRegister extends RobotiqInputRegister
{
   private gDTA gdta;
   private gDTB gdtb;
   private gDTC gdtc;
   private gDTS gdts;
   
   public ObjectDetectionRegister(gDTA gdta, gDTB gdtb, gDTC gdtc, gDTS gdts)
   {
      this.gdta = gdta;
      this.gdtb = gdtb;
      this.gdtc = gdtc;
      this.gdts = gdts;
   }

   public gDTA getGdta()
   {
      return gdta;
   }

   public void setGdta(gDTA gdta)
   {
      this.gdta = gdta;
   }

   public gDTB getGdtb()
   {
      return gdtb;
   }

   public void setGdtb(gDTB gdtb)
   {
      this.gdtb = gdtb;
   }

   public gDTC getGdtc()
   {
      return gdtc;
   }

   public void setGdtc(gDTC gdtc)
   {
      this.gdtc = gdtc;
   }

   public gDTS getGdts()
   {
      return gdts;
   }

   public void setGdts(gDTS gdts)
   {
      this.gdts = gdts;
   }

   @Override
   public byte getRegisterValue()
   {
      byte ret = (byte)0x0;
      
      ret |= gdts.getValue() << 6;
      ret |= gdtc.getValue() << 4;
      ret |= gdtb.getValue() << 2;
      ret |= gdta.getValue() << 0;
      
      return ret;
   }
   
   @Override
   public void setRegisterValue(byte value)
   {
      byte gdts = (byte) ((value >>> 6) & 0x3);
      byte gdtc = (byte) ((value >>> 4) & 0x3);
      byte gdtb = (byte) ((value >>> 2) & 0x3);
      byte gdta = (byte) (value & 0x3);
      
      for(gDTA g : gDTA.values())
      {
         if(g.getValue() == gdta)
            this.gdta = g;
      }
      
      for(gDTB g : gDTB.values())
      {
         if(g.getValue() == gdtb)
            this.gdtb = g;
      }
      
      for(gDTC g : gDTC.values())
      {
         if(g.getValue() == gdtc)
            this.gdtc = g;
      }
      
      for(gDTS g : gDTS.values())
      {
         if(g.getValue() == gdts)
            this.gdts = g;
      }
   }

   @Override
   public int getRegisterIndex()
   {
      return 1;
   }
   
   @Override
   public boolean equals(Object other)
   {
      boolean ret = super.equals(other);
      
      if(ret)
      {
         ret &= this.gdta.equals(((ObjectDetectionRegister) other).getGdta());
         ret &= this.gdtb.equals(((ObjectDetectionRegister) other).getGdtb());
         ret &= this.gdtc.equals(((ObjectDetectionRegister) other).getGdtc());
         ret &= this.gdts.equals(((ObjectDetectionRegister) other).getGdts());
      }
      
      return ret;
   }

   public enum gDTA implements RobotiqRegisterComponent
   {
      FINGER_A_IN_MOTION((byte)0x0), FINGER_A_CONTACT_WHILE_OPENING((byte)0x1),
      FINGER_A_CONTACT_WHILE_CLOSING((byte)0x2), FINGER_A_AT_REQUESTED_POSITION((byte)0x3);
      
      private byte value;
      
      private gDTA(byte value)
      {
         this.value = value;
      }

      @Override
      public byte getValue()
      {
         return value;
      }
   }
   
   public enum gDTB implements RobotiqRegisterComponent
   {
      FINGER_B_IN_MOTION((byte)0x0), FINGER_B_CONTACT_WHILE_OPENING((byte)0x1),
      FINGER_B_CONTACT_WHILE_CLOSING((byte)0x2), FINGER_B_AT_REQUESTED_POSITION((byte)0x3);
      
      private byte value;
      
      private gDTB(byte value)
      {
         this.value = value;
      }

      @Override
      public byte getValue()
      {
         return value;
      }
   }
   
   public enum gDTC implements RobotiqRegisterComponent
   {
      FINGER_C_IN_MOTION((byte)0x0), FINGER_C_CONTACT_WHILE_OPENING((byte)0x1),
      FINGER_C_CONTACT_WHILE_CLOSING((byte)0x2), FINGER_C_AT_REQUESTED_POSITION((byte)0x3);
      
      private byte value;
      
      private gDTC(byte value)
      {
         this.value = value;
      }

      @Override
      public byte getValue()
      {
         return value;
      }
   }
   
   public enum gDTS implements RobotiqRegisterComponent
   {
      SCISSOR_IN_MOTION((byte)0x0), SCISSOR_CONTACT_WHILE_OPENING((byte)0x1),
      SCISSOR_CONTACT_WHILE_CLOSING((byte)0x2), SCISSOR_AT_REQUESTED_POSITION((byte)0x3);
      
      private byte value;
      
      private gDTS(byte value)
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
