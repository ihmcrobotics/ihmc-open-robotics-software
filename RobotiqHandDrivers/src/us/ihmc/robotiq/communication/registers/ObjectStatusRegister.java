package us.ihmc.robotiq.communication.registers;

public class ObjectStatusRegister implements RobotiqRegister
{
   private gDTA gdta;
   private gDTB gdtb;
   private gDTC gdtc;
   private gDTS gdts;
   
   public ObjectStatusRegister(gDTA gdta, gDTB gdtb, gDTC gdtc, gDTS gdts)
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
      
      ret |= gdts.getValue() << 7;
      ret |= gdtc.getValue() << 5;
      ret |= gdtb.getValue() << 3;
      ret |= gdta.getValue() << 1;
      
      return ret;
   }

   @Override
   public int getRegisterIndex()
   {
      return 1;
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
