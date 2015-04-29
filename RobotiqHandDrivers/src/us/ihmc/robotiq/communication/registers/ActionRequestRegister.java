package us.ihmc.robotiq.communication.registers;

public class ActionRequestRegister implements RobotiqRegister
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
      return 0;
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
