package us.ihmc.robotiq.communication.registers;

import net.wimpi.modbus.procimg.SimpleRegister;

public class ByteSettableSimpleRegister extends SimpleRegister
{
   public ByteSettableSimpleRegister()
   {
      super();
   }
   
   public ByteSettableSimpleRegister(int value)
   {
      super(value);
   }
   
   public ByteSettableSimpleRegister(byte b1, byte b2)
   {
      super(b1, b2);
   }
   
   public void setValue(byte b1, byte b2)
   {
      m_Register[0] = b1;
      m_Register[1] = b2;
   }
}
