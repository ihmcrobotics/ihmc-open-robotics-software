package us.ihmc.robotiq.communication.registers;

public abstract class RobotiqRegister
{
   public abstract byte getRegisterValue();
   public abstract int getRegisterIndex();
   
   @Override
   public boolean equals(Object other)
   {
      if(!this.getClass().equals(other.getClass()))
         return false;
      
      return this.getRegisterValue() == ((RobotiqRegister)other).getRegisterValue() && this.getRegisterIndex() == ((RobotiqRegister)other).getRegisterIndex();
   }
}
