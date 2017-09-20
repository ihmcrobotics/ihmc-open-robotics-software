package us.ihmc.robotiq.communication.registers;

public abstract class RobotiqInputRegister extends RobotiqRegister
{
   public abstract void setRegisterValue(byte value);
}
