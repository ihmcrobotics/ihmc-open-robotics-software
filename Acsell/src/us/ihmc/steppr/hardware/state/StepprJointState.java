package us.ihmc.steppr.hardware.state;

public interface StepprJointState
{
   public double getQ();
   public double getQd();
   public double getTau();
   
   public int getNumberOfActuators();
   public double getMotorAngle(int actuator);
 
   public void update();
   
   public void updateOffsets();
}
