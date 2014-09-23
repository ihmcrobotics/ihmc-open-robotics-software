package us.ihmc.steppr.hardware.state;

public interface StepprJointState
{
   public double getQ();
   public double getQd();
   public double getTau();
 
   public void update();
}
