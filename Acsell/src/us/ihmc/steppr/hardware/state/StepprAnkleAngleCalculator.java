package us.ihmc.steppr.hardware.state;

public interface StepprAnkleAngleCalculator
{
   public void updateAnkleState(double motorAngleRight, double motorAngleLeft, double motorVelocityRight, double motorVelocityLeft);

   public double getQAnkleX();

   public double getQAnkleY();

   public double getQdAnkleX();

   public double getQdAnkleY();

}
