package us.ihmc.quadrupedRobotics.parameters;

public interface QuadrupedControllerParameters extends SwingTargetGeneratorParameters
{

   public double getInitalCoMHeight();
   
   public double getDefaultSwingHeight();

   public double getDefaultSwingDuration();

   public double getDefaultSubCircleRadius();

   public double getInitialDesiredFootCorrectionBreakFrequency();

   public double getDefaultDesiredFootCorrectionBreakFrequency();

}