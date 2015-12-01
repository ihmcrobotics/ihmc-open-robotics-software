package us.ihmc.quadrupedRobotics.parameters;

public class DefaultQuadrupedVMCStandParameters implements QuadrupedVMCStandParameters
{
   private final double[] bodyOrientationProportionalGains = {2000, 2000, 2000};
   private final double[] bodyOrientationDerivativeGains = {300, 300, 300};
   private final double[] bodyOrientationIntegralGains = {0, 0, 0};
   private final double bodyOrientationMaxIntegralError = 0;

   private final double icpForwardProportionalGain = 2.5;
   private final double icpForwardDerivativeGain = 0;
   private final double icpForwardIntegralGain = 0;
   private final double icpForwardMaxIntegralError = 0;

   private final double icpLateralProportionalGain = 2.5;
   private final double icpLateralDerivativeGain = 0;
   private final double icpLateralIntegralGain = 0;
   private final double icpLateralMaxIntegralError = 0;

   private final double comHeightProportionalGain = 5000;
   private final double comHeightDerivativeGain = 1000;
   private final double comHeightIntegralGain = 0;
   private final double comHeightMaxIntegralError = 0;
   private final double comHeightGravityFeedforwardConstant = 0.75;
   private final double comHeightSetpoint = 0.5;

   @Override
   public double[] getBodyOrientationProportionalGains()
   {
      return bodyOrientationProportionalGains;
   }

   @Override
   public double[] getBodyOrientationDerivativeGains()
   {
      return bodyOrientationDerivativeGains;
   }

   @Override
   public double[] getBodyOrientationIntegralGains()
   {
      return bodyOrientationIntegralGains;
   }

   @Override
   public double getBodyOrientationMaxIntegralError()
   {
      return bodyOrientationMaxIntegralError;
   }

   @Override
   public double getIcpForwardProportionalGain()
   {
      return icpForwardProportionalGain;
   }

   @Override
   public double getIcpForwardDerivativeGain()
   {
      return icpForwardDerivativeGain;
   }

   @Override
   public double getIcpForwardIntegralGain()
   {
      return icpForwardIntegralGain;
   }

   @Override
   public double getIcpForwardMaxIntegralError()
   {
      return icpForwardMaxIntegralError;
   }

   @Override
   public double getIcpLateralProportionalGain()
   {
      return icpLateralProportionalGain;
   }

   @Override
   public double getIcpLateralDerivativeGain()
   {
      return icpLateralDerivativeGain;
   }

   @Override
   public double getIcpLateralIntegralGain()
   {
      return icpLateralIntegralGain;
   }

   @Override
   public double getIcpLateralMaxIntegralError()
   {
      return icpLateralMaxIntegralError;
   }

   @Override
   public double getComHeightPropotionalGain()
   {
      return comHeightProportionalGain;
   }

   @Override
   public double getComHeightDerivativeGain()
   {
      return comHeightDerivativeGain;
   }

   @Override
   public double getComHeightIntegralGain()
   {
      return comHeightIntegralGain;
   }

   @Override
   public double getComHeightMaxIntegralError()
   {
      return comHeightMaxIntegralError;
   }

   @Override
   public double getComHeightGravityFeedforwardConstant()
   {
      return comHeightGravityFeedforwardConstant;
   }

   @Override
   public double getComHeightSetpoint()
   {
      return comHeightSetpoint;
   }
}