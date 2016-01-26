package us.ihmc.quadrupedRobotics.parameters;

public class DefaultQuadrupedVirtualModelBasedStepParameters implements QuadrupedVirtualModelBasedStepParameters
{
   private final double[] bodyOrientationProportionalGains = {5000, 5000, 2500};
   private final double[] bodyOrientationDerivativeGains = {750, 750, 500};
   private final double[] bodyOrientationIntegralGains = {0, 0, 0};
   private final double bodyOrientationMaxIntegralError = 0;

   private final double[] swingPositionProportionalGains = {0, 0, 0};
   private final double[] swingPositionDerivativeGains = {0, 0, 0};
   private final double[] swingPositionIntegralGains = {0, 0, 0};
   private final double swingPositionMaxIntegralError = 0;
   private final double swingPositionGravityFeedforwardForce = 0;

   private final double[] dcmProportionalGains = {2, 2, 0};
   private final double[] dcmIntegralGains = {0, 0, 0};
   private final double dcmMaxIntegralError = 0;

   private final double comHeightProportionalGain = 5000;
   private final double comHeightDerivativeGain = 750;
   private final double comHeightIntegralGain = 0;
   private final double comHeightMaxIntegralError = 0;
   private final double comHeightGravityFeedforwardConstant = 0.95;
   private final double comHeightNominal = 0.55;
   
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
   public double[] getSwingPositionProportionalGains()
   {
      return swingPositionProportionalGains;
   }

   @Override
   public double[] getSwingPositionDerivativeGains()
   {
      return swingPositionDerivativeGains;
   }

   @Override
   public double[] getSwingPositionIntegralGains()
   {
      return swingPositionIntegralGains;
   }

   @Override
   public double getSwingPositionMaxIntegralError()
   {
      return swingPositionMaxIntegralError;
   }

   @Override
   public double getSwingPositionGravityFeedforwardForce()
   {
      return swingPositionGravityFeedforwardForce;
   }

   @Override
   public double[] getDcmProportionalGains()
   {
      return dcmProportionalGains;
   }

   @Override
   public double[] getDcmIntegralGains()
   {
      return dcmIntegralGains;
   }

   @Override
   public double getDcmMaxIntegralError()
   {
      return dcmMaxIntegralError;
   }

   @Override
   public double getComHeightProportionalGain()
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
   public double getComHeightNominal()
   {
      return comHeightNominal;
   }
}