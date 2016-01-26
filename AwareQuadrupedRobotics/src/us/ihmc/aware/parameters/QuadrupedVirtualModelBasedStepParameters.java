package us.ihmc.quadrupedRobotics.parameters;

public interface QuadrupedVirtualModelBasedStepParameters
{
   public double[] getBodyOrientationProportionalGains();
   public double[] getBodyOrientationDerivativeGains();
   public double[] getBodyOrientationIntegralGains();
   public double getBodyOrientationMaxIntegralError();

   public double[] getSwingPositionProportionalGains();
   public double[] getSwingPositionDerivativeGains();
   public double[] getSwingPositionIntegralGains();
   public double getSwingPositionMaxIntegralError();
   public double getSwingPositionGravityFeedforwardForce();

   public double[] getDcmProportionalGains();
   public double[] getDcmIntegralGains();
   public double getDcmMaxIntegralError();

   public double getComHeightProportionalGain();
   public double getComHeightDerivativeGain();
   public double getComHeightIntegralGain();
   public double getComHeightMaxIntegralError();
   public double getComHeightGravityFeedforwardConstant();
   public double getComHeightNominal();
}
