package us.ihmc.aware.parameters;

public interface QuadrupedVirtualModelBasedStandParameters
{
   public double[] getBodyOrientationProportionalGains();
   public double[] getBodyOrientationDerivativeGains();
   public double[] getBodyOrientationIntegralGains();
   public double getBodyOrientationMaxIntegralError();
   
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
