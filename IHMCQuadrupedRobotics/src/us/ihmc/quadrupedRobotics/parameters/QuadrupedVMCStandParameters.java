package us.ihmc.quadrupedRobotics.parameters;

public interface QuadrupedVMCStandParameters
{
   public double[] getBodyOrientationProportionalGains();
   public double[] getBodyOrientationDerivativeGains();
   public double[] getBodyOrientationIntegralGains();
   public double getBodyOrientationMaxIntegralError();
   
   public double getIcpForwardProportionalGain();
   public double getIcpForwardDerivativeGain();
   public double getIcpForwardIntegralGain();
   public double getIcpForwardMaxIntegralError();
   
   public double getIcpLateralProportionalGain();
   public double getIcpLateralDerivativeGain();
   public double getIcpLateralIntegralGain();
   public double getIcpLateralMaxIntegralError();

   public double getComHeightPropotionalGain();
   public double getComHeightDerivativeGain();
   public double getComHeightIntegralGain();
   public double getComHeightMaxIntegralError();
   public double getComHeightGravityFeedforwardConstant();
   public double getComHeightNominal();
   
   public double getJointPositionLimitStiffness();
   public double getJointPositionLimitDamping();
   public double getCoefficientOfFriction();
}
