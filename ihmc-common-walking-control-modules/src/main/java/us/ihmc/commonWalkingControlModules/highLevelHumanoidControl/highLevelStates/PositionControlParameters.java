package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

public interface PositionControlParameters
{
   double getProportionalGain(int jointIndex);
   double getDerivativeGain(int jointIndex);
   double getIntegralGain(int jointIndex);

   double getProportionalGain(String jointName);
   double getDerivativeGain(String jointName);
   double getIntegralGain(String jointName);

   double getPositionControlMasterGain();
}
