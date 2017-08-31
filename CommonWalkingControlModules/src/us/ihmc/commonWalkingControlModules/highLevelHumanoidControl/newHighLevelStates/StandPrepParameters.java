package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

public interface StandPrepParameters
{
   double getSetpoint(int jointIndex);

   double getSetpoint(String jointName);

   double getProportionalGain(int jointIndex);
   double getDerivativeGain(int jointIndex);
   double getIntegralGain(int jointIndex);

   double getProportionalGain(String jointName);
   double getDerivativeGain(String jointName);
   double getIntegralGain(String jointName);
}
