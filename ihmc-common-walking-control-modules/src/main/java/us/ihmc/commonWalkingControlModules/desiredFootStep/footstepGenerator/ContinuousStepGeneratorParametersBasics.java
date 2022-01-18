package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;

public interface ContinuousStepGeneratorParametersBasics
{
   default void set(ContinuousStepGeneratorParametersBasics other)
   {
      setNumberOfFixedFootsteps(other.getNumberOfFixedFootsteps());
      setSwingHeight(other.getSwingHeight());
      setSwingDuration(other.getSwingDuration());
      setTransferDuration(other.getTransferDuration());
      setMaxStepLength(other.getMaxStepLength());
      setDefaultStepWidth(other.getDefaultStepWidth());
      setMinStepWidth(other.getMinStepWidth());
      setMaxStepWidth(other.getMaxStepWidth());
      setTurnMaxAngleInward(other.getTurnMaxAngleInward());
      setTurnMaxAngleOutward(other.getTurnMaxAngleOutward());
   }

   default void set(WalkingControllerParameters walkingControllerParameters)
   {
      setNumberOfFixedFootsteps(0);
      setSwingDuration(walkingControllerParameters.getDefaultSwingTime());
      setTransferDuration(walkingControllerParameters.getDefaultTransferTime());

      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();
      setSwingHeight(steppingParameters.getMinSwingHeightFromStanceFoot());
      setMaxStepLength(steppingParameters.getMaxStepLength());
      setDefaultStepWidth(steppingParameters.getInPlaceWidth());
      setMinStepWidth(steppingParameters.getMinStepWidth());
      setMaxStepWidth(steppingParameters.getMaxStepWidth());
      setTurnMaxAngleInward(steppingParameters.getMaxAngleTurnInwards());
      setTurnMaxAngleOutward(steppingParameters.getMaxAngleTurnOutwards());
   }

   void setNumberOfFixedFootsteps(int numberOfFixedFootsteps);

   void setSwingHeight(double swingHeight);

   void setSwingDuration(double swingDuration);

   void setTransferDuration(double transferDuration);

   void setMaxStepLength(double maxStepLength);

   void setDefaultStepWidth(double defaultStepWidth);

   void setMinStepWidth(double minStepWidth);

   void setMaxStepWidth(double maxStepWidth);

   void setTurnMaxAngleInward(double turnMaxAngleInward);

   void setTurnMaxAngleOutward(double turnMaxAngleOutward);

   int getNumberOfFixedFootsteps();

   double getSwingHeight();

   double getSwingDuration();

   double getTransferDuration();

   double getMaxStepLength();

   double getDefaultStepWidth();

   double getMinStepWidth();

   double getMaxStepWidth();

   double getTurnMaxAngleInward();

   double getTurnMaxAngleOutward();
}
