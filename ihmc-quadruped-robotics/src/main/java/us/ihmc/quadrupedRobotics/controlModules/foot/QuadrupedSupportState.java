package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commons.MathTools;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedSupportState extends QuadrupedFootState
{
   private final RobotQuadrant robotQuadrant;
   private final YoBoolean stepCommandIsValid;
   private final YoDouble timestamp;
   private final YoQuadrupedTimedStep currentStepCommand;

   public QuadrupedSupportState(RobotQuadrant robotQuadrant, YoBoolean stepCommandIsValid, YoDouble timestamp, YoQuadrupedTimedStep stepCommand)
   {
      this.robotQuadrant = robotQuadrant;
      this.stepCommandIsValid = stepCommandIsValid;
      this.timestamp = timestamp;
      this.currentStepCommand = stepCommand;
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public QuadrupedFootControlModule.FootEvent process()
   {
      // trigger swing phase
      if (stepCommandIsValid.getBooleanValue() && currentStepCommand.getTimeInterval().intervalContains(timestamp.getDoubleValue()))
      {
         if (stepTransitionCallback != null)
         {
            stepTransitionCallback.onLiftOff(robotQuadrant);
         }
         return QuadrupedFootControlModule.FootEvent.TIMEOUT;
      }

      return null;
   }

   @Override
   public void onExit()
   {
   }
}
