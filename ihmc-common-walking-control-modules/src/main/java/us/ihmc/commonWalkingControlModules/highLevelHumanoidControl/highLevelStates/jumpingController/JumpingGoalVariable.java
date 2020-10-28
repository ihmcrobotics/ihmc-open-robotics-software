package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.tools.saveableModule.YoSaveableModuleState;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class JumpingGoalVariable extends YoSaveableModuleState
{
   private final YoDouble goalLength;
   private final YoDouble goalFootWidth;
   private final YoDouble goalRotation;
   private final YoDouble goalHeight;

   private final YoDouble supportDuration;
   private final YoDouble flightDuration;

   public JumpingGoalVariable(String suffix, YoRegistry registry)
   {
      goalLength = new YoDouble("goalLength" + suffix, registry);
      goalFootWidth = new YoDouble("goalFootWidth" + suffix, registry);
      goalRotation = new YoDouble("goalRotation" + suffix, registry);
      goalHeight = new YoDouble("goalHeight" + suffix, registry);

      supportDuration = new YoDouble("supportDuration" + suffix, registry);
      flightDuration = new YoDouble("flightDuration" + suffix, registry);

      reset();
   }

   public void reset()
   {
      goalLength.setToNaN();
      goalFootWidth.setToNaN();
      goalRotation.setToNaN();
      goalHeight.setToNaN();

      supportDuration.setToNaN();
      flightDuration.setToNaN();
   }

   public void set(JumpingGoal jumpingGoal)
   {
      goalLength.set(jumpingGoal.getGoalLength());
      goalFootWidth.set(jumpingGoal.getGoalFootWidth());
      goalRotation.set(jumpingGoal.getGoalRotation());
      goalHeight.set(jumpingGoal.getGoalHeight());

      supportDuration.set(jumpingGoal.getSupportDuration());
      flightDuration.set(jumpingGoal.getFlightDuration());
   }

   public double getGoalLength()
   {
      return goalLength.getDoubleValue();
   }

   public double getSupportDuration()
   {
      return supportDuration.getDoubleValue();
   }

   public double getFlightDuration()
   {
      return flightDuration.getDoubleValue();
   }
}
