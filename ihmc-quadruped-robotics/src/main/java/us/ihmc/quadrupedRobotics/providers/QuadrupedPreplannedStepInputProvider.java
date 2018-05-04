package us.ihmc.quadrupedRobotics.providers;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedPreplannedStepInputProvider
{
   private final AtomicReference<List<QuadrupedTimedStep>> inputTimedSteps = new AtomicReference<>(new ArrayList<QuadrupedTimedStep>());
   private final List<QuadrupedTimedStep> emptyTimedSteps = new ArrayList<>();
   private final RecyclingArrayList<QuadrupedTimedStep> debugTimedSteps = new RecyclingArrayList<>(30, QuadrupedTimedStep.class);

   private final YoEnum<RobotQuadrant> yoTimedStepQuadrant;
   private final YoDouble yoTimedStepDuration;
   private final YoDouble yoTimedStepGroundClearance;
   private final YoFramePoint3D yoTimedStepGoalPosition;

   public QuadrupedPreplannedStepInputProvider(YoVariableRegistry registry)
   {

      yoTimedStepQuadrant = new YoEnum<>("timedStepQuadrant", registry, RobotQuadrant.class);
      yoTimedStepDuration = new YoDouble("timedStepDuration", registry);
      yoTimedStepGroundClearance = new YoDouble("timedStepGroundClearance", registry);
      yoTimedStepGoalPosition = new YoFramePoint3D("timedStepGoalPosition", ReferenceFrame.getWorldFrame(), registry);
      
      setupYoVariableChangedListener(yoTimedStepQuadrant);
      setupYoVariableChangedListener(yoTimedStepDuration);
      setupYoVariableChangedListener(yoTimedStepGroundClearance);
      setupYoVariableChangedListener(yoTimedStepGoalPosition.getYoX());
      setupYoVariableChangedListener(yoTimedStepGoalPosition.getYoY());
      setupYoVariableChangedListener(yoTimedStepGoalPosition.getYoZ());
   }
   
   private void setupYoVariableChangedListener(YoVariable<?> yoVariable)
   {
      yoVariable.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            debugTimedSteps.getAndGrowIfNeeded(0).setRobotQuadrant(yoTimedStepQuadrant.getEnumValue());
            debugTimedSteps.get(0).setGroundClearance(yoTimedStepGroundClearance.getDoubleValue());
            debugTimedSteps.get(0).setGoalPosition(yoTimedStepGoalPosition);
            debugTimedSteps.get(0).getTimeInterval().setInterval(0.5, 0.5 + yoTimedStepDuration.getDoubleValue());
            inputTimedSteps.set(debugTimedSteps);

         }
      });
   }

   public boolean isStepPlanAvailable()
   {
      List<QuadrupedTimedStep> steps = inputTimedSteps.get();
      return (steps.size() > 0);
   }

   public boolean isStepPlanExpressedInAbsoluteTime()
   {
      return false;
   }

   public List<QuadrupedTimedStep> getAndClearSteps()
   {
      List<QuadrupedTimedStep> steps = inputTimedSteps.get();
      inputTimedSteps.set(emptyTimedSteps);

      return steps;
   }
}
