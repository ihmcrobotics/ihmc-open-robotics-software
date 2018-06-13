package us.ihmc.quadrupedRobotics.planning.stepStream;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitPlanner;
import us.ihmc.quadrupedRobotics.planning.bodyPath.QuadrupedPlanarBodyPathProvider;
import us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser.PointFootSnapper;
import us.ihmc.quadrupedRobotics.providers.YoQuadrupedXGaitSettings;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class QuadrupedXGaitStepStream
{
   private static int NUMBER_OF_PREVIEW_STEPS = 16;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble minimumStepClearance = new YoDouble("minimumStepClearance", registry);
   private final YoDouble timestamp;

   private final YoQuadrupedXGaitSettings xGaitSettings;
   private final Vector3D desiredPlanarVelocity = new Vector3D();
   private final YoDouble timeShiftOnTransitionToWalk;

   private final QuadrupedXGaitPlanner xGaitStepPlanner;
   private final QuadrupedPlanarFootstepPlan footstepPlan;
   private final QuadrupedPlanarBodyPathProvider bodyPathProvider;

   public QuadrupedXGaitStepStream(YoQuadrupedXGaitSettings xGaitSettings, YoDouble timestamp,
                                   QuadrupedPlanarBodyPathProvider bodyPathProvider, YoVariableRegistry parentRegistry)
   {
      this.xGaitSettings = xGaitSettings;
      this.timestamp = timestamp;
      this.bodyPathProvider = bodyPathProvider;
      this.xGaitStepPlanner = new QuadrupedXGaitPlanner(bodyPathProvider, xGaitSettings);
      this.footstepPlan = new QuadrupedPlanarFootstepPlan(NUMBER_OF_PREVIEW_STEPS);
      minimumStepClearance.set(0.075);

      this.timeShiftOnTransitionToWalk = new YoDouble("timeShiftOnTransitionToWalk", registry);
      timeShiftOnTransitionToWalk.set(0.5);

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   private void updateXGaitSettings()
   {
      // increase stance dimensions as a function of velocity to prevent self collisions
      double strideRotation = desiredPlanarVelocity.getZ() * xGaitSettings.getStepDuration();
      double strideLength = Math.abs(2 * desiredPlanarVelocity.getX() * xGaitSettings.getStepDuration());
      double strideWidth = Math.abs(2 * desiredPlanarVelocity.getY() * xGaitSettings.getStepDuration());
      strideLength += Math.abs(xGaitSettings.getStanceWidth() / 2 * Math.sin(2 * strideRotation));
      strideWidth += Math.abs(xGaitSettings.getStanceLength() / 2 * Math.sin(2 * strideRotation));
      xGaitSettings.setStanceLength(Math.max(xGaitSettings.getStanceLength(), strideLength / 2 + minimumStepClearance.getValue()));
      xGaitSettings.setStanceWidth(Math.max(xGaitSettings.getStanceWidth(), strideWidth / 2 + minimumStepClearance.getValue()));
   }

   public void onEntry()
   {
      // initialize step queue
      updateXGaitSettings();
      double initialTime = timestamp.getDoubleValue() + timeShiftOnTransitionToWalk.getDoubleValue();
      RobotQuadrant initialQuadrant = (xGaitSettings.getEndPhaseShift() < 90) ? RobotQuadrant.HIND_LEFT : RobotQuadrant.FRONT_LEFT;
      bodyPathProvider.initialize();
      xGaitStepPlanner.computeInitialPlan(footstepPlan, initialQuadrant, initialTime);
      footstepPlan.initializeCurrentStepsFromPlannedSteps();
      this.process();
   }

   public void process()
   {
      double currentTime = timestamp.getDoubleValue();

      // update xgait current steps
      footstepPlan.updateCurrentSteps(timestamp.getDoubleValue());

      updateXGaitSettings();
      xGaitStepPlanner.computeOnlinePlan(footstepPlan, currentTime);
   }

   public List<? extends QuadrupedTimedStep> getSteps()
   {
      return footstepPlan.getCompleteStepSequence(timestamp.getDoubleValue());
   }

   public QuadrupedPlanarFootstepPlan getFootstepPlan()
   {
      return footstepPlan;
   }

   public void setStepSnapper(PointFootSnapper snapper)
   {
      xGaitStepPlanner.setStepSnapper(snapper);
   }
}
