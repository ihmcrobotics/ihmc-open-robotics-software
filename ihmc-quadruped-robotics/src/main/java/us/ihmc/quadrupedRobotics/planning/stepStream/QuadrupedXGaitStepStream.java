package us.ihmc.quadrupedRobotics.planning.stepStream;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitPlanner;
import us.ihmc.quadrupedRobotics.planning.bodyPath.QuadrupedBodyPathMultiplexer;
import us.ihmc.quadrupedRobotics.planning.bodyPath.QuadrupedConstantVelocityBodyPathProvider;
import us.ihmc.quadrupedRobotics.planning.bodyPath.QuadrupedPlanarBodyPathProvider;
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
   private final DoubleParameter initialStepDelayParameter = new DoubleParameter("initialStepDelay", registry, 0.5);
   private final DoubleParameter minimumStepClearanceParameter = new DoubleParameter("minimumStepClearance", registry, 0.075);
   private final YoDouble timestamp;

   private final YoQuadrupedXGaitSettings xGaitSettings;
   private final Vector3D desiredPlanarVelocity = new Vector3D();

   private final QuadrupedXGaitPlanner xGaitStepPlanner;
   private final QuadrupedPlanarFootstepPlan footstepPlan;
   private final QuadrupedPlanarBodyPathProvider bodyPathProvider;
   private final ReferenceFrame centroidFrame;

   public QuadrupedXGaitStepStream(YoQuadrupedXGaitSettings xGaitSettings, QuadrupedReferenceFrames referenceFrames, YoDouble timestamp,
                                   QuadrupedPlanarBodyPathProvider bodyPathProvider, YoVariableRegistry parentRegistry)
   {
      this(xGaitSettings, referenceFrames, Double.NaN, timestamp, bodyPathProvider, parentRegistry);
   }

   public QuadrupedXGaitStepStream(YoQuadrupedXGaitSettings xGaitSettings, QuadrupedReferenceFrames referenceFrames, double controlDT, YoDouble timestamp,
                                   QuadrupedPlanarBodyPathProvider bodyPathProvider, YoVariableRegistry parentRegistry)
   {
      this.xGaitSettings = xGaitSettings;
      this.timestamp = timestamp;
      this.bodyPathProvider = bodyPathProvider;
      this.xGaitStepPlanner = new QuadrupedXGaitPlanner(bodyPathProvider);
      this.footstepPlan = new QuadrupedPlanarFootstepPlan(NUMBER_OF_PREVIEW_STEPS);
      this.centroidFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();

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
      xGaitSettings.setStanceLength(Math.max(xGaitSettings.getStanceLength(), strideLength / 2 + minimumStepClearanceParameter.getValue()));
      xGaitSettings.setStanceWidth(Math.max(xGaitSettings.getStanceWidth(), strideWidth / 2 + minimumStepClearanceParameter.getValue()));
   }

   public void onEntry()
   {
      // initialize step queue
      updateXGaitSettings();
      double initialTime = timestamp.getDoubleValue() + initialStepDelayParameter.getValue();
      RobotQuadrant initialQuadrant = (xGaitSettings.getEndPhaseShift() < 90) ? RobotQuadrant.HIND_LEFT : RobotQuadrant.FRONT_LEFT;
      bodyPathProvider.initialize();
      xGaitStepPlanner.computeInitialPlan(footstepPlan, initialQuadrant, initialTime, xGaitSettings);
      footstepPlan.initializeCurrentStepsFromPlannedSteps();
      this.process();
   }

   public void process()
   {
      double currentTime = timestamp.getDoubleValue();

      // update xgait current steps
      footstepPlan.updateCurrentSteps(timestamp.getDoubleValue());

      updateXGaitSettings();
      xGaitStepPlanner.computeOnlinePlan(footstepPlan, currentTime, getCurrentHeight(), xGaitSettings);
   }

   private double getCurrentHeight()
   {
      return centroidFrame.getTransformToWorldFrame().getTranslationZ();
   }

   public List<? extends QuadrupedTimedStep> getSteps()
   {
      return footstepPlan.getCompleteStepSequence(timestamp.getDoubleValue());
   }

   public QuadrupedPlanarFootstepPlan getFootstepPlan()
   {
      return footstepPlan;
   }
}
