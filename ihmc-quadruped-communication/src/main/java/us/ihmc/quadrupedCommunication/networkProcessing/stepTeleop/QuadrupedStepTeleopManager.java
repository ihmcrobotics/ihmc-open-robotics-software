package us.ihmc.quadrupedCommunication.networkProcessing.stepTeleop;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedOrientedStep;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsBasics;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.stepStream.bodyPath.QuadrupedBodyPathMultiplexer;
import us.ihmc.quadrupedPlanning.footstepChooser.PlanarGroundPointFootSnapper;
import us.ihmc.quadrupedPlanning.footstepChooser.PlanarRegionBasedPointFootSnapper;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitStepStream;
import us.ihmc.robotics.math.filters.RateLimitedYoFrameVector;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

public class QuadrupedStepTeleopManager
{
   private static final TrajectoryType trajectoryType = TrajectoryType.DEFAULT;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final QuadrupedXGaitSettingsBasics xGaitSettings;
   private final YoDouble timestamp = new YoDouble("timestamp", registry);

   private final YoFrameVector3D desiredVelocity = new YoFrameVector3D("teleopDesiredVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble desiredVelocityRateLimit = new YoDouble("teleopDesiredVelocityRateLimit", registry);
   private final RateLimitedYoFrameVector limitedDesiredVelocity;

   private final YoDouble firstStepDelay = new YoDouble("firstStepDelay", registry);
   private final AtomicBoolean paused = new AtomicBoolean(false);

   private final AtomicLong timestampNanos = new AtomicLong();

   private final QuadrupedXGaitStepStream stepStream;
   private final QuadrupedBodyPathMultiplexer bodyPathMultiplexer;
   private final PlanarGroundPointFootSnapper groundPlaneSnapper;
   private final PlanarRegionBasedPointFootSnapper planarRegionSnapper;

   private QuadrupedTimedStepListMessage stepListMessage;
   private QuadrupedBodyOrientationMessage bodyOrientationMessage;

   public QuadrupedStepTeleopManager(QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, PointFootSnapperParameters pointFootSnapperParameters, QuadrupedReferenceFrames referenceFrames, double updateDT,
                                     YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);

      firstStepDelay.set(0.5);
      this.bodyPathMultiplexer = new QuadrupedBodyPathMultiplexer(referenceFrames, timestamp, xGaitSettings, firstStepDelay, graphicsListRegistry, registry);
      this.stepStream = new QuadrupedXGaitStepStream(xGaitSettings, timestamp, bodyPathMultiplexer, firstStepDelay, registry);

      desiredVelocityRateLimit.set(1.0);
      limitedDesiredVelocity = new RateLimitedYoFrameVector("limitedTeleopDesiredVelocity", "", registry, desiredVelocityRateLimit, updateDT, desiredVelocity);

      groundPlaneSnapper = new PlanarGroundPointFootSnapper(referenceFrames);
      planarRegionSnapper = new PlanarRegionBasedPointFootSnapper(pointFootSnapperParameters);
      planarRegionSnapper.setFallbackSnapper(groundPlaneSnapper);
      stepStream.setStepSnapper(planarRegionSnapper);

      parentRegistry.addChild(registry);
   }

   public void processBodyPathPlanMessage(QuadrupedBodyPathPlanMessage message)
   {
      bodyPathMultiplexer.initialize();
      bodyPathMultiplexer.handleBodyPathPlanMessage(message);
   }

   public void processFootstepStatusMessage(QuadrupedFootstepStatusMessage message)
   {
      if (message.getFootstepStatus() == QuadrupedFootstepStatusMessage.FOOTSTEP_STATUS_STARTED)
      {
         RobotQuadrant quadrant = RobotQuadrant.fromByte(message.getRobotQuadrant());
         bodyPathMultiplexer.startedFootstep(quadrant, message);
      }
      else if (message.getFootstepStatus() == QuadrupedFootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED)
      {
         RobotQuadrant quadrant = RobotQuadrant.fromByte(message.getRobotQuadrant());
         bodyPathMultiplexer.completedFootstep(quadrant, message);
      }
   }

   public void processPlanarRegionsListMessage(PlanarRegionsListMessage message)
   {
      planarRegionSnapper.setPlanarRegionsList(PlanarRegionMessageConverter.convertToPlanarRegionsList(message));
   }

   public void processGroundPlaneMessage(QuadrupedGroundPlaneMessage message)
   {
      groundPlaneSnapper.submitGroundPlane(message);
   }

   public void processXGaitSettingsPacket(QuadrupedXGaitSettingsPacket packet)
   {
      xGaitSettings.set(packet);
   }

   public void processTimestamp(long timestampInNanos)
   {
      this.timestampNanos.set(timestampInNanos);
   }

   public void setShiftPlanBasedOnStepAdjustment(boolean shiftPlanBasedOnStepAdjustment)
   {
      bodyPathMultiplexer.setShiftPlanBasedOnStepAdjustment(shiftPlanBasedOnStepAdjustment);
   }


   public void setPaused(boolean pause)
   {
      paused.set(pause);
   }

   public void initialize()
   {
      timestamp.set(Conversions.nanosecondsToSeconds(timestampNanos.get()));
      paused.set(false);
      stepStream.onEntry();
   }

   public void update()
   {
      limitedDesiredVelocity.update();

      timestamp.set(Conversions.nanosecondsToSeconds(timestampNanos.get()));
      bodyPathMultiplexer.setPlanarVelocityForJoystickPath(limitedDesiredVelocity.getX(), limitedDesiredVelocity.getY(), limitedDesiredVelocity.getZ());

      stepListMessage = null;
      bodyOrientationMessage = null;

      if (paused.get())
         return;

      stepStream.process();

      populateStepMessage();
      populateBodyOrientationMessage();
   }

   public void sleep()
   {
      desiredVelocity.setToZero();
      limitedDesiredVelocity.setToZero();

      paused.set(true);
   }

   public void setDesiredVelocity(double desiredVelocityX, double desiredVelocityY, double desiredVelocityZ)
   {
      this.desiredVelocity.set(desiredVelocityX, desiredVelocityY, desiredVelocityZ);
   }

   public void setEndPhaseShift(double endPhaseShift)
   {
      xGaitSettings.setEndPhaseShift(endPhaseShift);
   }

   public QuadrupedTimedStepListMessage getStepListMessage()
   {
      return stepListMessage;
   }

   public QuadrupedBodyOrientationMessage getBodyOrientationMessage()
   {
      return bodyOrientationMessage;
   }

   private void populateStepMessage()
   {
      List<? extends QuadrupedTimedStep> steps = stepStream.getSteps();
      List<QuadrupedTimedStepMessage> stepMessages = new ArrayList<>();
      for (int i = 0; i < steps.size(); i++)
      {
         QuadrupedTimedStepMessage stepMessage = QuadrupedMessageTools.createQuadrupedTimedStepMessage(steps.get(i));

         if(trajectoryType != null)
            stepMessage.getQuadrupedStepMessage().setTrajectoryType(trajectoryType.toByte());

         stepMessages.add(stepMessage);
      }

      stepListMessage = QuadrupedMessageTools.createQuadrupedTimedStepListMessage(stepMessages, true);
   }

   private void populateBodyOrientationMessage()
   {
      bodyOrientationMessage = QuadrupedMessageTools.createQuadrupedWorldFrameYawMessage(getPlannedStepsSortedByEndTime(), limitedDesiredVelocity.getZ());
   }

   private final List<QuadrupedTimedOrientedStep> plannedStepsSortedByEndTime = new ArrayList<>();

   private List<QuadrupedTimedOrientedStep> getPlannedStepsSortedByEndTime()
   {
      plannedStepsSortedByEndTime.clear();
      PreallocatedList<QuadrupedTimedOrientedStep> plannedSteps = stepStream.getFootstepPlan().getPlannedSteps();
      plannedStepsSortedByEndTime.addAll(plannedSteps);
      TimeIntervalTools.sortByEndTime(plannedStepsSortedByEndTime);
      return plannedStepsSortedByEndTime;
   }
}