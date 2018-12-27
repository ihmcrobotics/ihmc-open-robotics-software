package us.ihmc.quadrupedPlanning.input;

import com.google.common.util.concurrent.AtomicDouble;
import controller_msgs.msg.dds.*;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.quadrupedBasics.QuadrupedSteppingRequestedEvent;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedOrientedStep;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.bodyPath.QuadrupedBodyPathMultiplexer;
import us.ihmc.quadrupedPlanning.footstepChooser.PlanarGroundPointFootSnapper;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapper;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitStepStream;
import us.ihmc.robotics.math.filters.RateLimitedYoFrameVector;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedStepTeleopManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoQuadrupedXGaitSettings xGaitSettings;
   private final YoDouble timestamp = new YoDouble("timestamp", registry);

   private final YoFrameVector3D desiredVelocity = new YoFrameVector3D("teleopDesiredVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble desiredVelocityRateLimit = new YoDouble("teleopDesiredVelocityRateLimit", registry);
   private final RateLimitedYoFrameVector limitedDesiredVelocity;

   private final YoDouble firstStepDelay = new YoDouble("firstStepDelay", registry);
   private final AtomicBoolean paused = new AtomicBoolean(false);
   private final AtomicDouble desiredBodyHeight = new AtomicDouble();

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();
   private final AtomicLong timestampNanos = new AtomicLong();

   private final QuadrupedReferenceFrames referenceFrames;

   private final QuadrupedXGaitStepStream stepStream;
   private final QuadrupedBodyPathMultiplexer bodyPathMultiplexer;
   private final PlanarGroundPointFootSnapper snapper;

   private QuadrupedTimedStepListMessage stepListMessage;
   private QuadrupedBodyOrientationMessage bodyOrientationMessage;
   private QuadrupedBodyHeightMessage bodyHeightMessage;

   public QuadrupedStepTeleopManager(QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, double initialBodyHeight,
                                     QuadrupedReferenceFrames referenceFrames, double updateDT, YoGraphicsListRegistry graphicsListRegistry,
                                     YoVariableRegistry parentRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, null, registry);

      firstStepDelay.set(0.5);
      this.bodyPathMultiplexer = new QuadrupedBodyPathMultiplexer(referenceFrames, timestamp, xGaitSettings, firstStepDelay,
                                                                  graphicsListRegistry, registry);
      this.stepStream = new QuadrupedXGaitStepStream(xGaitSettings, timestamp, bodyPathMultiplexer, firstStepDelay, registry);

      desiredVelocityRateLimit.set(10.0);
      limitedDesiredVelocity = new RateLimitedYoFrameVector("limitedTeleopDesiredVelocity", "", registry, desiredVelocityRateLimit, updateDT, desiredVelocity);

      desiredBodyHeight.set(initialBodyHeight);
      snapper = new PlanarGroundPointFootSnapper(referenceFrames);
      stepStream.setStepSnapper(snapper);

      parentRegistry.addChild(registry);
   }

   public void processBodyPathPlanMessage(QuadrupedBodyPathPlanMessage message)
   {
      bodyPathMultiplexer.setBodyPathPlanMessage(message);
   }

   public void processFootstepStatusMessage(QuadrupedFootstepStatusMessage message)
   {
      if (message.getFootstepStatus() == QuadrupedFootstepStatusMessage.FOOTSTEP_STATUS_STARTED)
      {
         RobotQuadrant quadrant = RobotQuadrant.fromByte((byte) message.getFootstepQuadrant());
         bodyPathMultiplexer.startedFootstep(quadrant, message);
      }
      else if (message.getFootstepStatus() == QuadrupedFootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED)
      {
         RobotQuadrant quadrant = RobotQuadrant.fromByte((byte) message.getFootstepQuadrant());
         bodyPathMultiplexer.completedFootstep(quadrant, message);
      }
   }

   public void processHighLevelStateChangeMessage(HighLevelStateChangeStatusMessage message)
   {
      controllerStateChangeMessage.set(message);
   }

   public void processSteppingStateChangeMessage(QuadrupedSteppingStateChangeMessage message)
   {
      steppingStateChangeMessage.set(message);
   }

   public void processGroundPlaneMessage(QuadrupedGroundPlaneMessage message)
   {
      snapper.submitGroundPlane(message);
   }

   public void processTimestamp(long timestampInNanos)
   {
      this.timestampNanos.set(timestampInNanos);
   }

   public void wakeUp()
   {
      stepStream.onEntry();

      populateStepMessage();
      populateBodyHeightMessage();
      populateBodyOrientationMessage();
   }

   public void update()
   {
      limitedDesiredVelocity.update();

      timestamp.set(Conversions.nanosecondsToSeconds(timestampNanos.get()));
      bodyPathMultiplexer.setPlanarVelocityForJoystickPath(limitedDesiredVelocity.getX(), limitedDesiredVelocity.getY(), limitedDesiredVelocity.getZ());
      referenceFrames.updateFrames();

      stepListMessage = null;
      bodyOrientationMessage = null;
      bodyHeightMessage = null;

      if (paused.get())
         return;

      stepStream.process();

      populateBodyHeightMessage();
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


   public QuadrupedTimedStepListMessage getStepListMessage()
   {
      return stepListMessage;
   }

   public QuadrupedBodyOrientationMessage getBodyOrientationMessage()
   {
      return bodyOrientationMessage;
   }

   public QuadrupedBodyHeightMessage getBodyHeightMessage()
   {
      return bodyHeightMessage;
   }

   private void populateStepMessage()
   {
      List<? extends QuadrupedTimedStep> steps = stepStream.getSteps();
      List<QuadrupedTimedStepMessage> stepMessages = new ArrayList<>();
      for (int i = 0; i < steps.size(); i++)
         stepMessages.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(steps.get(i)));

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

   public void setDesiredBodyHeight(double desiredBodyHeight)
   {
      this.desiredBodyHeight.set(desiredBodyHeight);
   }

   private final FramePoint3D tempPoint = new FramePoint3D();

   private void populateBodyHeightMessage()
   {
      double bodyHeight = desiredBodyHeight.getAndSet(Double.NaN);

      if (!Double.isNaN(bodyHeight))
      {
         tempPoint.setIncludingFrame(referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds(), 0.0, 0.0, bodyHeight);
         tempPoint.changeFrame(ReferenceFrame.getWorldFrame());

         bodyHeightMessage = QuadrupedMessageTools.createQuadrupedBodyHeightMessage(0.0, tempPoint.getZ());
         bodyHeightMessage.setControlBodyHeight(true);
         bodyHeightMessage.setIsExpressedInAbsoluteTime(false);
      }
   }

   public void setStepSnapper(PointFootSnapper stepSnapper)
   {
      stepStream.setStepSnapper(stepSnapper);
   }

   public YoQuadrupedXGaitSettings getXGaitSettings()
   {
      return xGaitSettings;
   }

   public void setShiftPlanBasedOnStepAdjustment(boolean shiftPlanBasedOnStepAdjustment)
   {
      bodyPathMultiplexer.setShiftPlanBasedOnStepAdjustment(shiftPlanBasedOnStepAdjustment);
   }

   public void handleBodyPathPlanMessage(QuadrupedBodyPathPlanMessage bodyPathPlanMessage)
   {
      bodyPathMultiplexer.initialize();
      bodyPathMultiplexer.handleBodyPathPlanMessage(bodyPathPlanMessage);
   }

   public void setPaused(boolean pause)
   {
      paused.set(pause);

      steppingStateChangeMessage.set(null);
   }

   public boolean isPaused()
   {
      return paused.get();
   }
}