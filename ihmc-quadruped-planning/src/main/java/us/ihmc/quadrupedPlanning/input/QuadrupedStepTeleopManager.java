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
   private final QuadrupedXGaitStepStream stepStream;
   private final YoQuadrupedXGaitSettings xGaitSettings;
   private final YoDouble timestamp = new YoDouble("timestamp", registry);
   private final YoBoolean walking = new YoBoolean("walking", registry);
   private final Ros2Node ros2Node;

   private final YoBoolean xGaitRequested = new YoBoolean("xGaitRequested", registry);
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
   private final QuadrupedBodyPathMultiplexer bodyPathMultiplexer;
   private IHMCROS2Publisher<QuadrupedTimedStepListMessage> timedStepListPublisher;
   private IHMCROS2Publisher<QuadrupedBodyOrientationMessage> bodyOrientationPublisher;
   private IHMCROS2Publisher<QuadrupedBodyHeightMessage> bodyHeightPublisher;

   public QuadrupedStepTeleopManager(String robotName, Ros2Node ros2Node, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, double initialBodyHeight,
                                     QuadrupedReferenceFrames referenceFrames, double updateDT, YoGraphicsListRegistry graphicsListRegistry,
                                     YoVariableRegistry parentRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.ros2Node = ros2Node;
      this.xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, null, registry);

      firstStepDelay.set(0.5);
      this.bodyPathMultiplexer = new QuadrupedBodyPathMultiplexer(referenceFrames, timestamp, xGaitSettings, firstStepDelay,
                                                                  graphicsListRegistry, registry);
      this.stepStream = new QuadrupedXGaitStepStream(xGaitSettings, timestamp, bodyPathMultiplexer, firstStepDelay, registry);

      desiredVelocityRateLimit.set(10.0);
      limitedDesiredVelocity = new RateLimitedYoFrameVector("limitedTeleopDesiredVelocity", "", registry, desiredVelocityRateLimit, updateDT, desiredVelocity);

      desiredBodyHeight.set(initialBodyHeight);
      stepStream.setStepSnapper(new PlanarGroundPointFootSnapper(robotName, referenceFrames, ros2Node));

      MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(ros2Node, HighLevelStateChangeStatusMessage.class, controllerPubGenerator,
                                           s -> controllerStateChangeMessage.set(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedSteppingStateChangeMessage.class, controllerPubGenerator,
                                           s -> steppingStateChangeMessage.set(s.takeNextData()));
      ROS2Tools
            .createCallbackSubscription(ros2Node, RobotConfigurationData.class, controllerPubGenerator, s -> timestampNanos.set(s.takeNextData().timestamp_));
      ROS2Tools.createCallbackSubscription(ros2Node, HighLevelStateMessage.class, controllerPubGenerator, s -> setPaused(true));

      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedBodyPathPlanMessage.class, controllerPubGenerator,
                                           s -> bodyPathMultiplexer.setBodyPathPlanMessage(s.takeNextData()));

      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedFootstepStatusMessage.class, controllerPubGenerator, s -> {
         QuadrupedFootstepStatusMessage packet = s.takeNextData();
         if (packet.getFootstepStatus() == QuadrupedFootstepStatusMessage.FOOTSTEP_STATUS_STARTED)
         {
            RobotQuadrant quadrant = RobotQuadrant.fromByte((byte) packet.getFootstepQuadrant());
            bodyPathMultiplexer.startedFootstep(quadrant, packet);
         }
      });

      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedFootstepStatusMessage.class, controllerPubGenerator, s -> {
         QuadrupedFootstepStatusMessage packet = s.takeNextData();
         if (packet.getFootstepStatus() == QuadrupedFootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED)
         {
            RobotQuadrant quadrant = RobotQuadrant.fromByte((byte) packet.getFootstepQuadrant());
            bodyPathMultiplexer.completedFootstep(quadrant, packet);
         }
      });

      MessageTopicNameGenerator controllerSubGenerator = QuadrupedControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);

      timedStepListPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedTimedStepListMessage.class, controllerSubGenerator);
      bodyOrientationPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedBodyOrientationMessage.class, controllerSubGenerator);
      bodyHeightPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedBodyHeightMessage.class, controllerSubGenerator);

      parentRegistry.addChild(registry);
   }

   public void publishTimedStepListToController(QuadrupedTimedStepListMessage message)
   {
      timedStepListPublisher.publish(message);
   }

   public void update()
   {
      limitedDesiredVelocity.update();

      timestamp.set(Conversions.nanosecondsToSeconds(timestampNanos.get()));
      bodyPathMultiplexer.setPlanarVelocityForJoystickPath(limitedDesiredVelocity.getX(), limitedDesiredVelocity.getY(), limitedDesiredVelocity.getZ());
      referenceFrames.updateFrames();

      if (paused.get())
         return;

      if (xGaitRequested.getValue() && !isInStepState())
      {
         xGaitRequested.set(false);
         stepStream.onEntry();
         sendSteps();
         walking.set(true);
      }
      else if (isInBalancingState() && isInStepState())
      {
         sendDesiredBodyHeight();

         if (walking.getBooleanValue())
         {
            stepStream.process();
            sendSteps();
         }
      }
   }

   public void setDesiredVelocity(double desiredVelocityX, double desiredVelocityY, double desiredVelocityZ)
   {
      this.desiredVelocity.set(desiredVelocityX, desiredVelocityY, desiredVelocityZ);
   }


   public void requestXGait()
   {
      xGaitRequested.set(true);
   }

   private boolean isInBalancingState()
   {
      HighLevelStateChangeStatusMessage controllerStateChangeMessage = this.controllerStateChangeMessage.get();
      return (controllerStateChangeMessage != null && controllerStateChangeMessage.getEndHighLevelControllerName() == HighLevelControllerName.WALKING.toByte());
   }

   private boolean isInStepState()
   {
      QuadrupedSteppingStateChangeMessage steppingStateChangeMessage = this.steppingStateChangeMessage.get();
      return isInBalancingState() && (steppingStateChangeMessage != null
            && steppingStateChangeMessage.getEndQuadrupedSteppingStateEnum() == QuadrupedSteppingStateEnum.STEP.toByte());
   }

   public boolean isWalking()
   {
      return walking.getBooleanValue();
   }

   public void requestStanding()
   {
      walking.set(false);
   }

   private void sendSteps()
   {
      List<? extends QuadrupedTimedStep> steps = stepStream.getSteps();
      List<QuadrupedTimedStepMessage> stepMessages = new ArrayList<>();
      for (int i = 0; i < steps.size(); i++)
         stepMessages.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(steps.get(i)));

      QuadrupedTimedStepListMessage stepsMessage = QuadrupedMessageTools.createQuadrupedTimedStepListMessage(stepMessages, true);
      timedStepListPublisher.publish(stepsMessage);

      QuadrupedBodyOrientationMessage orientationMessage = QuadrupedMessageTools
            .createQuadrupedWorldFrameYawMessage(getPlannedStepsSortedByEndTime(), limitedDesiredVelocity.getZ());
      bodyOrientationPublisher.publish(orientationMessage);
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

   private void sendDesiredBodyHeight()
   {
      double bodyHeight = desiredBodyHeight.getAndSet(Double.NaN);

      if (!Double.isNaN(bodyHeight))
      {
         tempPoint.setIncludingFrame(referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds(), 0.0, 0.0, bodyHeight);
         tempPoint.changeFrame(ReferenceFrame.getWorldFrame());

         QuadrupedBodyHeightMessage bodyHeightMessage = QuadrupedMessageTools.createQuadrupedBodyHeightMessage(0.0, tempPoint.getZ());
         bodyHeightMessage.setControlBodyHeight(true);
         bodyHeightMessage.setIsExpressedInAbsoluteTime(false);
         bodyHeightPublisher.publish(bodyHeightMessage);
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
      walking.set(false);
   }

   public boolean isPaused()
   {
      return paused.get();
   }

   public QuadrupedReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }

   public Ros2Node getRos2Node()
   {
      return ros2Node;
   }
}