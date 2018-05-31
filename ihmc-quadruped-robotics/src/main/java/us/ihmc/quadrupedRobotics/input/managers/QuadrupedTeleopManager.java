package us.ihmc.quadrupedRobotics.input.managers;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

import com.google.common.util.concurrent.AtomicDouble;

import controller_msgs.msg.dds.ComPositionPacket;
import controller_msgs.msg.dds.QuadrupedBodyOrientationMessage;
import controller_msgs.msg.dds.QuadrupedBodyPathPlanMessage;
import controller_msgs.msg.dds.QuadrupedControllerStateChangeMessage;
import controller_msgs.msg.dds.QuadrupedRequestedControllerStateMessage;
import controller_msgs.msg.dds.QuadrupedRequestedSteppingStateMessage;
import controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage;
import controller_msgs.msg.dds.QuadrupedTimedStepListMessage;
import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.SO3TrajectoryPointMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.communication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedRobotics.communication.QuadrupedMessageTools;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerEnum;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedSteppingRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedRobotics.planning.bodyPath.QuadrupedBodyPathMultiplexer;
import us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser.PlanarGroundPointFootSnapper;
import us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser.PointFootSnapper;
import us.ihmc.quadrupedRobotics.planning.stepStream.QuadrupedXGaitStepStream;
import us.ihmc.quadrupedRobotics.providers.YoQuadrupedXGaitSettings;
import us.ihmc.robotics.math.filters.RateLimitedYoFrameVector;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

public class QuadrupedTeleopManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final QuadrupedXGaitStepStream stepStream;
   private final YoQuadrupedXGaitSettings xGaitSettings;
   private final YoDouble timestamp = new YoDouble("timestamp", registry);
   private final YoBoolean walking = new YoBoolean("walking", registry);

   private final YoBoolean xGaitRequested = new YoBoolean("xGaitRequested", registry);
   private final YoFrameVector3D desiredVelocity = new YoFrameVector3D("teleopDesiredVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble desiredVelocityRateLimit = new YoDouble("teleopDesiredVelocityRateLimit", registry);
   private final YoEnum<QuadrupedControllerRequestedEvent> controllerRequestedEvent = new YoEnum<>("teleopControllerRequestedEvent", registry, QuadrupedControllerRequestedEvent.class, true);
   private final RateLimitedYoFrameVector limitedDesiredVelocity;

   private final AtomicBoolean standingRequested = new AtomicBoolean();
   private final AtomicDouble desiredCoMHeight = new AtomicDouble();
   private final AtomicDouble desiredOrientationYaw = new AtomicDouble();
   private final AtomicDouble desiredOrientationPitch = new AtomicDouble();
   private final AtomicDouble desiredOrientationRoll = new AtomicDouble();
   private final AtomicDouble desiredOrientationTime = new AtomicDouble();

   private final AtomicReference<QuadrupedControllerStateChangeMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();
   private final AtomicLong timestampNanos = new AtomicLong();

   private final ComPositionPacket comPositionPacket = new ComPositionPacket();
   private final QuadrupedBodyOrientationMessage offsetBodyOrientationMessage = new QuadrupedBodyOrientationMessage();
   private final QuadrupedReferenceFrames referenceFrames;
   private final QuadrupedBodyPathMultiplexer bodyPathMultiplexer;
   private final IHMCROS2Publisher<QuadrupedRequestedControllerStateMessage> controllerStatePublisher;
   private final IHMCROS2Publisher<QuadrupedRequestedSteppingStateMessage> steppingStatePublisher;
   private IHMCROS2Publisher<QuadrupedTimedStepListMessage> timedStepListPublisher;
   private IHMCROS2Publisher<QuadrupedBodyOrientationMessage> bodyOrientationPublisher;
   private IHMCROS2Publisher<ComPositionPacket> comPositionPublisher;

   public QuadrupedTeleopManager(String robotName, Ros2Node ros2Node, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                                 double initialCoMHeight, QuadrupedReferenceFrames referenceFrames, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this(robotName, ros2Node, defaultXGaitSettings, initialCoMHeight, referenceFrames, 0.01, graphicsListRegistry, parentRegistry);
   }

   public QuadrupedTeleopManager(String robotName, Ros2Node ros2Node, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, double initialCoMHeight,
                                 QuadrupedReferenceFrames referenceFrames, double updateDT, YoGraphicsListRegistry graphicsListRegistry,
                                 YoVariableRegistry parentRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, null, registry);
      this.bodyPathMultiplexer = new QuadrupedBodyPathMultiplexer(robotName, referenceFrames, timestamp, xGaitSettings, ros2Node, graphicsListRegistry, registry);
      this.stepStream = new QuadrupedXGaitStepStream(xGaitSettings, timestamp, bodyPathMultiplexer, registry);

      desiredVelocityRateLimit.set(10.0);
      limitedDesiredVelocity = new RateLimitedYoFrameVector("limitedTeleopDesiredVelocity", "", registry, desiredVelocityRateLimit, updateDT, desiredVelocity);

      controllerRequestedEvent.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            QuadrupedControllerRequestedEvent requestedEvent = controllerRequestedEvent.getEnumValue();
            if (requestedEvent != null)
            {
               controllerRequestedEvent.set(null);
               QuadrupedRequestedControllerStateMessage controllerMessage = new QuadrupedRequestedControllerStateMessage();
               controllerMessage.setQuadrupedControllerRequestedEvent(requestedEvent.toByte());
               controllerStatePublisher.publish(controllerMessage);
            }
         }
      });

      desiredCoMHeight.set(initialCoMHeight);
      stepStream.setStepSnapper(new PlanarGroundPointFootSnapper(robotName, referenceFrames, ros2Node));

      MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedControllerStateChangeMessage.class, controllerPubGenerator, s -> controllerStateChangeMessage.set(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedSteppingStateChangeMessage.class, controllerPubGenerator, s -> steppingStateChangeMessage.set(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, controllerPubGenerator, s -> timestampNanos.set(s.takeNextData().timestamp_));

      MessageTopicNameGenerator controllerSubGenerator = QuadrupedControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);

      controllerStatePublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedRequestedControllerStateMessage.class, controllerSubGenerator);
      steppingStatePublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedRequestedSteppingStateMessage.class, controllerSubGenerator);
      timedStepListPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedTimedStepListMessage.class, controllerSubGenerator);
      bodyOrientationPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedBodyOrientationMessage.class, controllerSubGenerator);
      comPositionPublisher = ROS2Tools.createPublisher(ros2Node, ComPositionPacket.class, controllerSubGenerator);

      parentRegistry.addChild(registry);
   }

   public void pulishTimedStepListToController(QuadrupedTimedStepListMessage message)
   {
      timedStepListPublisher.publish(message);
   }

   public void update()
   {
      limitedDesiredVelocity.update();

      timestamp.set(Conversions.nanosecondsToSeconds(timestampNanos.get()));
      bodyPathMultiplexer.setPlanarVelocityForJoystickPath(limitedDesiredVelocity.getX(), limitedDesiredVelocity.getY(), limitedDesiredVelocity.getZ());
      referenceFrames.updateFrames();

      if (xGaitRequested.getValue() && !isInStepState())
      {
         xGaitRequested.set(false);
         stepStream.onEntry();
         sendSteps();
         walking.set(true);
      }
      else if (standingRequested.getAndSet(false))
      {
         requestStopWalking();
         walking.set(false);
      }
      else if (isInStepState() && walking.getBooleanValue())
      {
         stepStream.process();
         sendSteps();
      }
      else if (!walking.getBooleanValue())
      {
         sendBodyPose();
      }
   }

   public void setDesiredVelocity(double desiredVelocityX, double desiredVelocityY, double desiredVelocityZ)
   {
      this.desiredVelocity.set(desiredVelocityX, desiredVelocityY, desiredVelocityZ);
   }

   public void requestSteppingState()
   {
      QuadrupedRequestedControllerStateMessage controllerMessage = new QuadrupedRequestedControllerStateMessage();
      controllerMessage.setQuadrupedControllerRequestedEvent(QuadrupedControllerRequestedEvent.REQUEST_STEPPING.toByte());
      controllerStatePublisher.publish(controllerMessage);
   }

   public void requestStopWalking()
   {
      QuadrupedRequestedSteppingStateMessage steppingMessage = new QuadrupedRequestedSteppingStateMessage();
      steppingMessage.setQuadrupedSteppingRequestedEvent(QuadrupedSteppingRequestedEvent.REQUEST_STAND.toByte());
      steppingStatePublisher.publish(steppingMessage);
   }

   public void requestXGait()
   {
      xGaitRequested.set(true);
   }

   private boolean isInStepState()
   {
      QuadrupedControllerStateChangeMessage controllerStateChangeMessage = this.controllerStateChangeMessage.get();
      QuadrupedSteppingStateChangeMessage steppingStateChangeMessage = this.steppingStateChangeMessage.get();

      return (controllerStateChangeMessage != null && controllerStateChangeMessage.getEndQuadrupedControllerEnum() == QuadrupedControllerEnum.STEPPING.toByte()) &&
            (steppingStateChangeMessage != null && steppingStateChangeMessage.getEndQuadrupedSteppingStateEnum() == QuadrupedSteppingStateEnum.STEP.toByte());
   }

   public boolean isWalking()
   {
      return walking.getBooleanValue();
   }

   public void requestStanding()
   {
      standingRequested.set(true);
   }

   private void sendSteps()
   {
      List<? extends QuadrupedTimedStep> steps = stepStream.getSteps();
      List<QuadrupedTimedStepMessage> stepMessages = new ArrayList<>();
      for (int i = 0; i < steps.size(); i++)
         stepMessages.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(steps.get(i)));

      QuadrupedTimedStepListMessage stepsMessage = QuadrupedMessageTools.createQuadrupedTimedStepListMessage(stepMessages, true);
      timedStepListPublisher.publish(stepsMessage);

      QuadrupedBodyOrientationMessage orientationMessage = QuadrupedMessageTools.createQuadrupedWorldFrameYawMessage(stepStream.getFootstepPlan().getPlannedSteps(), limitedDesiredVelocity.getZ());
      bodyOrientationPublisher.publish(orientationMessage);
   }

   public void setDesiredCoMHeight(double desiredCoMHeight)
   {
      this.desiredCoMHeight.set(desiredCoMHeight);
   }

   public void setDesiredBodyOrientation(double yaw, double pitch, double roll, double time)
   {
      desiredOrientationYaw.set(yaw);
      desiredOrientationPitch.set(pitch);
      desiredOrientationRoll.set(roll);
      desiredOrientationTime.set(time);
   }

   private void sendBodyPose()
   {
      double comHeight = desiredCoMHeight.getAndSet(Double.NaN);
      double desiredYaw = desiredOrientationYaw.getAndSet(Double.NaN);
      double desiredPitch = desiredOrientationPitch.getAndSet(Double.NaN);
      double desiredRoll = desiredOrientationRoll.getAndSet(Double.NaN);
      double desiredTime = desiredOrientationTime.getAndSet(Double.NaN);

      if (!Double.isNaN(comHeight))
      {
         comPositionPacket.getPosition().set(0.0, 0.0, comHeight);
         comPositionPublisher.publish(comPositionPacket);
      }

      if (!Double.isNaN(desiredYaw))
      {
         offsetBodyOrientationMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().clear();
         offsetBodyOrientationMessage.setIsAnOffsetOrientation(true);
         SO3TrajectoryPointMessage trajectoryPointMessage = offsetBodyOrientationMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().add();
         trajectoryPointMessage.getOrientation().setYawPitchRoll(desiredYaw, desiredPitch, desiredRoll);
         trajectoryPointMessage.setTime(desiredTime);
         bodyOrientationPublisher.publish(offsetBodyOrientationMessage);
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

   public void handleBodyPathPlanMessage(QuadrupedBodyPathPlanMessage bodyPathPlanMessage)
   {
      bodyPathMultiplexer.initialize();
      bodyPathMultiplexer.handleBodyPathPlanMessage(bodyPathPlanMessage);
   }
}