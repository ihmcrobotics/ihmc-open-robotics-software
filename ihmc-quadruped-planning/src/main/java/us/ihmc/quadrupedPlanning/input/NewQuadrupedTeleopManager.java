package us.ihmc.quadrupedPlanning.input;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.quadrupedBasics.QuadrupedSteppingRequestedEvent;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapper;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

public class NewQuadrupedTeleopManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble timestamp = new YoDouble("timestamp", registry);
   private final Ros2Node ros2Node;

   private final YoEnum<HighLevelControllerName> controllerRequestedEvent = new YoEnum<>("teleopControllerRequestedEvent", registry,
                                                                                         HighLevelControllerName.class, true);

   private final YoBoolean standingRequested = new YoBoolean("standingRequested", registry);
   private final AtomicBoolean paused = new AtomicBoolean(false);

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();
   private final AtomicLong timestampNanos = new AtomicLong();

   private final QuadrupedReferenceFrames referenceFrames;
   private final IHMCROS2Publisher<HighLevelStateMessage> controllerStatePublisher;
   private final IHMCROS2Publisher<QuadrupedRequestedSteppingStateMessage> steppingStatePublisher;

   private final QuadrupedBodyTeleopManager bodyTeleopManager;
   private final QuadrupedStepTeleopManager stepTeleopManager;

   public NewQuadrupedTeleopManager(String robotName, Ros2Node ros2Node, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, double initialBodyHeight,
                                    QuadrupedReferenceFrames referenceFrames, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this(robotName, ros2Node, defaultXGaitSettings, initialBodyHeight, referenceFrames, 0.01, graphicsListRegistry, parentRegistry);
   }

   public NewQuadrupedTeleopManager(String robotName, Ros2Node ros2Node, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, double initialBodyHeight,
                                    QuadrupedReferenceFrames referenceFrames, double updateDT, YoGraphicsListRegistry graphicsListRegistry,
                                    YoVariableRegistry parentRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.ros2Node = ros2Node;

      bodyTeleopManager = new QuadrupedBodyTeleopManager(robotName, ros2Node, initialBodyHeight, referenceFrames, registry);
      stepTeleopManager = new QuadrupedStepTeleopManager(robotName, ros2Node, defaultXGaitSettings, initialBodyHeight, referenceFrames, updateDT, graphicsListRegistry, registry);

      controllerRequestedEvent.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            HighLevelControllerName requestedState = controllerRequestedEvent.getEnumValue();
            if (requestedState != null)
            {
               controllerRequestedEvent.set(null);
               HighLevelStateMessage controllerMessage = new HighLevelStateMessage();
               controllerMessage.setHighLevelControllerName(requestedState.toByte());
               controllerStatePublisher.publish(controllerMessage);
            }
         }
      });

      MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(ros2Node, HighLevelStateChangeStatusMessage.class, controllerPubGenerator,
                                           s -> controllerStateChangeMessage.set(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedSteppingStateChangeMessage.class, controllerPubGenerator,
                                           s -> steppingStateChangeMessage.set(s.takeNextData()));
      ROS2Tools
            .createCallbackSubscription(ros2Node, RobotConfigurationData.class, controllerPubGenerator, s -> timestampNanos.set(s.takeNextData().timestamp_));
      ROS2Tools.createCallbackSubscription(ros2Node, HighLevelStateMessage.class, controllerPubGenerator, s -> paused.set(true));



      MessageTopicNameGenerator controllerSubGenerator = QuadrupedControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);

      controllerStatePublisher = ROS2Tools.createPublisher(ros2Node, HighLevelStateMessage.class, controllerSubGenerator);
      steppingStatePublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedRequestedSteppingStateMessage.class, controllerSubGenerator);

      parentRegistry.addChild(registry);
   }

   public void publishTimedStepListToController(QuadrupedTimedStepListMessage message)
   {
      stepTeleopManager.publishTimedStepListToController(message);
   }

   public void update()
   {
      bodyTeleopManager.update();

      timestamp.set(Conversions.nanosecondsToSeconds(timestampNanos.get()));
      referenceFrames.updateFrames();

      if (paused.get())
      {
         return;
      }
      else if (isInStepState())
      {
         stepTeleopManager.update();
      }
      else if (standingRequested.getBooleanValue())
      {
         standingRequested.set(false);
         sendStopWalkingRequest();
      }
      else if (isInBalancingState())
      {
         bodyTeleopManager.update();
      }

   }

   public void setDesiredVelocity(double desiredVelocityX, double desiredVelocityY, double desiredVelocityZ)
   {
      stepTeleopManager.setDesiredVelocity(desiredVelocityX, desiredVelocityY, desiredVelocityZ);
   }

   public void requestStandPrep()
   {
      HighLevelStateMessage controllerMessage = new HighLevelStateMessage();
      controllerMessage.setHighLevelControllerName(HighLevelStateMessage.STAND_PREP_STATE);
      controllerStatePublisher.publish(controllerMessage);
   }

   public void requestSteppingState()
   {
      HighLevelStateMessage controllerMessage = new HighLevelStateMessage();
      controllerMessage.setHighLevelControllerName(HighLevelStateMessage.STAND_TRANSITION_STATE);
      controllerStatePublisher.publish(controllerMessage);
   }

   private void sendStopWalkingRequest()
   {
      QuadrupedRequestedSteppingStateMessage steppingMessage = new QuadrupedRequestedSteppingStateMessage();
      steppingMessage.setQuadrupedSteppingRequestedEvent(QuadrupedSteppingRequestedEvent.REQUEST_STAND.toByte());
      steppingStatePublisher.publish(steppingMessage);
   }

   public void requestXGait()
   {
      stepTeleopManager.requestXGait();
   }

   private boolean isInBalancingState()
   {
      HighLevelStateChangeStatusMessage controllerStateChangeMessage = this.controllerStateChangeMessage.get();
      return (controllerStateChangeMessage != null && controllerStateChangeMessage.getEndHighLevelControllerName() == HighLevelControllerName.WALKING.toByte());
   }

   public boolean isInStepState()
   {
      QuadrupedSteppingStateChangeMessage steppingStateChangeMessage = this.steppingStateChangeMessage.get();
      return isInBalancingState() && (steppingStateChangeMessage != null
            && steppingStateChangeMessage.getEndQuadrupedSteppingStateEnum() == QuadrupedSteppingStateEnum.STEP.toByte());
   }

   public boolean isInStandState()
   {
      QuadrupedSteppingStateChangeMessage steppingStateChangeMessage = this.steppingStateChangeMessage.get();
      return isInBalancingState() && (steppingStateChangeMessage != null
            && steppingStateChangeMessage.getEndQuadrupedSteppingStateEnum() == QuadrupedSteppingStateEnum.STAND.toByte());
   }

   public boolean isWalking()
   {
      return stepTeleopManager.isWalking();
   }

   public void requestStanding()
   {
      standingRequested.set(true);
      stepTeleopManager.requestStanding();
   }


   public void setDesiredBodyHeight(double desiredBodyHeight)
   {
      bodyTeleopManager.setDesiredBodyHeight(desiredBodyHeight);
   }

   public void setDesiredBodyOrientation(double yaw, double pitch, double roll, double time)
   {
      bodyTeleopManager.setDesiredBodyOrientation(yaw, pitch, roll, time);
   }

   public void setDesiredBodyPose(double x, double y, double yaw, double pitch, double roll, double time)
   {
      bodyTeleopManager.setDesiredBodyPose(x, y, yaw, pitch, roll, time);
   }



   public void setStepSnapper(PointFootSnapper stepSnapper)
   {
      stepTeleopManager.setStepSnapper(stepSnapper);
   }

   public YoQuadrupedXGaitSettings getXGaitSettings()
   {
      return stepTeleopManager.getXGaitSettings();
   }

   public void setShiftPlanBasedOnStepAdjustment(boolean shiftPlanBasedOnStepAdjustment)
   {
      stepTeleopManager.setShiftPlanBasedOnStepAdjustment(shiftPlanBasedOnStepAdjustment);
   }

   public void handleBodyPathPlanMessage(QuadrupedBodyPathPlanMessage bodyPathPlanMessage)
   {
      stepTeleopManager.handleBodyPathPlanMessage(bodyPathPlanMessage);
   }

   public void setPaused(boolean pause)
   {
      paused.set(pause);

      stepTeleopManager.setPaused(pause);

      steppingStateChangeMessage.set(null);
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