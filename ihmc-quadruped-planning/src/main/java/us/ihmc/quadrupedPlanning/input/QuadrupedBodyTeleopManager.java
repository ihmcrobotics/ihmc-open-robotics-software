package us.ihmc.quadrupedPlanning.input;

import com.google.common.util.concurrent.AtomicDouble;
import controller_msgs.msg.dds.*;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedBodyTeleopManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble timestamp = new YoDouble("timestamp", registry);
   private final Ros2Node ros2Node;

   private final AtomicBoolean paused = new AtomicBoolean(false);
   private final AtomicDouble desiredBodyHeight = new AtomicDouble();
   private final AtomicDouble desiredPositionX = new AtomicDouble();
   private final AtomicDouble desiredPositionY = new AtomicDouble();
   private final AtomicDouble desiredOrientationYaw = new AtomicDouble();
   private final AtomicDouble desiredOrientationPitch = new AtomicDouble();
   private final AtomicDouble desiredOrientationRoll = new AtomicDouble();
   private final AtomicDouble desiredOrientationTime = new AtomicDouble();

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicLong timestampNanos = new AtomicLong();

   private final QuadrupedBodyOrientationMessage offsetBodyOrientationMessage = new QuadrupedBodyOrientationMessage();
   private final QuadrupedBodyTrajectoryMessage bodyTrajectoryMessage = new QuadrupedBodyTrajectoryMessage();
   private final QuadrupedReferenceFrames referenceFrames;

   private IHMCROS2Publisher<QuadrupedBodyOrientationMessage> bodyOrientationPublisher;
   private IHMCROS2Publisher<QuadrupedBodyTrajectoryMessage> bodyTrajectoryPublisher;
   private IHMCROS2Publisher<QuadrupedBodyHeightMessage> bodyHeightPublisher;

   public QuadrupedBodyTeleopManager(String robotName, Ros2Node ros2Node, double initialBodyHeight, QuadrupedReferenceFrames referenceFrames,
                                     YoVariableRegistry parentRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.ros2Node = ros2Node;

      desiredBodyHeight.set(initialBodyHeight);

      MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(ros2Node, HighLevelStateChangeStatusMessage.class, controllerPubGenerator,
                                           s -> controllerStateChangeMessage.set(s.takeNextData()));
      ROS2Tools
            .createCallbackSubscription(ros2Node, RobotConfigurationData.class, controllerPubGenerator, s -> timestampNanos.set(s.takeNextData().timestamp_));
      ROS2Tools.createCallbackSubscription(ros2Node, HighLevelStateMessage.class, controllerPubGenerator, s -> paused.set(true));

      MessageTopicNameGenerator controllerSubGenerator = QuadrupedControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);

      bodyOrientationPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedBodyOrientationMessage.class, controllerSubGenerator);
      bodyTrajectoryPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedBodyTrajectoryMessage.class, controllerSubGenerator);
      bodyHeightPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedBodyHeightMessage.class, controllerSubGenerator);

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      timestamp.set(Conversions.nanosecondsToSeconds(timestampNanos.get()));
      referenceFrames.updateFrames();

      if (paused.get())
         return;

      if (isInBalancingState())
      {
         sendDesiredBodyHeight();
         sendDesiredBodyTrajectory();
         //            sendDesiredBodyOrientation();
      }
   }

   private boolean isInBalancingState()
   {
      HighLevelStateChangeStatusMessage controllerStateChangeMessage = this.controllerStateChangeMessage.get();
      return (controllerStateChangeMessage != null && controllerStateChangeMessage.getEndHighLevelControllerName() == HighLevelControllerName.WALKING.toByte());
   }

   public void setDesiredBodyHeight(double desiredBodyHeight)
   {
      this.desiredBodyHeight.set(desiredBodyHeight);
   }

   public void setDesiredBodyOrientation(double yaw, double pitch, double roll, double time)
   {
      desiredOrientationYaw.set(yaw);
      desiredOrientationPitch.set(pitch);
      desiredOrientationRoll.set(roll);
      desiredOrientationTime.set(time);
   }

   public void setDesiredBodyPose(double x, double y, double yaw, double pitch, double roll, double time)
   {
      desiredPositionX.set(x);
      desiredPositionY.set(y);

      desiredOrientationYaw.set(yaw);
      desiredOrientationPitch.set(pitch);
      desiredOrientationRoll.set(roll);
      desiredOrientationTime.set(time);
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

   private void sendDesiredBodyOrientation()
   {
      double desiredYaw = desiredOrientationYaw.getAndSet(Double.NaN);
      double desiredPitch = desiredOrientationPitch.getAndSet(Double.NaN);
      double desiredRoll = desiredOrientationRoll.getAndSet(Double.NaN);
      double desiredTime = desiredOrientationTime.getAndSet(Double.NaN);

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

   private void sendDesiredBodyTrajectory()
   {
      double desiredX = desiredPositionX.getAndSet(Double.NaN);
      double desiredY = desiredPositionY.getAndSet(Double.NaN);
      double bodyHeight = desiredBodyHeight.getAndSet(Double.NaN);

      double desiredYaw = desiredOrientationYaw.getAndSet(Double.NaN);
      double desiredPitch = desiredOrientationPitch.getAndSet(Double.NaN);
      double desiredRoll = desiredOrientationRoll.getAndSet(Double.NaN);
      double desiredTime = desiredOrientationTime.getAndSet(Double.NaN);

      SE3TrajectoryMessage se3Trajectory = bodyTrajectoryMessage.getSe3Trajectory();
      se3Trajectory.getTaskspaceTrajectoryPoints().clear();
      se3Trajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      se3Trajectory.getAngularSelectionMatrix().setXSelected(false);
      se3Trajectory.getAngularSelectionMatrix().setYSelected(false);
      se3Trajectory.getAngularSelectionMatrix().setZSelected(false);
      se3Trajectory.getLinearSelectionMatrix().setXSelected(false);
      se3Trajectory.getLinearSelectionMatrix().setYSelected(false);
      se3Trajectory.getLinearSelectionMatrix().setZSelected(false);
      SE3TrajectoryPointMessage trajectoryPointMessage = se3Trajectory.getTaskspaceTrajectoryPoints().add();

      if (!Double.isNaN(desiredYaw))
      {
         trajectoryPointMessage.getOrientation().setYawPitchRoll(desiredYaw, desiredPitch, desiredRoll);
         se3Trajectory.getAngularSelectionMatrix().setXSelected(true);
         se3Trajectory.getAngularSelectionMatrix().setYSelected(true);
         se3Trajectory.getAngularSelectionMatrix().setZSelected(true);
      }
      if (!Double.isNaN(bodyHeight))
      {
         tempPoint.setIncludingFrame(referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds(), 0.0, 0.0, bodyHeight);
         tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
         trajectoryPointMessage.getPosition().setZ(tempPoint.getZ());
         se3Trajectory.getLinearSelectionMatrix().setZSelected(true);
      }
      if (!Double.isNaN(desiredX) && !Double.isNaN(desiredY))
      {
         trajectoryPointMessage.getPosition().setX(desiredX);
         trajectoryPointMessage.getPosition().setY(desiredY);
         se3Trajectory.getLinearSelectionMatrix().setXSelected(true);
         se3Trajectory.getLinearSelectionMatrix().setYSelected(true);
      }
      trajectoryPointMessage.setTime(desiredTime);
      bodyTrajectoryPublisher.publish(bodyTrajectoryMessage);
   }

   public void setPaused(boolean pause)
   {
      paused.set(pause);
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