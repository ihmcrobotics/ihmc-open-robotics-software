package us.ihmc.humanoidBehaviors.behaviors.primitives;

import org.apache.commons.lang3.StringUtils;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage;
import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryExecutionStatus;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class ArmTrajectoryBehavior extends AbstractBehavior
{
   private static final boolean DEBUG = false;

   protected RobotSide robotSide;

   protected ArmTrajectoryMessage outgoingMessage;

   protected final YoBoolean hasPacketBeenSent;
   protected final YoDouble yoTime;
   protected final YoDouble startTime;
   protected final YoDouble trajectoryTime;
   private final YoDouble trajectoryTimeElapsed;

   protected final YoBoolean hasInputBeenSet;
   protected final YoBoolean hasStatusBeenReceived;
   private final YoBoolean isDone;

   private final IHMCROS2Publisher<ArmTrajectoryMessage> armTrajectoryPublisher;
   private final IHMCROS2Publisher<StopAllTrajectoryMessage> stopAllTrajectoryPublisher;

   private final ConcurrentListeningQueue<JointspaceTrajectoryStatusMessage> jointSpaceTrajectoryStatus = new ConcurrentListeningQueue<>(10);

   public ArmTrajectoryBehavior(String robotName, Ros2Node ros2Node, YoDouble yoTime)
   {
      this(robotName, null, ros2Node, yoTime);
   }

   public ArmTrajectoryBehavior(String robotName, String namePrefix, Ros2Node ros2Node, YoDouble yoTime)
   {
      super(robotName, namePrefix, ros2Node);

      this.yoTime = yoTime;
      String behaviorNameFirstLowerCase = StringUtils.uncapitalize(getName());
      hasPacketBeenSent = new YoBoolean(behaviorNameFirstLowerCase + "HasPacketBeenSent", registry);
      startTime = new YoDouble(behaviorNameFirstLowerCase + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new YoDouble(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      trajectoryTimeElapsed = new YoDouble(behaviorNameFirstLowerCase + "TrajectoryTimeElapsed", registry);
      trajectoryTimeElapsed.set(Double.NaN);

      hasInputBeenSet = new YoBoolean(behaviorNameFirstLowerCase + "HasInputBeenSet", registry);
      hasStatusBeenReceived = new YoBoolean(behaviorNameFirstLowerCase + "HasStatusBeenReceived", registry);
      isDone = new YoBoolean(behaviorNameFirstLowerCase + "IsDone", registry);

      createSubscriberFromController(JointspaceTrajectoryStatusMessage.class, jointSpaceTrajectoryStatus::put);

      armTrajectoryPublisher = createPublisherForController(ArmTrajectoryMessage.class);
      stopAllTrajectoryPublisher = createPublisherForController(StopAllTrajectoryMessage.class);
   }

   public void setInput(ArmTrajectoryMessage armTrajectoryMessage)
   {
      outgoingMessage = armTrajectoryMessage;

      robotSide = RobotSide.fromByte(armTrajectoryMessage.getRobotSide());
      startTime.set(yoTime.getDoubleValue());
      trajectoryTime.set(HumanoidMessageTools.unpackTrajectoryTime(armTrajectoryMessage.getJointspaceTrajectory()));

      hasInputBeenSet.set(true);
   }

   @Override
   public void doControl()
   {
      trajectoryTimeElapsed.set(yoTime.getDoubleValue() - startTime.getDoubleValue());

      if (!isDone.getBooleanValue() && hasInputBeenSet() && !isPaused.getBooleanValue() && !isAborted.getBooleanValue()
            && trajectoryTimeElapsed.getDoubleValue() > trajectoryTime.getDoubleValue())
      {
         if (DEBUG)
            PrintTools.debug(this, robotSide + " ArmTrajectoryBehavior setting isDone = true");
         //  isDone.set(true);
      }

      if (!hasPacketBeenSent.getBooleanValue() && (outgoingMessage != null))
      {
         sendOutgoingPacketToControllerAndNetworkProcessor();
      }

      JointspaceTrajectoryStatusMessage jointSpaceMessage;
      while ((jointSpaceMessage = jointSpaceTrajectoryStatus.poll()) != null)
      {
      //   System.out.println(robotSide + " ArmTrajectoryBehavior recieved the message: " + jointSpaceMessage.getJointNames().get(0).toString() + " "
      //         + TrajectoryExecutionStatus.fromByte(jointSpaceMessage.getTrajectoryExecutionStatus()));
         String value = "r_arm";
         if (robotSide == RobotSide.LEFT)
            value = "l_arm";
        // System.out.println("*****"+jointSpaceMessage.getJointNames().get(0).toString().contains(value));
         if (jointSpaceMessage.getJointNames().get(0).toString().contains(value)
               && TrajectoryExecutionStatus.fromByte(jointSpaceMessage.getTrajectoryExecutionStatus()) == TrajectoryExecutionStatus.COMPLETED)
         {
         //   System.out.println(" Arm MOTION COMPLETE FOR " + jointSpaceMessage.getJointNames().toString());
            isDone.set(true);
         }

      }

   }

   private void sendOutgoingPacketToControllerAndNetworkProcessor()
   {
      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {
         armTrajectoryPublisher.publish(outgoingMessage);

         hasPacketBeenSent.set(true);

         if (DEBUG)
            PrintTools.debug(this, "sending packet to controller and network processor: " + outgoingMessage);
      }
   }

   private void stopArmMotion()
   {
      if (outgoingMessage != null)
      {
         stopAllTrajectoryPublisher.publish(new StopAllTrajectoryMessage());
      }
   }

   @Override
   public void onBehaviorEntered()
   {

      jointSpaceTrajectoryStatus.clear();
      hasInputBeenSet.set(false);
      hasPacketBeenSent.set(false);
      outgoingMessage = null;

      hasStatusBeenReceived.set(false);
      isPaused.set(false);
      isDone.set(false);
      hasBeenInitialized.set(true);

      trajectoryTime.set(Double.NaN);
      startTime.set(Double.NaN);
      trajectoryTimeElapsed.set(Double.NaN);
   }

   @Override
   public void onBehaviorExited()
   {
      publishTextToSpeech("****************ARM MOTION COMPLETE****************");
      hasPacketBeenSent.set(false);
      outgoingMessage = null;

      isPaused.set(false);
      isAborted.set(false);

      hasInputBeenSet.set(false);
      hasStatusBeenReceived.set(false);

      trajectoryTime.set(Double.NaN);
      startTime.set(Double.NaN);
      trajectoryTimeElapsed.set(Double.NaN);

      isDone.set(false);
   }

   @Override
   public void onBehaviorAborted()
   {
      stopArmMotion();
      isAborted.set(true);
   }

   @Override
   public void onBehaviorPaused()
   {

         stopArmMotion();
      
   }

   @Override
   public void onBehaviorResumed()
   {
      

         if (hasInputBeenSet())
         {
            sendOutgoingPacketToControllerAndNetworkProcessor();
         }
      
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}
