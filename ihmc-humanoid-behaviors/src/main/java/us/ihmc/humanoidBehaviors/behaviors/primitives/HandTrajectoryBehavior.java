package us.ihmc.humanoidBehaviors.behaviors.primitives;

import java.util.concurrent.atomic.AtomicReference;

import org.apache.commons.lang3.StringUtils;

import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage;
import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryExecutionStatus;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class HandTrajectoryBehavior extends AbstractBehavior
{
   private static final boolean DEBUG = false;

   protected RobotSide robotSide;

   protected HandTrajectoryMessage outgoingMessage;

   protected final YoBoolean hasPacketBeenSent;
   protected final YoDouble yoTime;
   protected final YoDouble startTime;
   protected final YoDouble trajectoryTime;
   private final YoDouble trajectoryTimeElapsed;

   protected final YoBoolean hasInputBeenSet;
   protected final YoBoolean hasStatusBeenReceived;
   private final YoBoolean isDone;

   private final IHMCROS2Publisher<HandTrajectoryMessage> handTrajectoryPublisher;
   private final IHMCROS2Publisher<StopAllTrajectoryMessage> stopAllTrajectoryPublisher;
   private final ConcurrentListeningQueue<JointspaceTrajectoryStatusMessage> jointSpaceTrajectoryStatus = new ConcurrentListeningQueue<>(10);
   private final ConcurrentListeningQueue<TaskspaceTrajectoryStatusMessage> taskSpaceTrajectoryStatus = new ConcurrentListeningQueue<>(10);

   public HandTrajectoryBehavior(String robotName, Ros2Node ros2Node, YoDouble yoTime)
   {
      this(robotName, null, ros2Node, yoTime);
   }

   public HandTrajectoryBehavior(String robotName, String namePrefix, Ros2Node ros2Node, YoDouble yoTime)
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
      createSubscriberFromController(TaskspaceTrajectoryStatusMessage.class, taskSpaceTrajectoryStatus::put);

      handTrajectoryPublisher = createPublisherForController(HandTrajectoryMessage.class);
      stopAllTrajectoryPublisher = createPublisherForController(StopAllTrajectoryMessage.class);
   }

   public void setInput(HandTrajectoryMessage armTrajectoryMessage)
   {
      outgoingMessage = armTrajectoryMessage;

      robotSide = RobotSide.fromByte(armTrajectoryMessage.getRobotSide());
      startTime.set(yoTime.getDoubleValue());
      trajectoryTime.set(armTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().getLast().getTime());

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
            PrintTools.debug(this, robotSide + " HandTrajectoryBehavior setting isDone = true");
        // isDone.set(true);
      }

      /*
       * if(jointSpaceTrajectoryStatus.get()!=null) {
       * System.out.println("joint "+TrajectoryExecutionStatus.fromByte(
       * jointSpaceTrajectoryStatus.get().getTrajectoryExecutionStatus())+
       * "  "+jointSpaceTrajectoryStatus.get().joint_names_);
       * if(TrajectoryExecutionStatus.fromByte(jointSpaceTrajectoryStatus.get().
       * getTrajectoryExecutionStatus()) == TrajectoryExecutionStatus.COMPLETED)
       * { //isDone.set(true); } }
       */
      
      TaskspaceTrajectoryStatusMessage message;
      while ((message = taskSpaceTrajectoryStatus.poll()) != null)
      {
         System.out.println(robotSide+" HandTrajectoryBehavior recieved the message: "+message.getEndEffectorNameAsString()+" " + TrajectoryExecutionStatus.fromByte(message.getTrajectoryExecutionStatus()));
         String value = "r_hand";
         if (RobotSide.fromByte(outgoingMessage.getRobotSide()) == RobotSide.LEFT)
            value = "l_hand";
         if (message.getEndEffectorNameAsString().equals(value)
               && TrajectoryExecutionStatus.fromByte(message.getTrajectoryExecutionStatus()) == TrajectoryExecutionStatus.COMPLETED)
         {
            System.out.println(" HAND MOTION COMPLETE FOR " + message.getEndEffectorNameAsString());
            isDone.set(true);
         }

      }

      if (!hasPacketBeenSent.getBooleanValue() && (outgoingMessage != null))
      {
         sendOutgoingPacketToControllerAndNetworkProcessor();
      }
   }

   private void sendOutgoingPacketToControllerAndNetworkProcessor()
   {
      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {
         handTrajectoryPublisher.publish(outgoingMessage);

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
      //System.out.println("isDone check "+robotSide+" "+RobotSide.fromByte(outgoingMessage.getRobotSide())+" " +isDone.getBooleanValue());
      return isDone.getBooleanValue();
   }

   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}
