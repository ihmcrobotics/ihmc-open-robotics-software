package us.ihmc.humanoidBehaviors.behaviors.primitives;

import org.apache.commons.lang3.StringUtils;

import controller_msgs.msg.dds.FootTrajectoryMessage;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class FootTrajectoryBehavior extends AbstractBehavior
{
   private FootTrajectoryMessage outgoingFootTrajectoryMessage;

   private final YoBoolean hasPacketBeenSent;
   private final YoDouble yoTime;
   private final YoDouble startTime;
   private final YoDouble trajectoryTime;
   private final YoBoolean trajectoryTimeElapsed;
   private final YoBoolean doubleSupport;

   private final ROS2PublisherBasics<FootTrajectoryMessage> publisher;

   public FootTrajectoryBehavior(String robotName, ROS2Node ros2Node, YoDouble yoTime, YoBoolean yoDoubleSupport)
   {
      super(robotName, ros2Node);

      this.yoTime = yoTime;
      this.doubleSupport = yoDoubleSupport;

      String behaviorNameFirstLowerCase = StringUtils.uncapitalize(getName());
      hasPacketBeenSent = new YoBoolean(behaviorNameFirstLowerCase + "HasPacketBeenSent", registry);
      startTime = new YoDouble(behaviorNameFirstLowerCase + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new YoDouble(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      trajectoryTimeElapsed = new YoBoolean(behaviorNameFirstLowerCase + "TrajectoryTimeElapsed", registry);

      publisher = createPublisherForController(FootTrajectoryMessage.class);
   }

   public void setInput(FootTrajectoryMessage message)
   {
      this.outgoingFootTrajectoryMessage = message;
   }

   @Override
   public void doControl()
   {
      if (!hasPacketBeenSent.getBooleanValue() && (outgoingFootTrajectoryMessage != null))
      {
         sendFootPosePacketToController();
      }

      if (hasPacketBeenSent.getBooleanValue() && !isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {
         if (Double.isNaN(startTime.getDoubleValue()) && !doubleSupport.getBooleanValue())
         {
            startTime.set(yoTime.getDoubleValue());
         }
      }
   }

   private void sendFootPosePacketToController()
   {
      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {
         outgoingFootTrajectoryMessage.setDestination(PacketDestination.UI.ordinal());
         publisher.publish(outgoingFootTrajectoryMessage);
         hasPacketBeenSent.set(true);
         trajectoryTime.set(outgoingFootTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().getLast().getTime());
      }
   }

   @Override
   public void onBehaviorEntered()
   {
      hasPacketBeenSent.set(false);
      
      hasBeenInitialized.set(true);
      
      isPaused.set(false);
      isAborted.set(false);
   }

   @Override
   public void onBehaviorExited()
   {
      hasPacketBeenSent.set(false);
      outgoingFootTrajectoryMessage = null;

      isPaused.set(false);
      isAborted.set(false);

      startTime.set(Double.NaN);
      trajectoryTime.set(Double.NaN);
   }



   @Override
   public boolean isDone()
   {
      if (Double.isNaN(startTime.getDoubleValue()) || Double.isNaN(trajectoryTime.getDoubleValue()))
         trajectoryTimeElapsed.set(false);
      else
         trajectoryTimeElapsed.set(yoTime.getDoubleValue() - startTime.getDoubleValue() > trajectoryTime.getDoubleValue());

      return trajectoryTimeElapsed.getBooleanValue() && !isPaused.getBooleanValue();
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }

   

 
}
