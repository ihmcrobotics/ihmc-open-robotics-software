package us.ihmc.humanoidBehaviors.behaviors.primitives;

import org.apache.commons.lang3.StringUtils;

import controller_msgs.msg.dds.ChestTrajectoryMessage;
import us.ihmc.commons.PrintTools;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class ChestTrajectoryBehavior extends AbstractBehavior
{
   private static final boolean DEBUG = false;

   private ChestTrajectoryMessage outgoingChestTrajectoryMessage;

   private final YoBoolean hasPacketBeenSent;
   private final YoDouble yoTime;
   private final YoDouble startTime;
   private final YoDouble trajectoryTime;
   private final YoBoolean trajectoryTimeHasElapsed;

   private final ROS2PublisherBasics<ChestTrajectoryMessage> publisher;

   public ChestTrajectoryBehavior(String robotName, ROS2Node ros2Node, YoDouble yoTime)
   {
      super(robotName, ros2Node);

      this.yoTime = yoTime;
      String behaviorNameFirstLowerCase = StringUtils.uncapitalize(getName());
      hasPacketBeenSent = new YoBoolean(behaviorNameFirstLowerCase + "HasPacketBeenSent", registry);
      startTime = new YoDouble(behaviorNameFirstLowerCase + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new YoDouble(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      trajectoryTimeHasElapsed = new YoBoolean(behaviorNameFirstLowerCase + "TrajectoryTimeHasElapsed", registry);

      publisher = createPublisherForController(ChestTrajectoryMessage.class);
   }

   public void setInput(ChestTrajectoryMessage chestOrientationPacket)
   {
      this.outgoingChestTrajectoryMessage = chestOrientationPacket;
   }

   @Override
   public void doControl()
   {
      if (!hasPacketBeenSent.getBooleanValue() && (outgoingChestTrajectoryMessage != null))
      {
         sendChestPoseToController();
      }
   }

   private void sendChestPoseToController()
   {
      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {
         publisher.publish(outgoingChestTrajectoryMessage);
         hasPacketBeenSent.set(true);
         startTime.set(yoTime.getDoubleValue());
         trajectoryTime.set(outgoingChestTrajectoryMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().getLast().getTime());
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
      outgoingChestTrajectoryMessage = null;

      isPaused.set(false);
      isAborted.set(false);

      startTime.set(Double.NaN);
      trajectoryTime.set(Double.NaN);
   }

   @Override
   public boolean isDone()
   {
      boolean startTimeUndefined = Double.isNaN(startTime.getDoubleValue());
      boolean trajectoryTimeUndefined = Double.isNaN(trajectoryTime.getDoubleValue());
      double trajectoryTimeElapsed = yoTime.getDoubleValue() - startTime.getDoubleValue();

      if (DEBUG)
      {
         PrintTools.debug(this, "StartTimeUndefined: " + startTimeUndefined + ".  TrajectoryTimeUndefined: " + trajectoryTimeUndefined);
         PrintTools.debug(this, "TrajectoryTimeElapsed: " + trajectoryTimeElapsed);
      }

      if (startTimeUndefined || trajectoryTimeUndefined)
         trajectoryTimeHasElapsed.set(false);
      else
         trajectoryTimeHasElapsed.set(trajectoryTimeElapsed > trajectoryTime.getDoubleValue());

      return trajectoryTimeHasElapsed.getBooleanValue() && !isPaused.getBooleanValue();
   }

   public boolean hasInputBeenSet()
   {
      return outgoingChestTrajectoryMessage != null;
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
