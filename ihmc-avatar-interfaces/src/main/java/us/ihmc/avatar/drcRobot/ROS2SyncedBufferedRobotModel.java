package us.ihmc.avatar.drcRobot;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.ros2.ROS2NodeInterface;

import java.util.Iterator;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * As soon as you ask for a point in time, it's assumed you're caught up to then, so it remove the older stuff.
 */
public class ROS2SyncedBufferedRobotModel extends ROS2SyncedRobotModel
{
   public static final int HISTORY_LIMIT = 3000; // 3 seconds for our fastest (1 kHz) publishers
   private final ConcurrentLinkedQueue<RobotConfigurationData> robotConfigurationDataBuffer = new ConcurrentLinkedQueue<>();
   private final RigidBodyTransform interpolatedTransform = new RigidBodyTransform();
   private long monotonicNanos = -1;
   private RobotConfigurationData nextRobotConfigrationData;

   public ROS2SyncedBufferedRobotModel(DRCRobotModel robotModel, ROS2NodeInterface ros2Node)
   {
      super(robotModel, ros2Node);

      addRobotConfigurationDataReceivedCallback(robotConfigurationData ->
      {
         robotConfigurationDataBuffer.add(robotConfigurationData);
         if (robotConfigurationDataBuffer.size() > HISTORY_LIMIT)
         {
            robotConfigurationDataBuffer.remove();
         }
      });
   }

   /**
    * Assumes that there's some history. You should use getDataReceptionTimerSnapshot to check before calling this.
    */
   public void updateToBuffered(long monotonicNanos)
   {
      this.monotonicNanos = monotonicNanos;

      Iterator<RobotConfigurationData> iterator = robotConfigurationDataBuffer.iterator();
      robotConfigurationData = iterator.next();
      nextRobotConfigrationData = iterator.next();
      while (nextRobotConfigrationData.getMonotonicTime() < monotonicNanos)
      {
         robotConfigurationData = nextRobotConfigrationData;
         nextRobotConfigrationData = iterator.next();
      }

      updateInternal();
   }

   /**
    * Call right after updateToBuffered.
    */
   public RigidBodyTransformReadOnly interpolatedTransformToWorldFrame(ReferenceFrame referenceFrame)
   {
      if (nextRobotConfigrationData.getMonotonicTime() - robotConfigurationData.getMonotonicTime() < 100000)
      {
         return referenceFrame.getTransformToWorldFrame(); // no interpolation required
      }

      interpolatedTransform.set(referenceFrame.getTransformToWorldFrame());
      long priorTickTime = robotConfigurationData.getMonotonicTime();
      robotConfigurationData = nextRobotConfigrationData;
      long nextTickTime = robotConfigurationData.getMonotonicTime();
      updateInternal();

      long queriedTime = monotonicNanos - priorTickTime;
      long dt = nextTickTime - priorTickTime;
      long alpha = queriedTime / dt;

      interpolatedTransform.interpolate(referenceFrame.getTransformToWorldFrame(), alpha);

      return interpolatedTransform;
   }
}
