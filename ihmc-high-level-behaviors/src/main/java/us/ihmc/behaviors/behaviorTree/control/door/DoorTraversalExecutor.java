package us.ihmc.behaviors.behaviorTree.control.door;

import controller_msgs.msg.dds.RobotConfigurationData;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.actions.FootstepPlanActionExecutor;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ros2log.ROS2LogRecord;
import us.ihmc.communication.ros2log.ROS2LogSerialization;
import us.ihmc.communication.ros2log.ROS2LogTimeSource;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;

import java.util.List;

import static us.ihmc.behaviors.behaviorTree.BehaviorTreeTools.searchDFSFirstMatch;

public class DoorTraversalExecutor extends BehaviorTreeNodeExecutor<DoorTraversalState, DoorTraversalDefinition>
{
   private final ROS2Topic<KinematicsToolboxOutputStatus> kstOutputTopic;
   private ROS2LogRecord ros2LogRecord = null;
   private final KinematicsToolboxOutputStatus status = new KinematicsToolboxOutputStatus();

   public DoorTraversalExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new DoorTraversalState(id, rootNode.getState()), rootNode);

      kstOutputTopic = KinematicsStreamingToolboxModule.getOutputStatusTopic(robotModel.getSimpleRobotName());

      ROS2Publisher<KinematicsToolboxOutputStatus> publisher = ros2ControllerHelper.getROS2Node().createPublisher(kstOutputTopic);
      syncedRobot.addRobotConfigurationDataReceivedCallback(() ->
      {
         if (ros2LogRecord != null)
         {
            RobotConfigurationData rcd = syncedRobot.getLatestRobotConfigurationData();
            status.setSequenceId(rcd.getSequenceId());
            status.getDesiredJointAngles().set(rcd.getJointAngles());
            status.getDesiredRootPosition().set(rcd.getRootPosition());
            status.getDesiredRootOrientation().set(rcd.getRootOrientation());
            status.getDesiredJointVelocities().set(rcd.getJointVelocities());
            status.getDesiredRootLinearVelocity().set(rcd.getPelvisLinearVelocity());
            status.getDesiredRootAngularVelocity().set(rcd.getPelvisAngularVelocity());
            publisher.publish(status);
         }
      });
   }

   @Override
   public void update()
   {
      super.update();

      boolean executing = false;
      if (searchDFSFirstMatch(this, "Walk through push door") instanceof FootstepPlanActionExecutor walkThroughPushDoor)
         executing |= walkThroughPushDoor.getState().getIsExecuting();

      if (searchDFSFirstMatch(this, "Walk through") instanceof FootstepPlanActionExecutor walkThroughPullDoor)
         executing |= walkThroughPullDoor.getState().getIsExecuting();

      if (executing && ros2LogRecord == null)
      {
         ros2LogRecord = new ROS2LogRecord(robotModel.getSimpleRobotName(), List.of(kstOutputTopic), ROS2LogTimeSource.SYSTEM, ROS2LogSerialization.JSON);
         ros2LogRecord.start();
      }
      if (!executing && ros2LogRecord != null)
      {
         ROS2LogRecord ros2LogRecordLocal = ros2LogRecord;
         ThreadTools.startAThread(() ->
         {
            ros2LogRecordLocal.stop();
            ThreadTools.parkAtLeast(1.0);
            ros2LogRecordLocal.destroy();
         }, "StopLogging");
         ros2LogRecord = null;
      }
   }
}
