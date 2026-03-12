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
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;

import java.util.List;

import static us.ihmc.behaviors.behaviorTree.BehaviorTreeTools.searchDFSFirstMatch;

public class DoorTraversalExecutor extends BehaviorTreeNodeExecutor<DoorTraversalState, DoorTraversalDefinition>
{
   private final ROS2Topic<KinematicsToolboxOutputStatus> kstOutputTopic;
   private ROS2LogRecord ros2LogRecord = null;
   private final KinematicsToolboxOutputStatus status = new KinematicsToolboxOutputStatus();
   private final RigidBodyTransform initialPelvisPose = new RigidBodyTransform();
   private final RigidBodyTransform relativePelvisPose = new RigidBodyTransform();

   public DoorTraversalExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new DoorTraversalState(id, rootNode.getState()), rootNode);

      kstOutputTopic = KinematicsStreamingToolboxModule.getOutputStatusTopic(robotModel.getSimpleRobotName());

      List<Integer> nonFingerIndices = FullRobotModelUtils.getAllJointsExcludingHandsIndices(syncedRobot.getFullRobotModel());
      float[] nonFingerValues = new float[nonFingerIndices.size()];

      ROS2Publisher<KinematicsToolboxOutputStatus> publisher = ros2ControllerHelper.getROS2Node().createPublisher(kstOutputTopic);
      syncedRobot.addRobotConfigurationDataReceivedCallback(() ->
      {
         if (ros2LogRecord != null)
         {
            RobotConfigurationData rcd = syncedRobot.getLatestRobotConfigurationData();
            status.setSequenceId(rcd.getSequenceId());

            relativePelvisPose.set(rcd.getRootOrientation(), rcd.getRootPosition());
            initialPelvisPose.inverseTransform(relativePelvisPose);
            status.getDesiredRootPosition().set(relativePelvisPose.getTranslation());
            status.getDesiredRootOrientation().set(relativePelvisPose.getRotation());
            status.getDesiredRootLinearVelocity().set(rcd.getPelvisLinearVelocity());
            status.getDesiredRootAngularVelocity().set(rcd.getPelvisAngularVelocity());

            for (int i = 0; i < nonFingerValues.length; i++)
               nonFingerValues[i] = rcd.getJointAngles().get(nonFingerIndices.get(i));
            status.getDesiredJointAngles().clear();
            status.getDesiredJointAngles().add(nonFingerValues);
            for (int i = 0; i < nonFingerValues.length; i++)
               nonFingerValues[i] = rcd.getJointVelocities().get(nonFingerIndices.get(i));
            status.getDesiredJointVelocities().clear();
            status.getDesiredJointVelocities().add(nonFingerValues);

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
         initialPelvisPose.set(syncedRobot.getReferenceFrames().getPelvisFrame().getTransformToRoot());
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
