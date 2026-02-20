package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.BehaviorTreeYoDataMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.*;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.behaviors.behaviorTree.action.actions.HandPoseActionExecutor;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneExecutor;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.crdt.CRDTStatusSE3Trajectory;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicatorMap;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.sensors.ImageSensor;

/**
 * Top level class for the robot's behavior tree.
 */
public class ROS2BehaviorTreeExecutor extends BehaviorTreeExecutor
{
   private final ROS2BehaviorTree<BehaviorTreeNodeExecutor<?, ?>> ros2BehaviorTree;

   private final BehaviorTreeYoDataMessage yoDataMessage = new BehaviorTreeYoDataMessage();
   private final ROS2Publisher<BehaviorTreeYoDataMessage> yoDataPublisher;

   public ROS2BehaviorTreeExecutor(ROS2ControllerHelper ros2ControllerHelper,
                                   ROS2SyncedRobotModel syncedRobot,
                                   ImageSensor imageSensor,
                                   YOLOv8DetectionExecutor yolo,
                                   IsaacROSFoundationPoseCommunicatorMap foundationPose,
                                   TerrainMapData terrainMapData,
                                   ROS2PeerClockOffsetEstimator peerClockEstimator)
   {
      super(syncedRobot, peerClockEstimator, ros2ControllerHelper, imageSensor, yolo, foundationPose, terrainMapData);

      ros2BehaviorTree = new ROS2BehaviorTree<>((BehaviorTree) this, ros2ControllerHelper);

      yoDataPublisher = ros2ControllerHelper.getROS2Node().createPublisher(AutonomyAPI.BEHAVIOR_YO_DATA);
   }

   /** Expected to be called at the {@link ROS2BehaviorTree#SYNC_FREQUENCY} */
   public void update()
   {
      BehaviorTreeExecutor tree = (BehaviorTreeExecutor) ros2BehaviorTree.getBehaviorTree();
      BehaviorTreeSceneExecutor scene = tree.getScene();

      yoDataMessage.setNumberOfPersistentDetections((byte) scene.getPersistentDetections().size());
      yoDataMessage.setNumberOfSceneObjects((byte) scene.getObjects().size());

      for (int i = 0; i < yoDataMessage.getSceneObjectPose().length; i++)
      {
         if (scene.getObjects().size() > i)
            yoDataMessage.getSceneObjectPose()[i].set(scene.getObjects().get(i).getTransformToWorld());
         else
            yoDataMessage.getSceneObjectPose()[i].setToNaN();
      }

      BehaviorTreeRootNodeExecutor rootNode = tree.getRootNode();
      if (rootNode != null)
      {
         yoDataMessage.setAutomaticExecution(rootNode.getState().getAutomaticExecution());
         yoDataMessage.setExecutionNextIndex((byte) rootNode.getState().getExecutionNextIndex());
         yoDataMessage.setConcurrencyEnabled(rootNode.getState().getConcurrencyEnabled());
         yoDataMessage.setNumberOfExecutingActions((byte) rootNode.getCurrentlyExecutingLeaves().size());
         yoDataMessage.setNumberOfFailedActions((byte) rootNode.getFailedLeaves().size());
         
         for (int i = 0; i < yoDataMessage.getExecutingActionType().length; i++)
         {
            if (rootNode.getCurrentlyExecutingLeaves().size() > i)
            {
               yoDataMessage.getExecutingActionType()[i]
                     = BehaviorTreeDefinitionRegistry.getMessageByte(rootNode.getCurrentlyExecutingLeaves().get(i).getDefinition().getClass());
               yoDataMessage.getExecutingActionId()[i] = (short) rootNode.getCurrentlyExecutingLeaves().get(i).getState().getID();
               if (rootNode.getCurrentlyExecutingLeaves().get(i).getState() instanceof ActionNodeState<?> actionState)
                  yoDataMessage.getElapsedExecutionTime()[i] = (float) actionState.getElapsedExecutionTime();
            }
            else
            {
               yoDataMessage.getExecutingActionType()[i] = -1;
               yoDataMessage.getExecutingActionId()[i] = -1;
               yoDataMessage.getElapsedExecutionTime()[i] = Float.NaN;
            }
         }

         SideDependentList<HandPoseActionExecutor> lastHandPoseActions = new SideDependentList<>();
         for (LeafNodeExecutor<?, ?> leaf : rootNode.getCurrentlyExecutingLeaves())
            if (leaf instanceof HandPoseActionExecutor handPoseAction)
               lastHandPoseActions.put(handPoseAction.getDefinition().getSide(), handPoseAction);
         for (RobotSide side : lastHandPoseActions.sides())
         {
            HandPoseActionExecutor action = lastHandPoseActions.get(side);
            yoDataMessage.getCurrentHandPose()[side.ordinal()].set(action.getState().getCurrentPose().getValueReadOnly());
            CRDTStatusSE3Trajectory commandedTrajectory = action.getState().getCommandedTrajectory();
            if (!commandedTrajectory.isEmpty())
               yoDataMessage.getGoalHandPose()[side.ordinal()].set(commandedTrajectory.getLastValueReadOnly().getOrientation(),
                                                                   commandedTrajectory.getLastValueReadOnly().getPosition());
         }
      }
      
      yoDataPublisher.publish(yoDataMessage);

      ros2BehaviorTree.updatePublication();
      ros2BehaviorTree.updateSubscription();

      // TODO: Consider updating this at a higher rate than the comms
      super.update();
   }

   public void destroy()
   {
      ros2BehaviorTree.destroy();

      super.destroy();
   }
}
