package us.ihmc.behaviors.door;

import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.StaticRelativeSceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorSceneNodeDefinitions;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class DoorTraversalExecutor extends BehaviorTreeNodeExecutor<DoorTraversalState, DoorTraversalDefinition>
{
   private final DoorTraversalState state;
   private final DoorTraversalDefinition definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final SceneGraph sceneGraph;

   private final SideDependentList<RevoluteJoint> x1KnuckleJoints = new SideDependentList<>();
   private final SideDependentList<RevoluteJoint> x2KnuckleJoints = new SideDependentList<>();

   private final transient StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();
   private boolean waitForPullScrewToFinish = false;
   private boolean waitForGraspToFinish = false;

   private transient final FramePose3D pushDoorPanelPose = new FramePose3D();

   public DoorTraversalExecutor(long id,
                                CRDTInfo crdtInfo,
                                WorkspaceResourceDirectory saveFileDirectory,
                                ROS2ControllerHelper ros2ControllerHelper,
                                ROS2SyncedRobotModel syncedRobot,
                                SceneGraph sceneGraph)
   {
      super(new DoorTraversalState(id, crdtInfo, saveFileDirectory));

      state = getState();
      definition = getDefinition();

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.sceneGraph = sceneGraph;

      for (RobotSide side : RobotSide.values)
      {
         if (syncedRobot.getRobotModel().getRobotVersion().hasSakeGripperJoints(side))
         {
            x1KnuckleJoints.put(side, (RevoluteJoint) syncedRobot.getFullRobotModel().getHand(side).getChildrenJoints().get(0));
            x2KnuckleJoints.put(side, (RevoluteJoint) syncedRobot.getFullRobotModel().getHand(side).getChildrenJoints().get(1));
         }
      }
   }

   @Override
   public void tick()
   {
      super.tick();

      // TODO: Tick children
   }

   @Override
   public void update()
   {
      super.update();

      updateActionSubtree(this);

      state.getDoorHingeJointAngle().setValue(Double.NaN);

      // TODO: PUSH_DOOR_FRAME_NAME renamed to RIGHT_DOOR_FRAME?
      SceneNode pushDoorFrameNode = sceneGraph.getNamesToNodesMap().get(DoorSceneNodeDefinitions.PUSH_DOOR_FRAME_NAME);
      SceneNode pushDoorPanelNode = sceneGraph.getNamesToNodesMap().get(DoorSceneNodeDefinitions.RIGHT_DOOR_PANEL_NAME);

      DetectableSceneNode yoloDoorHandleNode = (DetectableSceneNode) sceneGraph.getNamesToNodesMap().get("YOLO door lever");
      StaticRelativeSceneNode staticHandleClosedDoor = (StaticRelativeSceneNode) sceneGraph.getNamesToNodesMap().get("doorStaticHandle");

      if (pushDoorFrameNode != null && pushDoorPanelNode != null)
      {
         pushDoorPanelPose.setFromReferenceFrame(pushDoorPanelNode.getNodeFrame());
         pushDoorPanelPose.changeFrame(pushDoorFrameNode.getNodeFrame());
         state.getDoorHingeJointAngle().setValue(pushDoorPanelPose.getOrientation().getYaw());
      }

      if (state.getStabilizeDetectionAction() != null && state.getStabilizeDetectionAction().getIsExecuting())
      {
         if (staticHandleClosedDoor != null)
         {
            staticHandleClosedDoor.clearOffset();
            staticHandleClosedDoor.freeze();
         }
      }

      if (state.arePullRetryNodesPresent())
      {
         // Here we are preventing the below logic from triggering more than once at a time
         if (!state.getPostPullDoorEvaluationAction().getIsExecuting())
         {
            waitForPullScrewToFinish = false;
         }
         if (!waitForPullScrewToFinish && state.getPostPullDoorEvaluationAction().getIsExecuting())
         {
            if (yoloDoorHandleNode != null)
            {
               double openedDoorHandleDistanceFromStart = getDefinition().getOpenedDoorHandleDistanceFromStart().getValue();
               double distanceHandleFromStart = yoloDoorHandleNode.getNodeToParentFrameTransform().getTranslation()
                                                                  .differenceNorm(staticHandleClosedDoor.getNodeToParentFrameTransform().getTranslation());
               state.getDoorHandleDistanceFromStart().setValue(distanceHandleFromStart);
               if (state.getDoorHandleDistanceFromStart().getValue() < openedDoorHandleDistanceFromStart)
               {
                  state.getLogger().info("""
                                         Retrying pull door. Distance door handle from start %.2f / %.2f [m].
                                         Stopping all trajectories.
                                         Going back to %s.
                                         """.formatted(state.getDoorHandleDistanceFromStart().getValue(), openedDoorHandleDistanceFromStart, state.getWaitToOpenRightHandAction().getDefinition().getName()));
                  ros2ControllerHelper.publishToController(stopAllTrajectoryMessage);
                  waitForPullScrewToFinish = true;
                  state.getActionSequence().setExecutionNextIndex(state.getWaitToOpenRightHandAction().getActionIndex());
               }
            }
//            double knuckle1Q = x1KnuckleJoints.get(RobotSide.RIGHT).getQ();
//            double knuckle2Q = x2KnuckleJoints.get(RobotSide.RIGHT).getQ();
//            double handOpenAngle = SakeHandParameters.knuckleJointAnglesToHandOpenAngle(knuckle1Q, knuckle2Q);
//
//            double lostGraspDetectionHandOpenAngle = getDefinition().getLostGraspDetectionHandOpenAngle().getValue();
//            if (handOpenAngle < lostGraspDetectionHandOpenAngle)
//            {
//               state.getLogger().info("""
//                                      Retrying pull door. Hand open angle %.2f%s / %.2f%s degrees.
//                                      Stopping all trajectories.
//                                      Going back to %s.
//                                      """.formatted(Math.toDegrees(knuckle1Q),
//                                                    EuclidCoreMissingTools.DEGREE_SYMBOL,
//                                                    Math.toDegrees(lostGraspDetectionHandOpenAngle),
//                                                    EuclidCoreMissingTools.DEGREE_SYMBOL,
//                                                    state.getWaitToOpenRightHandAction().getDefinition().getName()));
//               ros2ControllerHelper.publishToController(stopAllTrajectoryMessage);
//               waitForPullScrewToFinish = true;
//               state.getActionSequence().setExecutionNextIndex(state.getWaitToOpenRightHandAction().getActionIndex());
//               state.getPullScrewPrimitiveAction().setIsExecuting(false);
//            }
         }
         if (!state.getPostGraspEvaluationAction().getIsExecuting())
         {
            waitForGraspToFinish = false;
         }
         if (!waitForGraspToFinish && state.getPostGraspEvaluationAction().getIsExecuting())
         {
            if (staticHandleClosedDoor != null)
            {
               double handToHandleDistance = syncedRobot.getFullRobotModel().getHandControlFrame(RobotSide.RIGHT).
                     getTransformToDesiredFrame(staticHandleClosedDoor.getNodeFrame()).getTranslation().norm();
               if (handToHandleDistance > 0.19)
               {
                  state.getLogger().info("""
                                      Retrying reach door handle. Distance hand to door handle %.2f / %.2f [m].
                                      Stopping all trajectories.
                                      Going back to %s.
                                      """.formatted(handToHandleDistance, 0.19, state.getWaitToOpenRightHandAction().getDefinition().getName()));
                  ros2ControllerHelper.publishToController(stopAllTrajectoryMessage);
                  waitForGraspToFinish = true;
                  state.getActionSequence().setExecutionNextIndex(state.getWaitToOpenRightHandAction().getActionIndex());
               }
            }
         }
      }
   }

   public void updateActionSubtree(BehaviorTreeNodeExecutor<?, ?> node)
   {
      for (BehaviorTreeNodeExecutor<?, ?> child : node.getChildren())
      {
         if (child instanceof ActionNodeExecutor<?, ?> actionNode)
         {

         }
         else
         {
            updateActionSubtree(child);
         }
      }
   }
}
