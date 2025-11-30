package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage;
import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import behavior_msgs.msg.dds.HandPoseActionStateMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.SwapReference;
import us.ihmc.yoVariables.euclid.YoPose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Enabled logging behavior data synchronized with control data.
 *
 * TODO:
 *   hand pose desired, current
 */
public class ROS2BehaviorTreeYoRegistry
{
   protected final YoRegistry registry = new YoRegistry("BehaviorTreeExecutor");
   private final Notification notification = new Notification();
   private final SwapReference<BehaviorTreeStateMessage> subscription;

   private final ROS2BehaviorTreeSubscriptionNode subscriptionRootNode = new ROS2BehaviorTreeSubscriptionNode();
   private int depthFirstIndex = 0;
   private int executingActionIndex = 0;

   private final YoInteger persistentDetections = new YoInteger("persistentDetections", registry);
   private final YoInteger sceneObjects = new YoInteger("sceneObjects", registry);
   private final YoBoolean automaticExecution = new YoBoolean("automaticExecution", registry);
   private final YoInteger executionNextIndex = new YoInteger("executionNextIndex", registry);
   private final YoBoolean concurrencyEnabled = new YoBoolean("concurrencyEnabled", registry);
   private final YoInteger executingActions = new YoInteger("executingActions", registry);
   private final YoInteger failedActions = new YoInteger("failedActions", registry);
   private final YoInteger[] executingActionTypes = new YoInteger[5];
   private final YoPose3D[] currentHandPoses = new YoPose3D[5];
   private final YoPose3D[] goalHandPoses = new YoPose3D[5];

   public ROS2BehaviorTreeYoRegistry(ROS2Node ros2Node)
   {
      subscription = ROS2Tools.createSwapReferenceSubscription(ros2Node, AutonomyAPI.BEHAVIOR_TREE.getStatusTopic(), notification);

      for (int i = 0; i < executingActionTypes.length; i++)
      {
         executingActionTypes[i] = new YoInteger("executingAction" + i, registry);
         currentHandPoses[i] = new YoPose3D("currentHandPose" + i, registry);
         goalHandPoses[i] = new YoPose3D("goalHandPose" + i, registry);
      }
   }

   public void update()
   {
      if (notification.poll())
      {
         synchronized (subscription)
         {
            BehaviorTreeStateMessage state = subscription.getForThreadTwo();

            persistentDetections.set(state.getScene().getPersistentDetections().size());
            sceneObjects.set(state.getScene().getObjects().size());

            subscriptionRootNode.clear();
            depthFirstIndex = 0;
            executingActionIndex = 0;
            executingActions.set(0);
            for (int i = 0; i < executingActionTypes.length; i++)
            {
               executingActionTypes[i].set(-1);
               currentHandPoses[i].setToNaN();
               goalHandPoses[i].setToNaN();
            }
            failedActions.set(0);
            if (!state.getRootNodes().isEmpty())
            {
               BehaviorTreeRootNodeStateMessage root = state.getRootNodes().get(0);
               automaticExecution.set(root.getAutomaticExecution());
               concurrencyEnabled.set(root.getConcurrencyEnabled());
               executionNextIndex.set(root.getExecutionNextIndex());

               buildSubscriptionTree(state, subscriptionRootNode);

               processNode(subscriptionRootNode);
            }
         }
      }
   }

   private void processNode(ROS2BehaviorTreeSubscriptionNode node)
   {
      if (node.getLeafNodeStateMessage() != null)
      {
         if (node.getLeafNodeStateMessage().getIsExecuting())
         {
            executingActions.increment();
            executingActionTypes[executingActionIndex].set(node.getPackedType());
            ++executingActionIndex;

            if (node.getHandPoseActionStateMessage() != null)
            {
               HandPoseActionStateMessage handPose = node.getHandPoseActionStateMessage();

               currentHandPoses[node.getPackedType()].set(handPose.getState().getCurrentPose());

               Object<SE3TrajectoryPointMessage> trajectory = handPose.getState().getCommandedTrajectory();
               if (!trajectory.isEmpty())
               {
                  SE3TrajectoryPointMessage lastPose = trajectory.get(trajectory.size() - 1);
                  goalHandPoses[node.getPackedType()].getPosition().set(lastPose.getPosition());
                  goalHandPoses[node.getPackedType()].getOrientation().set(lastPose.getOrientation());
               }
            }
         }

         if (node.getLeafNodeStateMessage().getFailed())
         {
            failedActions.increment();
         }
      }

      for (ROS2BehaviorTreeSubscriptionNode child : node.getChildren())
      {
         processNode(child);
      }
   }

   private void buildSubscriptionTree(BehaviorTreeStateMessage behaviorTreeStateMessage, ROS2BehaviorTreeSubscriptionNode subscriptionNode)
   {
      subscriptionNode.setSequenceId(behaviorTreeStateMessage.getSequenceId());
      subscriptionNode.setPackedType(behaviorTreeStateMessage.getBehaviorTreeTypes().get(depthFirstIndex));

      ROS2BehaviorTreeMessageTools.packSubscriptionNode(behaviorTreeStateMessage.getBehaviorTreeTypes().get(depthFirstIndex),
                                                        (int) behaviorTreeStateMessage.getBehaviorTreeIndices().get(depthFirstIndex),
                                                        behaviorTreeStateMessage,
                                                        subscriptionNode);

      for (int i = 0; i < subscriptionNode.getBehaviorTreeNodeDefinitionMessage().getNumberOfChildren(); i++)
      {
         ++depthFirstIndex;
         ROS2BehaviorTreeSubscriptionNode subscriptionTreeChildNode = new ROS2BehaviorTreeSubscriptionNode();
         buildSubscriptionTree(behaviorTreeStateMessage, subscriptionTreeChildNode);
         subscriptionNode.getChildren().add(subscriptionTreeChildNode);
      }
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
