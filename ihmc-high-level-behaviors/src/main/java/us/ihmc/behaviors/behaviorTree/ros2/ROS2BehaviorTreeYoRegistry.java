package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.BehaviorTreeYoDataMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.SwapReference;
import us.ihmc.yoVariables.euclid.YoPose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * Injects behavior tree data into YoVariables so it can be logged.
 */
public class ROS2BehaviorTreeYoRegistry
{
   private final FullHumanoidRobotModel fullRobotModel;
   private final YoRegistry registry = new YoRegistry("BehaviorTreeExecutor");
   private final Notification notification = new Notification();
   private final SwapReference<BehaviorTreeYoDataMessage> subscription;

   private final YoLong messagesReceived = new YoLong("messagesReceived", registry);
   private final YoInteger persistentDetections = new YoInteger("persistentDetections", registry);
   private final YoInteger sceneObjects = new YoInteger("sceneObjects", registry);
   private final YoPose3D[] sceneObjectPoses = new YoPose3D[3];
   private final YoBoolean automaticExecution = new YoBoolean("automaticExecution", registry);
   private final YoInteger executionNextIndex = new YoInteger("executionNextIndex", registry);
   private final YoBoolean concurrencyEnabled = new YoBoolean("concurrencyEnabled", registry);
   private final YoInteger executingActions = new YoInteger("executingActions", registry);
   private final YoInteger failedActions = new YoInteger("failedActions", registry);
   private final YoInteger[] executingActionTypes = new YoInteger[5];
   private final YoInteger[] executingActionIDs = new YoInteger[5];
   private final YoDouble[] elapsedExecutionTimes = new YoDouble[5];
   private final RigidBodyTransform transform = new RigidBodyTransform();
   private final YoPose3D estimatorChestPose;
   private final SideDependentList<YoPose3D> estimatorHandPoses = new SideDependentList<>();
   private final SideDependentList<YoPose3D> currentHandPoses = new SideDependentList<>();
   private final SideDependentList<YoPose3D> goalHandPoses = new SideDependentList<>();

   public ROS2BehaviorTreeYoRegistry(ROS2Node ros2Node, FullHumanoidRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;

      subscription = ROS2Tools.createSwapReferenceSubscription(ros2Node, AutonomyAPI.BEHAVIOR_YO_DATA, notification);

      for (int i = 0; i < sceneObjectPoses.length; i++)
         sceneObjectPoses[i] = new YoPose3D("sceneObject" + i, registry);

      for (int i = 0; i < executingActionTypes.length; i++)
      {
         executingActionTypes[i] = new YoInteger("executingActionType" + i, registry);
         executingActionIDs[i] = new YoInteger("executingActionID" + i, registry);
         elapsedExecutionTimes[i] = new YoDouble("elapsedExecutionTime" + i, registry);
      }

      estimatorChestPose = new YoPose3D("estimatorChestPose", registry);
      for (RobotSide side : RobotSide.values)
      {
         estimatorHandPoses.put(side, new YoPose3D("estimatorHandPose" + side.getPascalCaseName(), registry));
         currentHandPoses.put(side, new YoPose3D("currentHandPose" + side.getPascalCaseName(), registry));
         goalHandPoses.put(side, new YoPose3D("goalHandPose" + side.getPascalCaseName(), registry));
      }
   }

   public void update()
   {
      try // Make sure exceptions don't crash controller
      {
         fullRobotModel.getChest().getParentJoint().getFrameAfterJoint().getTransformToDesiredFrame(transform, ReferenceFrame.getWorldFrame());
         estimatorChestPose.set(transform);
         for (RobotSide side : RobotSide.values)
         {
            if (fullRobotModel.getHandControlFrame(side) != null)
            {
               fullRobotModel.getHandControlFrame(side).getTransformToDesiredFrame(transform, ReferenceFrame.getWorldFrame());
               estimatorHandPoses.get(side).set(transform);
            }
         }

         if (notification.poll())
         {
            synchronized (subscription)
            {
               BehaviorTreeYoDataMessage data = subscription.getForThreadTwo();

               messagesReceived.increment();
               persistentDetections.set(data.getNumberOfPersistentDetections());
               sceneObjects.set(data.getNumberOfSceneObjects());

               for (int i = 0; i < sceneObjectPoses.length; i++)
                  sceneObjectPoses[i].set(data.getSceneObjectPose()[i]);

               automaticExecution.set(data.getAutomaticExecution());
               executionNextIndex.set(data.getExecutionNextIndex());
               concurrencyEnabled.set(data.getConcurrencyEnabled());
               executingActions.set(data.getNumberOfExecutingActions());
               failedActions.set(data.getNumberOfFailedActions());

               for (int i = 0; i < executingActionTypes.length; i++)
               {
                  executingActionTypes[i].set(data.getExecutingActionType()[i]);
                  executingActionIDs[i].set(data.getExecutingActionId()[i]);
                  elapsedExecutionTimes[i].set(data.getElapsedExecutionTime()[i]);
               }

               for (RobotSide side : RobotSide.values)
               {
                  currentHandPoses.get(side).set(data.getCurrentHandPose()[side.ordinal()]);
                  goalHandPoses.get(side).set(data.getGoalHandPose()[side.ordinal()]);
               }
            }
         }
      }
      catch (Throwable t)
      {
         t.printStackTrace();
      }
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
