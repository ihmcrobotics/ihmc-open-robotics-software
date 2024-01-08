package us.ihmc.behaviors.sequence;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTreeExecutor;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.Throttler;

import java.util.Collections;

/**
 * The entry point and initial setup of the behavior sequence on-robot process.
 */
public class BehaviorTreeModule
{
   private volatile boolean running = true;
   private final ROS2Node ros2Node;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2SceneGraph sceneGraph;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final Throttler throttler = new Throttler();
   private final double PERIOD = Conversions.hertzToSeconds(30.0);
   private final ROS2BehaviorTreeExecutor behaviorTreeExecutor;
   private final Notification stopped = new Notification();

   public BehaviorTreeModule(DRCRobotModel robotModel)
   {
      ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "behavior_tree");
      ros2ControllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2ControllerHelper.getROS2NodeInterface());

      sceneGraph = new ROS2SceneGraph(ros2ControllerHelper);
      referenceFrameLibrary = new ReferenceFrameLibrary();
      referenceFrameLibrary.addAll(Collections.singleton(ReferenceFrame.getWorldFrame()));
      referenceFrameLibrary.addAll(syncedRobot.getReferenceFrames().getCommonReferenceFrames());
      referenceFrameLibrary.addDynamicCollection(sceneGraph.asNewDynamicReferenceFrameCollection());

      behaviorTreeExecutor = new ROS2BehaviorTreeExecutor(ros2ControllerHelper,
                                                          robotModel,
                                                          syncedRobot,
                                                          referenceFrameLibrary);

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "Shutdown"));
      ThreadTools.startAThread(this::actionThread, "ActionThread");
   }

   private void actionThread()
   {
      while (running)
      {
         throttler.waitAndRun(PERIOD);

         syncedRobot.update();

         sceneGraph.updateSubscription();

         behaviorTreeExecutor.update();
      }

      sceneGraph.destroy();
      behaviorTreeExecutor.destroy();
      ros2Node.destroy();
      stopped.set();
   }

   private void destroy()
   {
      running = false;
      stopped.blockingPoll();
   }
}
