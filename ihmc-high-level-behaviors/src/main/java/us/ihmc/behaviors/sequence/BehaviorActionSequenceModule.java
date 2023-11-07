package us.ihmc.behaviors.sequence;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.Throttler;

/**
 * The entry point and initial setup of the behavior sequence on-robot process.
 */
public class BehaviorActionSequenceModule
{
   private volatile boolean running = true;
   private final ROS2Node ros2Node;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SceneGraph sceneGraph;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final Throttler throttler = new Throttler();
   private final double PERIOD = Conversions.hertzToSeconds(30.0);
   private final BehaviorActionSequence sequence;
   private final Notification stopped = new Notification();

   public BehaviorActionSequenceModule(DRCRobotModel robotModel)
   {
      ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "behavior_action_sequence");
      ros2ControllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);

      ROS2SyncedRobotModel syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2ControllerHelper.getROS2NodeInterface());

      sceneGraph = new ROS2SceneGraph(ros2ControllerHelper);
      referenceFrameLibrary = new ReferenceFrameLibrary();
      referenceFrameLibrary.addAll(syncedRobot.getReferenceFrames().getCommonReferenceFrames());
      referenceFrameLibrary.addDynamicCollection(sceneGraph.asNewDynamicReferenceFrameCollection());

      sequence = new BehaviorActionSequence(robotModel, syncedRobot, ros2ControllerHelper, referenceFrameLibrary);

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "Shutdown"));
      ThreadTools.startAThread(this::actionThread, "ActionThread");
   }

   private void actionThread()
   {
      while (running)
      {
         throttler.waitAndRun(PERIOD);

         sceneGraph.updateSubscription();

         sequence.update();
      }

      sceneGraph.destroy();
      sequence.destroy();
      ros2Node.destroy();
      stopped.set();
   }

   private void destroy()
   {
      running = false;
      stopped.blockingPoll();
   }
}
