package us.ihmc.behaviors.sequence;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.perception.sceneGraph.PredefinedSceneNodeLibrary;
import us.ihmc.perception.sceneGraph.ROS2DetectableSceneNodesSubscription;
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
   private final ROS2DetectableSceneNodesSubscription detectableSceneNodesSubscription;
   private final Throttler throttler = new Throttler();
   private final double PERIOD = Conversions.hertzToSeconds(30.0);
   private final BehaviorActionSequence sequence;
   private final Notification stopped = new Notification();

   public BehaviorActionSequenceModule(DRCRobotModel robotModel, PredefinedSceneNodeLibrary predefinedSceneNodeLibrary)
   {
      ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "behavior_action_sequence");
      ros2ControllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);

      detectableSceneNodesSubscription = new ROS2DetectableSceneNodesSubscription(predefinedSceneNodeLibrary.getDetectableSceneNodes(), ros2ControllerHelper,
                                                                                  ROS2IOTopicQualifier.STATUS);

      ReferenceFrameLibrary referenceFrameLibrary = new ReferenceFrameLibrary();
      referenceFrameLibrary.addAll(predefinedSceneNodeLibrary.getReferenceFrameSuppliers());

      sequence = new BehaviorActionSequence(robotModel, ros2ControllerHelper, referenceFrameLibrary);

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "Shutdown"));
      ThreadTools.startAThread(this::actionThread, "ActionThread");
   }

   private void actionThread()
   {
      while (running)
      {
         throttler.waitAndRun(PERIOD);

         detectableSceneNodesSubscription.update();
         sequence.update();
      }

      sequence.destroy();
      detectableSceneNodesSubscription.destroy();
      ros2Node.destroy();
      stopped.set();
   }

   private void destroy()
   {
      running = false;
      stopped.blockingPoll();
   }
}
