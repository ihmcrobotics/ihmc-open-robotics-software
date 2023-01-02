package us.ihmc.behaviors.sequence;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.tools.thread.Throttler;

public class BehaviorActionSequenceModule
{
   private volatile boolean running = true;
   private final Throttler throttler = new Throttler();
   private final double PERIOD = Conversions.hertzToSeconds(30.0);
   private final BehaviorActionSequence sequence;

   public BehaviorActionSequenceModule(DRCRobotModel robotModel, ReferenceFrameLibrary referenceFrameLibrary)
   {
      ROS2ControllerHelper ros2 = new ROS2ControllerHelper(PubSubImplementation.FAST_RTPS, "behavior_action_sequence", robotModel);
      sequence = new BehaviorActionSequence(robotModel, ros2, referenceFrameLibrary);

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "Shutdown"));
      ThreadTools.startAThread(this::actionThread, "ActionThread");
   }

   private void actionThread()
   {
      while (running)
      {
         throttler.waitAndRun(PERIOD);

         // Update sequences edited by user

         sequence.update();
      }
   }

   private void destroy()
   {
      running = false;
   }
}
