package us.ihmc.humanoidBehaviors.behaviors.examples;

import perception_msgs.msg.dds.VideoPacket;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.ros2.ROS2Node;

public class GetVideoPacketExampleBehavior extends AbstractBehavior
{

   private int frameNumber = 0;
   private int NUMBER_OF_FRAMES = 25;

   private final ConcurrentListeningQueue<VideoPacket> videoPacketQueue = new ConcurrentListeningQueue<VideoPacket>(20);

   CommunicationBridge coactiveBehaviorsNetworkManager;

   public GetVideoPacketExampleBehavior(String robotName, ROS2Node ros2Node)
   {
      super(robotName, ros2Node);
//      coactiveBehaviorsNetworkManager = ros2Node; FIXME
      createSubscriber(VideoPacket.class, ROS2Tools.IHMC_ROOT, videoPacketQueue::put);
   }

   @Override
   public void doControl()
   {
      if (videoPacketQueue.isNewPacketAvailable())
      {
         processVideoPacket(videoPacketQueue.getLatestPacket());
      }
   }

   protected void processVideoPacket(VideoPacket videoPacket)
   {
      frameNumber++;
      System.out.println("GetVideoPacketExampleBehavior: got video frame "+frameNumber);
   }

   @Override
   public boolean isDone()
   {
      return frameNumber >= NUMBER_OF_FRAMES;
   }

   @Override
   public void onBehaviorEntered()
   {
      //reset necessary values so this behavior can run again properly
      frameNumber = 0;
      publishTextToSpeech("Getting Video Packets");
      //let the UI know this specific behavior has started
      coactiveBehaviorsNetworkManager.sendToUI("GetVideoPacketExampleBehavior", 1);
   }

   @Override
   public void onBehaviorExited()
   {
      //let the UI know this specific behavior has ended
      coactiveBehaviorsNetworkManager.sendToUI("GetVideoPacketExampleBehavior", 0);
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }
}
