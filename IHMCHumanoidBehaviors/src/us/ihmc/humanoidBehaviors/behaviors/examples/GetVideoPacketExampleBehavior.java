package us.ihmc.humanoidBehaviors.behaviors.examples;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.CoactiveBehaviorsNetworkManager;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;

public class GetVideoPacketExampleBehavior extends AbstractBehavior
{

   private int frameNumber = 0;
   private int NUMBER_OF_FRAMES = 10;

   private final ConcurrentListeningQueue<VideoPacket> videoPacketQueue = new ConcurrentListeningQueue<VideoPacket>();

   CoactiveBehaviorsNetworkManager coactiveBehaviorsNetworkManager;

   public GetVideoPacketExampleBehavior(BehaviorCommunicationBridge communicationBridge)
   {
      super(communicationBridge);
      coactiveBehaviorsNetworkManager = new CoactiveBehaviorsNetworkManager(communicationBridge, communicationBridge);
      this.attachNetworkProcessorListeningQueue(videoPacketQueue, VideoPacket.class);
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

      //example of forwarding a packet to the UI
      coactiveBehaviorsNetworkManager.sendToUI("videoPacketRecieved", frameNumber);
   }

   @Override
   public boolean isDone()
   {
      return frameNumber >= NUMBER_OF_FRAMES;
   }

   @Override
   public void initialize()
   {
      super.initialize();
      //reset necessary values so this behavior can run again properly
      frameNumber = 0;
      //let the UI know this specific behavior has started
      coactiveBehaviorsNetworkManager.sendToUI("GetVideoPacketExampleBehavior", 1);
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      super.doPostBehaviorCleanup();

      //let the UI know this specific behavior has ended
      coactiveBehaviorsNetworkManager.sendToUI("GetVideoPacketExampleBehavior", 0);
   }
}
