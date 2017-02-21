package us.ihmc.humanoidBehaviors.behaviors.examples;

import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;

public class GetLidarScanExampleBehavior extends AbstractBehavior
{

   private int scanNumber = 0;
   private int NUMBER_OF_SCANS =25;

   protected final ConcurrentListeningQueue<PointCloudWorldPacket> pointCloudQueue = new ConcurrentListeningQueue<PointCloudWorldPacket>(100);

   CommunicationBridge coactiveBehaviorsNetworkManager;

   public GetLidarScanExampleBehavior(CommunicationBridge communicationBridge)
   {
      super(communicationBridge);
      coactiveBehaviorsNetworkManager = communicationBridge;
      this.attachNetworkListeningQueue(pointCloudQueue, PointCloudWorldPacket.class);
   }

   @Override
   public void doControl()
   {
      if (pointCloudQueue.isNewPacketAvailable())
      {
         processPointCloud(pointCloudQueue.getLatestPacket().getDecayingWorldScan());
      }
   }

   protected void processPointCloud(Point3D32[] points)
   {
      scanNumber++;

      coactiveBehaviorsNetworkManager.sendToUI("PointCloudRecieved", scanNumber);
   }

   @Override
   public boolean isDone()
   {
      return scanNumber >= NUMBER_OF_SCANS;
   }

   @Override
   public void onBehaviorEntered()
   {
      //reset necessary values so this behavior can run again properly
      scanNumber = 0;
      TextToSpeechPacket p1 = new TextToSpeechPacket("Getting Lidar");
      sendPacket(p1);
      //let the UI know this specific behavior has started
      coactiveBehaviorsNetworkManager.sendToUI("GetLidarScanExampleBehavior", 1);
   }

   @Override
   public void onBehaviorExited()
   {
      //let the UI know this specific behavior has ended
      coactiveBehaviorsNetworkManager.sendToUI("GetLidarScanExampleBehavior", 0);
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
