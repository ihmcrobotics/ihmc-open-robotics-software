package us.ihmc.humanoidBehaviors.behaviors.examples;

import javax.vecmath.Point3f;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;

public class GetLidarScanExampleBehavior extends AbstractBehavior
{

   private int scanNumber = 0;
   private int NUMBER_OF_SCANS = 5;

   protected final ConcurrentListeningQueue<PointCloudWorldPacket> pointCloudQueue = new ConcurrentListeningQueue<PointCloudWorldPacket>();

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

   protected void processPointCloud(Point3f[] points)
   {
      scanNumber++;

      System.out.println("got scan of size "+points.length);
      coactiveBehaviorsNetworkManager.sendToUI("PointCloudRecieved", scanNumber);
   }

   @Override
   public boolean isDone()
   {
      return scanNumber >= NUMBER_OF_SCANS;
   }

   @Override
   public void initialize()
   {
      super.initialize();
      //reset necessary values so this behavior can run again properly
      scanNumber = 0;
      //let the UI know this specific behavior has started
      coactiveBehaviorsNetworkManager.sendToUI("GetLidarScanExampleBehavior", 1);
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      super.doPostBehaviorCleanup();

      //let the UI know this specific behavior has ended
      coactiveBehaviorsNetworkManager.sendToUI("GetLidarScanExampleBehavior", 0);
   }
}
