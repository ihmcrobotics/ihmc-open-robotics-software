package us.ihmc.humanoidBehaviors.behaviors.examples;

import controller_msgs.msg.dds.PointCloudWorldPacket;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.ros2.Ros2Node;

public class GetLidarScanExampleBehavior extends AbstractBehavior
{

   private int scanNumber = 0;
   private int NUMBER_OF_SCANS = 25;

   protected final ConcurrentListeningQueue<PointCloudWorldPacket> pointCloudQueue = new ConcurrentListeningQueue<PointCloudWorldPacket>(100);

   CommunicationBridge coactiveBehaviorsNetworkManager;

   public GetLidarScanExampleBehavior(String robotName, Ros2Node ros2Node)
   {
      super(robotName, ros2Node);
      //      coactiveBehaviorsNetworkManager = ros2Node; FIXME
      createSubscriber(PointCloudWorldPacket.class, ROS2Tools.getDefaultTopicNameGenerator(), pointCloudQueue::put);
   }

   @Override
   public void doControl()
   {
      if (pointCloudQueue.isNewPacketAvailable())
      {
         processPointCloud(HumanoidMessageTools.getDecayingWorldScan(pointCloudQueue.getLatestPacket()));
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
      publishTextToSpeech("Getting Lidar");
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
