package us.ihmc.darpaRoboticsChallenge.networkProcessor.modules;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;

import sensor_msgs.PointCloud2;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.sensing.MultisenseMocapExperimentPacket;
import us.ihmc.communication.packets.sensing.MultisenseTest;
import us.ihmc.communication.packets.sensing.MultisenseTest.MultisenseFrameName;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

public class MultisensePointCloudReceiver extends RosPointCloudSubscriber 
{
   private final PacketCommunicator packetCommunicator;
   private final MultisenseTest testInfo;
   private final MultisenseFrameName frame;
   private final AtomicReference<RigidBodyTransform> lidarOriginInWorldAtomicReference = new AtomicReference<RigidBodyTransform>(new RigidBodyTransform());
   private final RigidBodyTransform transformFromMocapCentroidToLidarBaseFrame;
   
   public MultisensePointCloudReceiver(PacketCommunicator packetCommunicator, MultisenseTest testInfo)
   {
      this.packetCommunicator = packetCommunicator;
      this.testInfo = testInfo;
      this.frame = testInfo.getFrame();
      transformFromMocapCentroidToLidarBaseFrame = frame.getTransformFromMocapCentroidToLidarFrame();
      lidarOriginInWorldAtomicReference.set(new RigidBodyTransform(transformFromMocapCentroidToLidarBaseFrame));
   }
   
   public void setWorldTransform(RigidBodyTransform lidarOriginInMocap)
   {
      RigidBodyTransform lidarOriginInWorld = new RigidBodyTransform(lidarOriginInMocap);
      lidarOriginInWorld.multiply(transformFromMocapCentroidToLidarBaseFrame);
      lidarOriginInWorldAtomicReference.set(lidarOriginInWorld);
   }
   
   /**
    * received a point cloud from the multisense
    * transform the points using the mocap data
    */
   @Override
   public void onNewMessage(PointCloud2 pointCloud)
   {
      UnpackedPointCloud pointCloudData = unpackPointsAndIntensities(pointCloud);
      Point3d[] points = pointCloudData.getPoints();
      
      for(int i = 0; i < points.length; i++)
      {
         lidarOriginInWorldAtomicReference.get().transform(points[i]);
      }
      
      MultisenseMocapExperimentPacket pointCloudPacket = new MultisenseMocapExperimentPacket();
      pointCloudPacket.setPointCloud(points, testInfo);
      packetCommunicator.send(pointCloudPacket);
   }
   
   public MultisenseFrameName getFrame()
   {
      return frame;
   }

}
