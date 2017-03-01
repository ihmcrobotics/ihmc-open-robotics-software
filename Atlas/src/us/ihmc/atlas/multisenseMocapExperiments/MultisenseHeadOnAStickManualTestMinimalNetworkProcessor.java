package us.ihmc.atlas.multisenseMocapExperiments;

import java.io.IOException;
import java.net.URI;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import com.martiansoftware.jsap.JSAPException;

import optiTrack.IHMCMocapDataClient;
import optiTrack.MocapRigidBody;
import optiTrack.MocapRigidbodiesListener;
import sensor_msgs.PointCloud2;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

/**
 * Pulls lidar from the multisense over the /multisense/lidar_points2 ros topic
 * Puts it in mocap head frame
 * sends it to the LidarMocapViewerBasic UI.
 * 
 * 	  //head 39 calibrated by Daniel and Brandon April 22 2015
 *    //head 39: setEuler(Math.toRadians(-1.4), Math.toRadians(0.5), Math.toRadians(2.0)
      //head 39: setTranslation(-0.005,-0.003,-0.003)
 *
 */
public class MultisenseHeadOnAStickManualTestMinimalNetworkProcessor  extends RosPointCloudSubscriber implements MocapRigidbodiesListener
{
   private static final NetClassList NETCLASSLIST = new IHMCCommunicationKryoNetClassList();

   private static final int MULTISENSE_MOCAP_ID = 0;

   private final PacketCommunicator uiPacketServer = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.NETWORK_PROCESSOR_TO_UI_TCP_PORT, NETCLASSLIST);
   private final RigidBodyTransform orientationTransformFromLeftOpticalFrameToZUp = new RigidBodyTransform();
   private final IHMCMocapDataClient mocapDataClient;
   private final AtomicReference<RigidBodyTransform> headPoseInZUp = new  AtomicReference<RigidBodyTransform>(new RigidBodyTransform());
   
   public MultisenseHeadOnAStickManualTestMinimalNetworkProcessor(DRCRobotModel robotModel) throws IOException
   {
      URI rosMasterURI = NetworkParameters.getROSURI();
      RosMainNode rosMainNode = new RosMainNode(rosMasterURI, "atlas/AtlasMinimalMultisenseMocapNetworkProcessor", true);
      rosMainNode.attachSubscriber("/multisense/lidar_points2", this);
      rosMainNode.execute();
      uiPacketServer.connect();
      
      mocapDataClient = new IHMCMocapDataClient();
      mocapDataClient.registerRigidBodiesListener(this);
      System.out.println("running minimal experiment");
   }
   
   RigidBodyTransform rpyCalibrationOffset = new RigidBodyTransform();
   
   /**
    * received a point cloud from the multisense
    * rotate the points to zUp from left camera optical frame
    * offset the points to account for mocap rig problems
    * transform the points using the mocap data
    */
   @Override
   public void onNewMessage(PointCloud2 pointCloud)
   {
      UnpackedPointCloud pointCloudData = unpackPointsAndIntensities(pointCloud);
      Point3D[] points = pointCloudData.getPoints();
      orientationTransformFromLeftOpticalFrameToZUp.setRotationEulerAndZeroTranslation(-Math.PI/2, 0.0, -Math.PI/2);
      
      rpyCalibrationOffset.setRotationEulerAndZeroTranslation(Math.toRadians(-1.4), Math.toRadians(0.5), Math.toRadians(2.0));
      rpyCalibrationOffset.setTranslation(-0.005,-0.003,-0.003);
      
      for(int i = 0; i < points.length; i++)
      {
         orientationTransformFromLeftOpticalFrameToZUp.transform(points[i]);
         rpyCalibrationOffset.transform(points[i]);
         headPoseInZUp.get().transform(points[i]);
      }
      
      PointCloudWorldPacket pointCloudPacket = new PointCloudWorldPacket();
      pointCloudPacket.setDecayingWorldScan(points);
      uiPacketServer.send(pointCloudPacket);
   }
   
   /**
    * received an update from mocap
    * gets the head pose in mocap zUp
    */
   @Override
   public void updateRigidbodies(ArrayList<MocapRigidBody> listOfRigidbodies)
   {
      for (int i = 0; i < listOfRigidbodies.size(); i++)
      {
         MocapRigidBody mocapObject = listOfRigidbodies.get(i);
         int id = mocapObject.getId();

         if (id == MULTISENSE_MOCAP_ID)
         {
            RigidBodyTransform pose = new RigidBodyTransform();
            mocapObject.packPose(pose);
            headPoseInZUp.set(pose);
         }
      }
   }

   public static void main(String[] args) throws JSAPException, IOException
   {
      boolean headless = false;
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.HEAD_ON_A_STICK, headless);
      new MultisenseHeadOnAStickManualTestMinimalNetworkProcessor(robotModel);
   }
}
