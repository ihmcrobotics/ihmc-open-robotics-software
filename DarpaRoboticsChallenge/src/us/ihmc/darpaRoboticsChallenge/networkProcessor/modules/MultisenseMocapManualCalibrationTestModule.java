package us.ihmc.darpaRoboticsChallenge.networkProcessor.modules;

import java.io.IOException;
import java.net.URI;
import java.util.ArrayList;

import optitrack.MocapDataClient;
import optitrack.MocapRigidBody;
import optitrack.MocapRigidbodiesListener;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.sensing.MultisenseTest;
import us.ihmc.communication.packets.sensing.MultisenseTest.MultisenseFrameName;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.mocap.RosConnectedZeroPoseRobotConfigurationDataProducer;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.ros.RosMainNode;

public class MultisenseMocapManualCalibrationTestModule implements MocapRigidbodiesListener
{
   private static final boolean ENABLE_ZERO_POSE_CONFIGURATION_PUBLISHER = true;
   private static final int MULTISENSE_MOCAP_ID = 0;
   private static final NetClassList NETCLASSLIST = new IHMCCommunicationKryoNetClassList();

   private final PacketCommunicator packetCommunicator =
      PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.MULTISENSE_MOCAP_MANUAL_CALIBRATION_TEST_MODULE, NETCLASSLIST);
   private final ArrayList<MultisensePointCloudReceiver> multisensePointCloudReceivers = new ArrayList<MultisensePointCloudReceiver>();
   private final RosConnectedZeroPoseRobotConfigurationDataProducer zeroPoseProducer;
   private final MocapDataClient mocapDataClient;

   /** Manual calibration offset to correct for errors in mocap rig */
   private final RigidBodyTransform mocapManualCalibration = new RigidBodyTransform();

   /**
    * Used to inspect the lidar at each stage on the route from multisense to the DRCOperatorInterface
    * @param rosMasterURI
    * @param robotModel
    */
   public MultisenseMocapManualCalibrationTestModule(DRCRobotModel robotModel, URI rosMasterURI)
   {
      RosMainNode rosMainNode = new RosMainNode(rosMasterURI, "atlas/MultisenseMocapManualCalibrationTest", true);

      // for each of the MultisenseTests send a point cloud to the ui
      for (MultisenseTest topicToTest : MultisenseTest.values())
      {
         MultisensePointCloudReceiver lidarReceiver = new MultisensePointCloudReceiver(packetCommunicator, topicToTest);
         rosMainNode.attachSubscriber(topicToTest.getRosTopic(), lidarReceiver);
         multisensePointCloudReceivers.add(lidarReceiver);
      }

      if (ENABLE_ZERO_POSE_CONFIGURATION_PUBLISHER)
      {
         zeroPoseProducer = new RosConnectedZeroPoseRobotConfigurationDataProducer(rosMasterURI, packetCommunicator, robotModel);
      }
      else
      {
         zeroPoseProducer = null;
      }


      mocapDataClient = new MocapDataClient();
      mocapDataClient.registerRigidBodiesListener(this);

      try
      {
         packetCommunicator.connect();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      rosMainNode.execute();
   }

   /**
    * received an update from mocap
    * convert the mocap head pose to zUpFrame
    * send the pose to the multisense lidar receivers
    */
   @Override
   public void updateRigidbodies(ArrayList<MocapRigidBody> listOfRigidbodies)
   {
      for (int rigidBodyIndex = 0; rigidBodyIndex < listOfRigidbodies.size(); rigidBodyIndex++)
      {
         MocapRigidBody mocapObject = listOfRigidbodies.get(rigidBodyIndex);

         if (mocapObject.getId() == MULTISENSE_MOCAP_ID)
         {
            RigidBodyTransform pose = new RigidBodyTransform();
            mocapObject.getPose(pose);

            pose.multiply(mocapManualCalibration, pose);
            updateReceiversWithMocapPose(pose);
         }
      }
   }

   /**
    * @param headPose the location of the multisense in world
    */
   private void updateReceiversWithMocapPose(RigidBodyTransform headPose)
   {
      if (zeroPoseProducer != null)
      {
         zeroPoseProducer.updateRobotLocationBasedOnMultisensePose(headPose);
      }

      for (int i = 0; i < multisensePointCloudReceivers.size(); i++)
      {
         MultisensePointCloudReceiver pointCloudReceiver = multisensePointCloudReceivers.get(i);
         if (pointCloudReceiver.getFrame() != MultisenseFrameName.WORLD)
         {
            pointCloudReceiver.setWorldTransform(headPose);
         }
      }
   }
}


//~ Formatted by Jindent --- http://www.jindent.com
