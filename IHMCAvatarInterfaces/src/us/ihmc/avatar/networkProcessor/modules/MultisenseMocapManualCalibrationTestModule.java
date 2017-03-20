package us.ihmc.avatar.networkProcessor.modules;

import java.io.IOException;
import java.net.URI;
import java.util.ArrayList;

import optiTrack.IHMCMocapDataClient;
import optiTrack.MocapRigidBody;
import optiTrack.MocapRigidbodiesListener;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.mocap.MocapToHeadFrameConverter;
import us.ihmc.avatar.networkProcessor.modules.mocap.RosConnectedZeroPoseRobotConfigurationDataProducer;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.DetectedObjectPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.MultisenseTest;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.utilities.ros.RosMainNode;

public class MultisenseMocapManualCalibrationTestModule implements MocapRigidbodiesListener
{
   private static final boolean ENABLE_ZERO_POSE_CONFIGURATION_PUBLISHER = false;
   private static final boolean USE_ROBOT_FRAME = true;
   
   private static final int MULTISENSE_MOCAP_ID = 0;
   private static final NetClassList NETCLASSLIST = new IHMCCommunicationKryoNetClassList();

   private final PacketCommunicator packetCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.MULTISENSE_MOCAP_MANUAL_CALIBRATION_TEST_MODULE, NETCLASSLIST);
   private final ArrayList<MultisensePointCloudReceiver> multisensePointCloudReceivers = new ArrayList<MultisensePointCloudReceiver>();
   private final RosConnectedZeroPoseRobotConfigurationDataProducer zeroPoseProducer;
   private final MocapToHeadFrameConverter mocapToHeadFrameConverter;
   
   private final IHMCMocapDataClient mocapDataClient = new IHMCMocapDataClient();
   
   private final RigidBodyTransform mocapCalibrationTransform = new RigidBodyTransform();
   private static final RigidBodyTransform transformFromMocapCentroidToHeadRoot = new RigidBodyTransform();
   static 
   {
      transformFromMocapCentroidToHeadRoot.setTranslation(0, -0.035, 0.002);
   }
   
   /**
    * Used to inspect the lidar at each stage on the route from multisense to the DRCOperatorInterface
    */
   public MultisenseMocapManualCalibrationTestModule(DRCRobotModel robotModel, URI rosMasterURI)
   {
      RosMainNode rosMainNode = null;
      mocapToHeadFrameConverter = new MocapToHeadFrameConverter(robotModel, packetCommunicator);
      
      if (ENABLE_ZERO_POSE_CONFIGURATION_PUBLISHER)
      {
         zeroPoseProducer = new RosConnectedZeroPoseRobotConfigurationDataProducer(rosMasterURI, packetCommunicator, robotModel);
      } else {
         zeroPoseProducer = null;
      }
      
      mocapToHeadFrameConverter.setTransformFromMocapCentroidToHeadRoot(transformFromMocapCentroidToHeadRoot);
      
      if(rosMasterURI != null)
      {
         rosMainNode = new RosMainNode(rosMasterURI, "atlas/MultisenseMocapManualCalibrationTest", true);
      // for each of the MultisenseTests send a point cloud to the ui
         for (MultisenseTest topicToTest : MultisenseTest.values())
         {
            if(topicToTest != MultisenseTest.NEAR_SCAN_IN_POINT_CLOUD_DATA_RECEIVER)
            {
               MultisensePointCloudReceiver lidarReceiver = new MultisensePointCloudReceiver(packetCommunicator, topicToTest, robotModel);
               rosMainNode.attachSubscriber(topicToTest.getRosTopic(), lidarReceiver);
               multisensePointCloudReceivers.add(lidarReceiver);
            }
         }
         rosMainNode.execute();
      }
      
      mocapDataClient.registerRigidBodiesListener(this);
      connect();
   }

	private void connect() {
		try
	      {
	         packetCommunicator.connect();
	      }
	      catch (IOException e)
	      {
	         e.printStackTrace();
	      }
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
         RigidBodyTransform pose = new RigidBodyTransform();
         mocapObject.packPose(pose);
         int id = mocapObject.getId();

         if (id == MULTISENSE_MOCAP_ID)
         {
            //Set this to false to pause mocap updates. Remember to turn it back on before restarting
            boolean enableMocapUpdates = true;
            mocapToHeadFrameConverter.enableMocapUpdates(enableMocapUpdates);
            
            //manual calibration offsets from mocap jig misalignment
            //had to make pitch and roll negative to match frames of ui tool
            mocapCalibrationTransform.setRotationEulerAndZeroTranslation(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0));
            mocapToHeadFrameConverter.setMocapJigCalibrationTransform(mocapCalibrationTransform);
            mocapToHeadFrameConverter.update(mocapObject);
            
            RigidBodyTransform poseInStateEstimatorFrame = new RigidBodyTransform(pose);
            if(USE_ROBOT_FRAME)
            {
               poseInStateEstimatorFrame = mocapToHeadFrameConverter.convertMocapPoseToRobotFrame(mocapObject);
            }
            if (zeroPoseProducer != null)
            {
               zeroPoseProducer.updateRobotLocationBasedOnMultisensePose(poseInStateEstimatorFrame);
            }
            
            updateReceiversWithMocapPose(poseInStateEstimatorFrame);
         }
         sendDetectedObjectPacketToUi(mocapObject);
      }
   }

   /**
    * @param headPose the location of the multisense in world
    */
   private void updateReceiversWithMocapPose(RigidBodyTransform headPose)
   {
 	  for (int i = 0; i < multisensePointCloudReceivers.size(); i++)
 	  {
 		  MultisensePointCloudReceiver pointCloudReceiver = multisensePointCloudReceivers.get(i);
		  pointCloudReceiver.setHeadRootInWorldFromMocap(headPose);
      }
   }
   
   private void sendDetectedObjectPacketToUi(MocapRigidBody mocapObject)
   {
      RigidBodyTransform pose = new RigidBodyTransform();
      mocapObject.packPose(pose);
      if(USE_ROBOT_FRAME)
      {
         pose = mocapToHeadFrameConverter.convertMocapPoseToRobotFrame(mocapObject);
      }
      DetectedObjectPacket detectedMocapObject = new DetectedObjectPacket(pose, mocapObject.getId());
      if(packetCommunicator.isConnected())
      {
         packetCommunicator.send(detectedMocapObject);
      }
   }
}


