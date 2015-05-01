package us.ihmc.darpaRoboticsChallenge.networkProcessor.modules;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;

import optitrack.IHMCMocapDataClient;
import optitrack.MocapRigidBody;
import optitrack.MocapRigidbodiesListener;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.sensing.MultisenseTest;
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

   private final PacketCommunicator packetCommunicator;
   private final ArrayList<MultisensePointCloudReceiver> multisensePointCloudReceivers = new ArrayList<MultisensePointCloudReceiver>();
   private final RosConnectedZeroPoseRobotConfigurationDataProducer zeroPoseProducer;
   private IHMCMocapDataClient mocapDataClient;
   private final RigidBodyTransform mocapCalibrationTransform = new RigidBodyTransform();
   private final RigidBodyTransform transformFromMocapCentroidToHeadRoot = new RigidBodyTransform();
   

   public MultisenseMocapManualCalibrationTestModule(URI rosMasterURI)
   {
	   packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.NETWORK_PROCESSOR_TO_UI_TCP_PORT, NETCLASSLIST);
	   zeroPoseProducer = null;
	   setupRosConnections(rosMasterURI);
	   setupMocap();
	   connect();
   }
   
   /**
    * Used to inspect the lidar at each stage on the route from multisense to the DRCOperatorInterface
    * @param rosMasterURI
    * @param robotModel
    */
   public MultisenseMocapManualCalibrationTestModule(DRCRobotModel robotModel, URI rosMasterURI)
   {
	   packetCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.MULTISENSE_MOCAP_MANUAL_CALIBRATION_TEST_MODULE, NETCLASSLIST);

      if (ENABLE_ZERO_POSE_CONFIGURATION_PUBLISHER)
      {
         zeroPoseProducer = new RosConnectedZeroPoseRobotConfigurationDataProducer(rosMasterURI, packetCommunicator, robotModel);
      }
      else
      {
         zeroPoseProducer = null;
      }
      setupRosConnections(rosMasterURI);
      setupMocap();
      connect();
   }

	private void setupMocap() 
	{
	   transformFromMocapCentroidToHeadRoot.setTranslation(0, -0.035, 0.002);
		mocapDataClient = new IHMCMocapDataClient();
		mocapDataClient.registerRigidBodiesListener(this);
	}
   
   public void setupRosConnections(URI rosMasterURI)
   {
	   RosMainNode rosMainNode = new RosMainNode(rosMasterURI, "atlas/MultisenseMocapManualCalibrationTest", true);
	   
	   // for each of the MultisenseTests send a point cloud to the ui
	   for (MultisenseTest topicToTest : MultisenseTest.values())
	   {
		   if(topicToTest != MultisenseTest.NEAR_SCAN_IN_POINT_CLOUD_DATA_RECEIVER)
		   {
			   MultisensePointCloudReceiver lidarReceiver = new MultisensePointCloudReceiver(packetCommunicator, topicToTest, zeroPoseProducer.getReferenceFrames());
			   rosMainNode.attachSubscriber(topicToTest.getRosTopic(), lidarReceiver);
			   multisensePointCloudReceivers.add(lidarReceiver);
		   }
	   }
	   rosMainNode.execute();
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

         if (mocapObject.getId() == MULTISENSE_MOCAP_ID)
         {
            RigidBodyTransform pose = new RigidBodyTransform();
            mocapObject.getPose(pose);
            
            //manual calibration offsets from mocap jig misalignment
            mocapCalibrationTransform.setEuler(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(-0.95));
            mocapCalibrationTransform.setTranslation(0.0, 0.0, 0.0);
          
            pose.multiply(mocapCalibrationTransform);
            pose.multiply(transformFromMocapCentroidToHeadRoot);
            
            //Set this to false to pause mocap updates. Remember to turn it back on before restarting
            boolean UPDATE_LOCATION_IN_WORLD_USING_MOCAP = true;
            if(UPDATE_LOCATION_IN_WORLD_USING_MOCAP )
            {
               updateReceiversWithMocapPose(pose);
            }
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
		  pointCloudReceiver.setHeadRootInWorldFromMocap(headPose);
      }
   }
   
   public static void main(String[] args) throws URISyntaxException
   {
	   new MultisenseMocapManualCalibrationTestModule(new URI("http://localhost:11311"));
   }
   
}


