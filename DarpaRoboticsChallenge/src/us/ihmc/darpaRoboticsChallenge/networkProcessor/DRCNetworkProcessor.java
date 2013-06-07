package us.ihmc.darpaRoboticsChallenge.networkProcessor;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.configuration.DRCNetClassList;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotDataReceiver;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCSensorParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.CameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.GazeboCameraReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.SCSCameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.driving.DrivingProcessorFactory;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.GazeboLidarDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.LidarDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.RobotBoundingBoxes;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.SCSLidarDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosMainNode;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosNativeNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.state.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.vrc.VRCScoreDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.darpaRoboticsChallenge.networking.dataProducers.DRCJointConfigurationData;
import us.ihmc.graphics3DAdapter.camera.VideoSettings;
import us.ihmc.graphics3DAdapter.camera.VideoSettingsFactory;
import us.ihmc.utilities.net.AtomicSettableTimestampProvider;
import us.ihmc.utilities.net.KryoObjectClient;
import us.ihmc.utilities.net.LocalObjectCommunicator;
import us.ihmc.utilities.net.ObjectCommunicator;

public class DRCNetworkProcessor
{
   //TODO: make command line option
   private final VideoSettings videoSettings = VideoSettingsFactory.get32kBitSettings();
   
   
   private final ObjectCommunicator fieldComputerClient;
   private final AtomicSettableTimestampProvider timestampProvider = new AtomicSettableTimestampProvider();
   private final DRCNetworkProcessorNetworkingManager networkingManager;
   private final RobotPoseBuffer robotPoseBuffer;

  
   private final SDFFullRobotModel fullRobotModel;
   private final DRCRobotJointMap jointMap;
   private final RobotBoundingBoxes robotBoundingBoxes;

   /*
    * This will become a stand-alone application in the final competition. Do
    * NOT pass in objects shared with the DRC simulation!
    */
   public DRCNetworkProcessor(URI rosCoreURI)
   {
      this(rosCoreURI, null);
   }
   
   public DRCNetworkProcessor(URI rosCoreURI, ObjectCommunicator drcNetworkObjectCommunicator)
   {
      this(drcNetworkObjectCommunicator);

      System.out.println("Connecting to ROS");
      
      RosMainNode rosMainNode;
      rosMainNode = new RosMainNode(rosCoreURI, "darpaRoboticsChallange/networkProcessor");
      
      RosNativeNetworkProcessor rosNativeNetworkProcessor;
      if(RosNativeNetworkProcessor.hasNativeLibrary())
      {
         rosNativeNetworkProcessor = RosNativeNetworkProcessor.getInstance(rosCoreURI.toString());
         rosNativeNetworkProcessor.connect();
      }
      else
      {
         rosNativeNetworkProcessor = null;
      }

      
      CameraDataReceiver cameraDataReceiver = new GazeboCameraReceiver(robotPoseBuffer, videoSettings, rosMainNode, networkingManager, DRCSensorParameters.FIELD_OF_VIEW);
      LidarDataReceiver lidarDataReceiver = new GazeboLidarDataReceiver(rosMainNode, robotPoseBuffer, networkingManager, fullRobotModel, robotBoundingBoxes, jointMap, rosNativeNetworkProcessor);
      new VRCScoreDataReceiver(networkingManager, lidarDataReceiver, rosNativeNetworkProcessor);
      rosMainNode.execute();
      
      
      DrivingProcessorFactory.createCheatingDrivingProcessor(networkingManager.getControllerCommandHandler(), cameraDataReceiver, timestampProvider, rosCoreURI.toString());
   }
   
   public DRCNetworkProcessor(LocalObjectCommunicator scsCommunicator, ObjectCommunicator drcNetworkObjectCommunicator)
   {
      this(drcNetworkObjectCommunicator);
      new SCSCameraDataReceiver(robotPoseBuffer, videoSettings, scsCommunicator, networkingManager);
      new SCSLidarDataReceiver(robotPoseBuffer, scsCommunicator, networkingManager, fullRobotModel, robotBoundingBoxes, jointMap);
   }

   private DRCNetworkProcessor(ObjectCommunicator fieldComputerClient)
   {
      if(fieldComputerClient == null)
      {
         this.fieldComputerClient = new KryoObjectClient(DRCConfigParameters.SCS_MACHINE_IP_ADDRESS, DRCConfigParameters.NETWORK_PROCESSOR_TO_CONTROLLER_TCP_PORT,
               new DRCNetClassList());
         ((KryoObjectClient)this.fieldComputerClient).setReconnectAutomatically(true);
      }
      else
      {
         this.fieldComputerClient = fieldComputerClient;
      }
     
      robotPoseBuffer = new RobotPoseBuffer(this.fieldComputerClient, 1000, timestampProvider);
      networkingManager = new DRCNetworkProcessorNetworkingManager(this.fieldComputerClient, timestampProvider);
      
      jointMap = new DRCRobotJointMap(DRCRobotModel.ATLAS_SANDIA_HANDS, false);  
      JaxbSDFLoader loader = DRCRobotSDFLoader.loadDRCRobot(jointMap, true);
      
      fullRobotModel = loader.createFullRobotModel(jointMap);
      DRCRobotDataReceiver drcRobotDataReceiver = new DRCRobotDataReceiver(DRCRobotModel.ATLAS_SANDIA_HANDS, fullRobotModel);
      this.fieldComputerClient.attachListener(DRCJointConfigurationData.class, drcRobotDataReceiver);
      robotBoundingBoxes = new RobotBoundingBoxes(drcRobotDataReceiver, fullRobotModel);
      
      try
      {
         this.fieldComputerClient.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }


   
   public static void main(String[] args) throws URISyntaxException
   {
      
      if(args.length == 1)
      {
         new DRCNetworkProcessor(new URI(args[0]));
      }
      else
      {
         new DRCNetworkProcessor(new URI(DRCConfigParameters.ROS_MASTER_URI));
      }
   }
}
