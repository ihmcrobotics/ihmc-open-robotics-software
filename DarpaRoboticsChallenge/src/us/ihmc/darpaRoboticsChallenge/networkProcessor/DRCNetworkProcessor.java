package us.ihmc.darpaRoboticsChallenge.networkProcessor;

import com.martiansoftware.jsap.*;
import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.atlas.AlwaysZeroOffsetPPSTimestampOffsetProvider;
import us.ihmc.atlas.PPSTimestampOffsetProvider;
import us.ihmc.atlas.RealRobotPPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.configuration.DRCNetClassList;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotDataReceiver;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCSensorParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.CameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.GazeboCameraReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.MultiSenseCameraInfoReciever;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.SCSCameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.GazeboLidarDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.LidarDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.RobotBoundingBoxes;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.SCSLidarDataReceiver;
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
import us.ihmc.utilities.ros.RosMainNode;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

public class DRCNetworkProcessor
{
   private final VideoSettings videoSettings;

   private final ObjectCommunicator fieldComputerClient;
   private final AtomicSettableTimestampProvider timestampProvider = new AtomicSettableTimestampProvider();
   private final DRCNetworkProcessorNetworkingManager networkingManager;
   private final RobotPoseBuffer robotPoseBuffer;

   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;

   private final SDFFullRobotModel fullRobotModel;
   private final DRCRobotJointMap jointMap;
   private final RobotBoundingBoxes robotBoundingBoxes;

   private static String scsMachineIPAddress = DRCConfigParameters.ROBOT_CONTROLLER_IP_ADDRESS;
   private static String rosMasterURI = DRCConfigParameters.ROS_MASTER_URI;

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
      if (RosNativeNetworkProcessor.hasNativeLibrary())
      {
         rosNativeNetworkProcessor = RosNativeNetworkProcessor.getInstance(rosCoreURI.toString());
         rosNativeNetworkProcessor.connect();
      }
      else
      {
         rosNativeNetworkProcessor = null;
      }

      //      GeoregressionTransformListenerAndProvider transformForDrivingProviderListener = new GeoregressionTransformListenerAndProvider();

      CameraDataReceiver cameraDataReceiver = new GazeboCameraReceiver(robotPoseBuffer, videoSettings, rosMainNode, networkingManager,
            DRCSensorParameters.FIELD_OF_VIEW, ppsTimestampOffsetProvider);
      MultiSenseCameraInfoReciever cameraInfoReciever = new MultiSenseCameraInfoReciever(rosMainNode, fieldComputerClient);

      LidarDataReceiver lidarDataReceiver = new GazeboLidarDataReceiver(rosMainNode, robotPoseBuffer, networkingManager, fullRobotModel, robotBoundingBoxes,
            jointMap, fieldComputerClient, rosNativeNetworkProcessor, ppsTimestampOffsetProvider);
      new VRCScoreDataReceiver(networkingManager, lidarDataReceiver, rosNativeNetworkProcessor);

      ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);

      rosMainNode.execute();

      //      if (DRCConfigParameters.USE_DUMMY_DRIVNG)
      //      {
      //         DrivingProcessorFactory.createCheatingDrivingProcessor(networkingManager, cameraDataReceiver, timestampProvider, rosCoreURI.toString(),
      //                 transformForDrivingProviderListener);
      //      }
      //      else
      //      {
      //         DrivingProcessorFactory.createDrivingProcessor(networkingManager, cameraDataReceiver, timestampProvider, fieldComputerClient,
      //                 transformForDrivingProviderListener);
      //      }
   }

   public DRCNetworkProcessor(LocalObjectCommunicator scsCommunicator, ObjectCommunicator drcNetworkObjectCommunicator)
   {
      this(drcNetworkObjectCommunicator);
      CameraDataReceiver cameraDataReceiver = new SCSCameraDataReceiver(robotPoseBuffer, videoSettings, scsCommunicator, networkingManager,
            ppsTimestampOffsetProvider);
      new SCSLidarDataReceiver(robotPoseBuffer, scsCommunicator, networkingManager, fullRobotModel, robotBoundingBoxes, jointMap, fieldComputerClient,
            ppsTimestampOffsetProvider);

   }

   private DRCNetworkProcessor(ObjectCommunicator fieldComputerClient)
   {
      if (fieldComputerClient == null)
      {
         this.fieldComputerClient = new KryoObjectClient(scsMachineIPAddress, DRCConfigParameters.NETWORK_PROCESSOR_TO_CONTROLLER_TCP_PORT,
               new DRCNetClassList());
         ((KryoObjectClient) this.fieldComputerClient).setReconnectAutomatically(true);
      }
      else
      {
         this.fieldComputerClient = fieldComputerClient;
      }

      robotPoseBuffer = new RobotPoseBuffer(this.fieldComputerClient, 1000, timestampProvider);
      networkingManager = new DRCNetworkProcessorNetworkingManager(this.fieldComputerClient, timestampProvider);

      jointMap = new DRCRobotJointMap(DRCConfigParameters.robotModelToUse, false);
      JaxbSDFLoader loader = DRCRobotSDFLoader.loadDRCRobot(jointMap, true);
      fullRobotModel = loader.createFullRobotModel(jointMap);
      
      DRCRobotDataReceiver drcRobotDataReceiver = new DRCRobotDataReceiver(DRCConfigParameters.robotModelToUse, fullRobotModel);
      this.fieldComputerClient.attachListener(DRCJointConfigurationData.class, drcRobotDataReceiver);
      robotBoundingBoxes = new RobotBoundingBoxes(drcRobotDataReceiver, fullRobotModel);

      if (DRCConfigParameters.USING_REAL_HEAD)
      {
         ppsTimestampOffsetProvider = new RealRobotPPSTimestampOffsetProvider();
         videoSettings = VideoSettingsFactory.get32kBitSettingsWide();
      }
      else
      {
         ppsTimestampOffsetProvider = new AlwaysZeroOffsetPPSTimestampOffsetProvider();
         videoSettings = VideoSettingsFactory.get32kBitSettingsSquare();
      }

      try
      {
         this.fieldComputerClient.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static void main(String[] args) throws URISyntaxException, JSAPException
   {
      JSAP jsap = new JSAP();
      FlaggedOption scsIPFlag = new FlaggedOption("scs-ip").setLongFlag("scs-ip").setShortFlag(JSAP.NO_SHORTFLAG).setRequired(false)
            .setStringParser(JSAP.STRING_PARSER);
      FlaggedOption rosURIFlag = new FlaggedOption("ros-uri").setLongFlag("ros-uri").setShortFlag(JSAP.NO_SHORTFLAG).setRequired(false)
            .setStringParser(JSAP.STRING_PARSER);
      Switch simulateController = new Switch("simulate-controller").setShortFlag('d').setLongFlag(JSAP.NO_LONGFLAG);

      jsap.registerParameter(scsIPFlag);
      jsap.registerParameter(rosURIFlag);
      jsap.registerParameter(simulateController);

      JSAPResult config = jsap.parse(args);

      if (config.success())
      {
         if (config.getString(scsIPFlag.getID()) != null)
         {
            scsMachineIPAddress = config.getString(scsIPFlag.getID());
         }

         if (config.getString(rosURIFlag.getID()) != null)
         {
            rosMasterURI = config.getString(rosURIFlag.getID());
         }

         if (config.getBoolean(simulateController.getID()))
         {
            System.err.println("Simulating DRC Controller");
            ObjectCommunicator objectCommunicator = new LocalObjectCommunicator();

            new DummyController(rosMasterURI, objectCommunicator);
            new DRCNetworkProcessor(new URI(rosMasterURI), objectCommunicator);
         }
         else
         {
            new DRCNetworkProcessor(new URI(rosMasterURI));
         }
      }
      else
      {
         new DRCNetworkProcessor(new URI(rosMasterURI));
      }
   }
}
