package us.ihmc.atlas.sensors;

import java.net.URI;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.AtomicSettableTimestampProvider;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.producers.RobotPoseBuffer;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.communication.util.DRCSensorParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.FishEyeDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.SCSCameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.DepthDataProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.SCSLidarDataReceiver;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.darpaRoboticsChallenge.sensors.multisense.MultiSenseSensorManager;
import us.ihmc.ihmcPerception.depthData.DepthDataFilter;
import us.ihmc.ihmcPerception.depthData.RobotBoundingBoxes;
import us.ihmc.ihmcPerception.depthData.RobotDepthDataFilter;
import us.ihmc.ihmcPerception.footstepPlanner.FootstepParameters;
import us.ihmc.ros.jni.wrapper.ROSNativeTransformTools;
import us.ihmc.ros.jni.wrapper.RosNativeNetworkProcessor;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotPointCloudParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.wholeBodyController.DRCHandType;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class AtlasSensorSuiteManager implements DRCSensorSuiteManager
{
   private final KryoLocalPacketCommunicator sensorSuitePacketCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
         PacketDestination.SENSOR_MANAGER.ordinal(), "ATLAS_SENSOR_MANAGER");
   private final AtomicSettableTimestampProvider timestampProvider = new AtomicSettableTimestampProvider();
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final DRCRobotSensorInformation sensorInformation;
   private final SDFFullRobotModel sdfFullRobotModel;
   private final RobotDataReceiver drcRobotDataReceiver;
   private final DepthDataFilter lidarDataFilter;
   private final RobotBoundingBoxes robotBoundingBoxes;
   private final AtlasTarget targetDeployment;
   private RobotPoseBuffer robotPoseBuffer;
   private DepthDataProcessor depthDataProcessor;

   public AtlasSensorSuiteManager(PPSTimestampOffsetProvider ppsTimestampOffsetProvider, DRCRobotSensorInformation sensorInformation,
         DRCRobotJointMap jointMap, AtlasPhysicalProperties physicalProperties, FootstepParameters footstepParameters, SDFFullRobotModel sdfFullRobotModel,
         DRCHandType handType, AtlasTarget targetDeployment)
   {
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.sensorInformation = sensorInformation;
      this.sdfFullRobotModel = sdfFullRobotModel;
      this.targetDeployment = targetDeployment;
      this.drcRobotDataReceiver = new RobotDataReceiver(sdfFullRobotModel, null, true);
      this.robotBoundingBoxes = new RobotBoundingBoxes(drcRobotDataReceiver, handType, sdfFullRobotModel);
      this.lidarDataFilter = new RobotDepthDataFilter(robotBoundingBoxes, sdfFullRobotModel);
   }

   @Override
   public void initializeSimulatedSensors(PacketCommunicator scsSensorsCommunicator)
   {
      robotPoseBuffer = new RobotPoseBuffer(sensorSuitePacketCommunicator, 10000, timestampProvider);
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, drcRobotDataReceiver);

      this.depthDataProcessor = new DepthDataProcessor(sensorSuitePacketCommunicator, lidarDataFilter);
      SCSCameraDataReceiver cameraReceiver = new SCSCameraDataReceiver(robotPoseBuffer, scsSensorsCommunicator, sensorSuitePacketCommunicator,
            ppsTimestampOffsetProvider);

      //      if (sensorInformation.getPointCloudParameters().length > 0)
      //      {
      //         new SCSPointCloudDataReceiver(depthDataProcessor, robotPoseBuffer, scsCommunicator);
      //      }

      if (sensorInformation.getLidarParameters().length > 0)
      {
         SCSLidarDataReceiver scsLidarDataReceiver = new SCSLidarDataReceiver(depthDataProcessor, robotPoseBuffer, scsSensorsCommunicator,
               ppsTimestampOffsetProvider, sdfFullRobotModel, sensorInformation.getLidarParameters());
         Thread lidarThread = new Thread(scsLidarDataReceiver, "scsLidarDataReceiver");
         lidarThread.start();
      }

      //      if (DRCConfigParameters.CALIBRATE_ARM_MODE)
      //      {
      //         ArmCalibrationHelper armCalibrationHelper = new ArmCalibrationHelper(sensorSuitePacketCommunicator, jointMap);
      //         cameraReceiver.registerCameraListener(armCalibrationHelper);
      //      }

      IMUBasedHeadPoseCalculatorFactory.create(scsSensorsCommunicator, sensorInformation);
   }

   @Override
   public void initializePhysicalSensors(URI rosCoreURI)
   {
      robotPoseBuffer = new RobotPoseBuffer(sensorSuitePacketCommunicator, 100000, timestampProvider);
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, drcRobotDataReceiver);

      this.depthDataProcessor = new DepthDataProcessor(sensorSuitePacketCommunicator, lidarDataFilter);
      RosMainNode rosMainNode = new RosMainNode(rosCoreURI, "atlas/sensorSuiteManager", true);

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
      ROSNativeTransformTools rosTransformProvider = ROSNativeTransformTools.getInstance(rosCoreURI);
      rosTransformProvider.connect();

      DRCRobotCameraParameters leftFishEyeCameraParameters = sensorInformation.getCameraParameters(AtlasSensorInformation.BLACKFLY_LEFT_CAMERA_ID);
      DRCRobotCameraParameters multisenseLeftEyeCameraParameters = sensorInformation.getCameraParameters(AtlasSensorInformation.MULTISENSE_SL_LEFT_CAMERA_ID);
      DRCRobotLidarParameters multisenseLidarParameters = sensorInformation.getLidarParameters(AtlasSensorInformation.MULTISENSE_LIDAR_ID);
      DRCRobotPointCloudParameters multisenseStereoParameters = sensorInformation.getPointCloudParameters(AtlasSensorInformation.MULTISENSE_STEREO_ID);

      MultiSenseSensorManager multiSenseSensorManager = new MultiSenseSensorManager(depthDataProcessor, rosTransformProvider, robotPoseBuffer, rosMainNode,
            sensorSuitePacketCommunicator, rosNativeNetworkProcessor, ppsTimestampOffsetProvider, rosCoreURI, multisenseLeftEyeCameraParameters,
            multisenseLidarParameters, multisenseStereoParameters, sensorInformation.setupROSParameterSetters());

      FishEyeDataReceiver fishEyeDataReceiver = new FishEyeDataReceiver(robotPoseBuffer, rosMainNode, sensorSuitePacketCommunicator,
            DRCSensorParameters.DEFAULT_FIELD_OF_VIEW, ppsTimestampOffsetProvider, sensorInformation.setupROSParameterSetters());

      ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);

      //      if (DRCConfigParameters.CALIBRATE_ARM_MODE)
      //      {
      //         ArmCalibrationHelper armCalibrationHelper = new ArmCalibrationHelper(sensorSuitePacketCommunicator, jointMap);
      //         multiSenseSensorManager.registerCameraListener(armCalibrationHelper);
      //      }

      multiSenseSensorManager.initializeParameterListeners();

      IMUBasedHeadPoseCalculatorFactory.create(sensorSuitePacketCommunicator, sensorInformation, rosMainNode);
      rosMainNode.execute();
   }

   public PacketCommunicator createSensorModule(URI sensorURI)
   {
      KryoLocalPacketCommunicator sensorSuitePacketCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
            PacketDestination.SENSOR_MANAGER.ordinal(), "ATLAS_SENSOR_MANAGER");

      switch (targetDeployment)
      {
      case GAZEBO:
         System.out.println("Not sure what to do about gazebo sensors!!!");
         break;
      case REAL_ROBOT:
         initializePhysicalSensors(sensorURI);
         break;
      case SIM:
         //         initializeSimulatedSensors(sensorSuitePacketCommunicator);
         break;
      }
      return sensorSuitePacketCommunicator;
   }

   @Override
   public PacketCommunicator getProcessedSensorsCommunicator()
   {
      return sensorSuitePacketCommunicator;
   }

}
