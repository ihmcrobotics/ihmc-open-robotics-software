package us.ihmc.atlas.sensors;

import java.net.URI;

import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.SCSCameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.PointCloudDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.SCSPointCloudLidarReceiver;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.darpaRoboticsChallenge.sensors.multisense.MultiSenseSensorManager;
import us.ihmc.pathGeneration.footstepPlanner.FootstepPlanningParameterization;
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
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final DRCRobotSensorInformation sensorInformation;
   private final PointCloudDataReceiver pointCloudDataReceiver;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;
   private final SDFFullRobotModelFactory modelFactory;

   public AtlasSensorSuiteManager(SDFFullRobotModelFactory modelFactory, PPSTimestampOffsetProvider ppsTimestampOffsetProvider, DRCRobotSensorInformation sensorInformation,
         DRCRobotJointMap jointMap, AtlasPhysicalProperties physicalProperties, FootstepPlanningParameterization footstepParameters,
         DRCHandType handType, AtlasTarget targetDeployment)
   {
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.sensorInformation = sensorInformation;
      this.robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
      this.pointCloudDataReceiver = new PointCloudDataReceiver(modelFactory, handType, ppsTimestampOffsetProvider, jointMap, robotConfigurationDataBuffer, sensorSuitePacketCommunicator);
      this.modelFactory = modelFactory;
   }

   @Override
   public void initializeSimulatedSensors(PacketCommunicator scsSensorsCommunicator)
   {
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, robotConfigurationDataBuffer);

      SCSCameraDataReceiver cameraDataReceiver = new SCSCameraDataReceiver(modelFactory, sensorInformation.getCameraParameters(0).getSensorNameInSdf(), robotConfigurationDataBuffer, scsSensorsCommunicator, sensorSuitePacketCommunicator,
            ppsTimestampOffsetProvider);
      cameraDataReceiver.start();
      

      //      if (sensorInformation.getPointCloudParameters().length > 0)
      //      {
      //         new SCSPointCloudDataReceiver(depthDataProcessor, robotPoseBuffer, scsCommunicator);
      //      }

      if (sensorInformation.getLidarParameters().length > 0)
      {
         new SCSPointCloudLidarReceiver(sensorInformation.getLidarParameters(0).getSensorNameInSdf(), scsSensorsCommunicator, pointCloudDataReceiver);
         pointCloudDataReceiver.start();
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
      if(rosCoreURI==null)
         new RuntimeException(getClass().getSimpleName() + " Physical sensor requires rosURI to be set in "+ NetworkParameters.defaultParameterFile);
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, robotConfigurationDataBuffer);

      RosMainNode rosMainNode = new RosMainNode(rosCoreURI, "atlas/sensorSuiteManager", true);

//      DRCRobotCameraParameters leftFishEyeCameraParameters = sensorInformation.getCameraParameters(AtlasSensorInformation.BLACKFLY_LEFT_CAMERA_ID);
      DRCRobotCameraParameters multisenseLeftEyeCameraParameters = sensorInformation.getCameraParameters(AtlasSensorInformation.MULTISENSE_SL_LEFT_CAMERA_ID);
      DRCRobotLidarParameters multisenseLidarParameters = sensorInformation.getLidarParameters(AtlasSensorInformation.MULTISENSE_LIDAR_ID);
      DRCRobotPointCloudParameters multisenseStereoParameters = sensorInformation.getPointCloudParameters(AtlasSensorInformation.MULTISENSE_STEREO_ID);

      MultiSenseSensorManager multiSenseSensorManager = new MultiSenseSensorManager(modelFactory, pointCloudDataReceiver, robotConfigurationDataBuffer, rosMainNode,
            sensorSuitePacketCommunicator, ppsTimestampOffsetProvider, rosCoreURI, multisenseLeftEyeCameraParameters,
            multisenseLidarParameters, multisenseStereoParameters, sensorInformation.setupROSParameterSetters());
      pointCloudDataReceiver.start();

//      FishEyeDataReceiver fishEyeDataReceiver = new FishEyeDataReceiver(robotPoseBuffer, rosMainNode, sensorSuitePacketCommunicator,
//            DRCSensorParameters.DEFAULT_FIELD_OF_VIEW, ppsTimestampOffsetProvider, sensorInformation.setupROSParameterSetters());

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

   @Override
   public PacketCommunicator getProcessedSensorsCommunicator()
   {
      return sensorSuitePacketCommunicator;
   }

}
