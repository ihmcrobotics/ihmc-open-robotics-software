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
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.SCSCameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.PointCloudDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.SCSCheatingPointCloudLidarReceiver;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.darpaRoboticsChallenge.sensors.multisense.MultiSenseSensorManager;
import us.ihmc.ihmcPerception.depthData.RobotBoundingBoxes;
import us.ihmc.pathGeneration.footstepPlanner.FootstepPlanningParameterization;
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
   private final RobotDataReceiver drcRobotDataReceiver;
   private final PointCloudDataReceiver pointCloudDataReceiver;
   private final RobotPoseBuffer robotPoseBuffer;
   private final RobotBoundingBoxes robotBoundingBoxes;

   public AtlasSensorSuiteManager(PPSTimestampOffsetProvider ppsTimestampOffsetProvider, DRCRobotSensorInformation sensorInformation,
         DRCRobotJointMap jointMap, AtlasPhysicalProperties physicalProperties, FootstepPlanningParameterization footstepParameters, SDFFullRobotModel sdfFullRobotModel,
         DRCHandType handType, AtlasTarget targetDeployment)
   {
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.sensorInformation = sensorInformation;
      this.drcRobotDataReceiver = new RobotDataReceiver(sdfFullRobotModel, null, true);
      this.robotBoundingBoxes = new RobotBoundingBoxes(drcRobotDataReceiver, handType, sdfFullRobotModel);
      this.robotPoseBuffer = new RobotPoseBuffer(sensorSuitePacketCommunicator, 10000, timestampProvider);
      this.pointCloudDataReceiver = new PointCloudDataReceiver(sdfFullRobotModel, jointMap, robotPoseBuffer, sensorSuitePacketCommunicator);
      
   }

   @Override
   public void initializeSimulatedSensors(PacketCommunicator scsSensorsCommunicator)
   {
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, drcRobotDataReceiver);

      new SCSCameraDataReceiver(robotPoseBuffer, scsSensorsCommunicator, sensorSuitePacketCommunicator,
            ppsTimestampOffsetProvider);

      //      if (sensorInformation.getPointCloudParameters().length > 0)
      //      {
      //         new SCSPointCloudDataReceiver(depthDataProcessor, robotPoseBuffer, scsCommunicator);
      //      }

      if (sensorInformation.getLidarParameters().length > 0)
      {
         new SCSCheatingPointCloudLidarReceiver(robotBoundingBoxes, scsSensorsCommunicator, pointCloudDataReceiver);
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
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, drcRobotDataReceiver);

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

//      DRCRobotCameraParameters leftFishEyeCameraParameters = sensorInformation.getCameraParameters(AtlasSensorInformation.BLACKFLY_LEFT_CAMERA_ID);
      DRCRobotCameraParameters multisenseLeftEyeCameraParameters = sensorInformation.getCameraParameters(AtlasSensorInformation.MULTISENSE_SL_LEFT_CAMERA_ID);
      DRCRobotLidarParameters multisenseLidarParameters = sensorInformation.getLidarParameters(AtlasSensorInformation.MULTISENSE_LIDAR_ID);
      DRCRobotPointCloudParameters multisenseStereoParameters = sensorInformation.getPointCloudParameters(AtlasSensorInformation.MULTISENSE_STEREO_ID);

      MultiSenseSensorManager multiSenseSensorManager = new MultiSenseSensorManager(pointCloudDataReceiver, rosTransformProvider, robotPoseBuffer, rosMainNode,
            sensorSuitePacketCommunicator, rosNativeNetworkProcessor, ppsTimestampOffsetProvider, rosCoreURI, multisenseLeftEyeCameraParameters,
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
