package us.ihmc.valkyrie.sensors;

import java.io.IOException;
import java.net.URI;

import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.CameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.RosCameraReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.SCSCameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.PointCloudDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.PointCloudSource;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.RosPointCloudReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.SCSPointCloudLidarReceiver;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.partNames.NeckJointName;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.wholeBodyController.DRCHandType;

public class ValkyrieSensorSuiteManager implements DRCSensorSuiteManager
{
   private final PacketCommunicator sensorSuitePacketCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.SENSOR_MANAGER,
         new IHMCCommunicationKryoNetClassList());

   
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final DRCRobotSensorInformation sensorInformation;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
   private final PointCloudDataReceiver pointCloudDataReceiver;
   private final SDFFullRobotModelFactory fullRobotModelFactory;

   public ValkyrieSensorSuiteManager(SDFFullRobotModelFactory fullRobotModelFactory, PPSTimestampOffsetProvider ppsTimestampOffsetProvider,
         DRCRobotSensorInformation sensorInformation, ValkyrieJointMap jointMap, boolean runningOnRealRobot)
   {
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.fullRobotModelFactory = fullRobotModelFactory;
      this.sensorInformation = sensorInformation;
      if(sensorInformation.getPointCloudParameters().length>0)
      {
         this.pointCloudDataReceiver = new PointCloudDataReceiver(fullRobotModelFactory, null, DRCHandType.VALKYRIE, ppsTimestampOffsetProvider, jointMap, robotConfigurationDataBuffer, sensorSuitePacketCommunicator);
      }
      else
      {
         this.pointCloudDataReceiver = null;
      }
   }

   @Override
   public void initializeSimulatedSensors(ObjectCommunicator scsSensorsCommunicator)
   {
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, robotConfigurationDataBuffer);

      CameraDataReceiver cameraDataReceiver = new SCSCameraDataReceiver(fullRobotModelFactory, sensorInformation.getCameraParameters(0).getSensorNameInSdf(), robotConfigurationDataBuffer, scsSensorsCommunicator, sensorSuitePacketCommunicator, ppsTimestampOffsetProvider);
      if (sensorInformation.getLidarParameters().length > 0)
      {
         new SCSPointCloudLidarReceiver(sensorInformation.getLidarParameters(0).getSensorNameInSdf(), scsSensorsCommunicator, pointCloudDataReceiver);
      }
      cameraDataReceiver.start();
   }

   @Override
   public void initializePhysicalSensors(URI sensorURI)
   {
	   if(sensorURI == null)
	   {
		   throw new IllegalArgumentException("The ros uri was null, val's physical sensors require a ros uri to be set! Check your Network Parameters.ini file");
	   }
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, robotConfigurationDataBuffer);
      
      RosMainNode rosMainNode = new RosMainNode(sensorURI, "darpaRoboticsChallange/networkProcessor");

      DRCRobotCameraParameters cameraParamaters = sensorInformation.getCameraParameters(0);

      CameraDataReceiver cameraDataReceiver = new RosCameraReceiver(fullRobotModelFactory, sensorInformation.getCameraParameters(0), robotConfigurationDataBuffer, rosMainNode, sensorSuitePacketCommunicator, ppsTimestampOffsetProvider, null, sensorURI);
      cameraDataReceiver.start();
      
      if(pointCloudDataReceiver != null)
      {
    	 ReferenceFrames referenceFrames = pointCloudDataReceiver.getReferenceFrames();
          
    	 ReferenceFrame neckFrame = referenceFrames.getNeckFrame(NeckJointName.UPPER_NECK_PITCH);
    	 ReferenceFrame ensensoFrame = new ReferenceFrame("ensensoFrame", neckFrame) {
			
			@Override
			protected void updateTransformToParent(RigidBodyTransform transformToParent) 
			{
				//<pose>-0.16 0.0065 -0.02325 -1.5708 2.11758e-22 3.14159</pose> a guess between the ibeo and forehead camera
				transformToParent.setEuler(0,Math.PI/2,0);
				transformToParent.setTranslation(0,0,0);
			}
		};
		new RosPointCloudReceiver(sensorInformation.getPointCloudParameters(0).getSensorNameInSdf(),sensorInformation.getPointCloudParameters(0).getRosTopic(),
               rosMainNode, ensensoFrame, pointCloudDataReceiver,PointCloudSource.NEARSCAN, PointCloudSource.QUADTREE);
      }
      
      ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);
      rosMainNode.execute();
   }


   @Override
   public void connect() throws IOException
   {
      sensorSuitePacketCommunicator.connect();
      if (sensorInformation.getLidarParameters().length > 0 || sensorInformation.getPointCloudParameters().length > 0)
      {
         if(pointCloudDataReceiver != null)
         {
            pointCloudDataReceiver.start();
         }
      }
   }
}
