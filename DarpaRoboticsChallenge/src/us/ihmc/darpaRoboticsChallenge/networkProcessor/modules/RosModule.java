package us.ihmc.darpaRoboticsChallenge.networkProcessor.modules;

import java.net.URI;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.sensing.LocalizationPacket;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.ros.RosRobotJointStatePublisher;
import us.ihmc.darpaRoboticsChallenge.ros.RosSCSCameraPublisher;
import us.ihmc.darpaRoboticsChallenge.ros.RosSCSLidarPublisher;
import us.ihmc.darpaRoboticsChallenge.ros.RosTfPublisher;
import us.ihmc.ihmcPerception.RosLocalizationServiceClient;
import us.ihmc.ihmcPerception.RosLocalizationUpdateSubscriber;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;


public class RosModule
{
   private static final boolean DEBUG = false;

   private static final String ROS_NAMESPACE = "networkProcessor/rosModule";

   private final KryoLocalPacketCommunicator rosModulePacketCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
         PacketDestination.ROS_MODULE.ordinal(), "RosModule");
   
   private final RosMainNode rosMainNode;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final DRCRobotSensorInformation sensorInformation;
//   private final DRCRobotPhysicalProperties physicalProperties;

   
   
   public RosModule(DRCRobotModel robotModel, URI rosCoreURI, PacketCommunicator simulatedSensorCommunicator)
   {
      rosMainNode = new RosMainNode(rosCoreURI, ROS_NAMESPACE, true);
      
//      physicalProperties = robotModel.getPhysicalProperties();
      ppsTimestampOffsetProvider = robotModel.getPPSTimestampOffsetProvider();
      ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);
      rosModulePacketCommunicator.attachListener(RobotConfigurationData.class, ppsTimestampOffsetProvider);
      
      sensorInformation = robotModel.getSensorInformation();

      RosTfPublisher tfPublisher = new RosTfPublisher(rosMainNode);

      new RosRobotJointStatePublisher(robotModel, rosModulePacketCommunicator, rosMainNode, ppsTimestampOffsetProvider,robotModel.getSimpleRobotName().toLowerCase(),tfPublisher);

      if(sensorInformation.setupROSLocationService())
      {
         setupRosLocalization();
      }

//      setupFootstepServiceClient();
//      setupFootstepPathPlannerService();
      if(simulatedSensorCommunicator != null)
      {
         publishSimulatedCameraAndLidar(robotModel.createFullRobotModel(), sensorInformation, tfPublisher, simulatedSensorCommunicator);
      }

      System.out.flush();
      rosMainNode.execute();
      printIfDebug("Finished creating ROS Module.");
   }

   private void publishSimulatedCameraAndLidar(SDFFullRobotModel fullRobotModel, DRCRobotSensorInformation sensorInformation, RosTfPublisher tfPublisher, PacketCommunicator simulatedSensorCommunicator)
   {
      if (sensorInformation.getCameraParameters().length > 0)
      {
         new RosSCSCameraPublisher(simulatedSensorCommunicator, rosMainNode, ppsTimestampOffsetProvider, sensorInformation.getCameraParameters());
      }

      if (sensorInformation.getLidarParameters().length > 0)
      {
         new RosSCSLidarPublisher(simulatedSensorCommunicator, rosMainNode, ppsTimestampOffsetProvider, fullRobotModel,
               sensorInformation.getLidarParameters(), tfPublisher);
      }
   }

   private void setupRosLocalization()
   {
      new RosLocalizationUpdateSubscriber(rosMainNode, rosModulePacketCommunicator, ppsTimestampOffsetProvider);
      RosLocalizationServiceClient rosLocalizationServiceClient = new RosLocalizationServiceClient(rosMainNode);
      rosModulePacketCommunicator.attachListener(LocalizationPacket.class, rosLocalizationServiceClient);
   }

//   private void setupFootstepServiceClient()
//   {
//      RosFootstepServiceClient rosFootstepServiceClient = new RosFootstepServiceClient(rosModulePacketCommunicator, rosMainNode, physicalProperties.getAnkleHeight());
//      rosModulePacketCommunicator.attachListener(SnapFootstepPacket.class, rosFootstepServiceClient);
//   }
//
//   private void setupFootstepPathPlannerService()
//   {
//      FootstepPlanningParameterization footstepParameters = new BasicFootstepPlanningParameterization();
//      FootstepPathPlannerService footstepPathPlannerService;
////    footstepPathPlannerService = new AStarPathPlannerService(rosMainNode, footstepParameters, physicalProperties.getAnkleHeight(), fieldObjectCommunicator);
////    footstepPathPlannerService = new DStarPathPlannerService(rosMainNode, footstepParameters, physicalProperties.getAnkleHeight(), fieldObjectCommunicator);
//      footstepPathPlannerService = new ADStarPathPlannerService(rosMainNode, footstepParameters, physicalProperties.getAnkleHeight(), rosModulePacketCommunicator);
//      rosModulePacketCommunicator.attachListener(FootstepPlanRequestPacket.class, footstepPathPlannerService);
//   }

   private void printIfDebug(String str)
   {
      if(DEBUG)
      {
         System.out.println("[DEBUG] " + str);
      }
   }

   public PacketCommunicator getCommunicator()
   {
      return rosModulePacketCommunicator;
   }
   
}
