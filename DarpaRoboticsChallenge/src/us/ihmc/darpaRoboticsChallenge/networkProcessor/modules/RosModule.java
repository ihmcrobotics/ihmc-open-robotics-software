package us.ihmc.darpaRoboticsChallenge.networkProcessor.modules;

import java.net.URI;
import java.util.Arrays;
import java.util.List;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.FootstepParameters;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.AtomicSettableTimestampProvider;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.sensing.LocalizationPacket;
import us.ihmc.communication.packets.walking.FootstepPlanRequestPacket;
import us.ihmc.communication.packets.walking.SnapFootstepPacket;
import us.ihmc.communication.producers.RobotPoseBuffer;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.ros.RosRobotJointStatePublisher;
import us.ihmc.darpaRoboticsChallenge.ros.RosRobotPosePublisher;
import us.ihmc.darpaRoboticsChallenge.ros.RosSCSLidarPublisher;
import us.ihmc.darpaRoboticsChallenge.ros.RosTfPublisher;
import us.ihmc.ihmcPerception.RosLocalizationServiceClient;
import us.ihmc.ihmcPerception.RosLocalizationUpdateSubscriber;
import us.ihmc.pathGeneration.footstepPlanner.FootstepPathPlannerService;
import us.ihmc.pathGeneration.footstepPlanner.FootstepPlanningParameterization;
import us.ihmc.pathGeneration.footstepPlanner.RosFootstepServiceClient;
import us.ihmc.pathGeneration.footstepPlanner.aDStar.ADStarPathPlannerService;
import us.ihmc.pathGeneration.terrainAnalysis.BasicFootstepPlanningParameterization;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;


public class RosModule
{
   private static final String ROS_NAMESPACE = "networkProcessor/rosModule";

   private final KryoLocalPacketCommunicator rosModulePacketCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
         PacketDestination.ROS_MODULE.ordinal(), "RosModule");
   
   private final AtomicSettableTimestampProvider timestampProvider = new AtomicSettableTimestampProvider();
   private final RosMainNode rosMainNode;
   private final SDFFullRobotModel sdfFullRobotModel;
   private final RobotPoseBuffer robotPoseBuffer;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final DRCRobotSensorInformation sensorInformation;
   private final RobotDataReceiver robotDataReceiver;
   private final DRCRobotPhysicalProperties physicalProperties;

   
   
   public RosModule(DRCRobotModel robotModel, URI rosCoreURI, PacketCommunicator simulatedSensorCommunicator)
   {
      rosMainNode = new RosMainNode(rosCoreURI, ROS_NAMESPACE, true);
      
      physicalProperties = robotModel.getPhysicalProperties();
      robotPoseBuffer = new RobotPoseBuffer(rosModulePacketCommunicator, 10000, timestampProvider);
      sdfFullRobotModel = robotModel.createFullRobotModel();
      ppsTimestampOffsetProvider = robotModel.getPPSTimestampOffsetProvider();
      ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);
      sensorInformation = robotModel.getSensorInformation();
      
      List<ForceSensorDefinition> forceSensorDefinitions = Arrays.asList(sdfFullRobotModel.getForceSensorDefinitions());
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(forceSensorDefinitions);
      robotDataReceiver = new RobotDataReceiver(sdfFullRobotModel, forceSensorDataHolder);
      rosModulePacketCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);
      
      RosTfPublisher tfPublisher = new RosTfPublisher(rosMainNode);
      new RosRobotPosePublisher(rosModulePacketCommunicator, rosMainNode, ppsTimestampOffsetProvider, robotPoseBuffer, sensorInformation, "atlas", tfPublisher);
      new RosRobotJointStatePublisher(sdfFullRobotModel, rosModulePacketCommunicator, rosMainNode, ppsTimestampOffsetProvider,"atlas");
      
//      multiSenseSensorManager.setRobotPosePublisher(robotPosePublisher);
      
      if (sensorInformation.getLidarParameters().length > 0)
      {
//         new RosSCSLidarPublisher(simulatedSensorCommunicator, rosMainNode, ppsTimestampOffsetProvider, sdfFullRobotModel, sensorInformation.getLidarParameters(), tfPublisher);
      }
      
      if(sensorInformation.setupROSLocationService())
      {
         setupRosLocalization();
      }

//      setupFootstepServiceClient();
//      setupFootstepPathPlannerService();
      
      rosMainNode.execute();
      System.out.println("created ros module");
   }

   private void setupRosLocalization()
   {
      new RosLocalizationUpdateSubscriber(rosMainNode, rosModulePacketCommunicator, ppsTimestampOffsetProvider);
      RosLocalizationServiceClient rosLocalizationServiceClient = new RosLocalizationServiceClient(rosMainNode);
      rosModulePacketCommunicator.attachListener(LocalizationPacket.class, rosLocalizationServiceClient);
   }

   private void setupFootstepServiceClient()
   {
      RosFootstepServiceClient rosFootstepServiceClient = new RosFootstepServiceClient(rosModulePacketCommunicator, rosMainNode, physicalProperties.getAnkleHeight());
      rosModulePacketCommunicator.attachListener(SnapFootstepPacket.class, rosFootstepServiceClient);
   }

   private void setupFootstepPathPlannerService()
   {
      FootstepPlanningParameterization footstepParameters = new BasicFootstepPlanningParameterization();
      FootstepPathPlannerService footstepPathPlannerService;
//    footstepPathPlannerService = new AStarPathPlannerService(rosMainNode, footstepParameters, physicalProperties.getAnkleHeight(), fieldObjectCommunicator);
//    footstepPathPlannerService = new DStarPathPlannerService(rosMainNode, footstepParameters, physicalProperties.getAnkleHeight(), fieldObjectCommunicator);
      footstepPathPlannerService = new ADStarPathPlannerService(rosMainNode, footstepParameters, physicalProperties.getAnkleHeight(), rosModulePacketCommunicator);
      rosModulePacketCommunicator.attachListener(FootstepPlanRequestPacket.class, footstepPathPlannerService);
   }


   public PacketCommunicator getCommunicator()
   {
      return rosModulePacketCommunicator;
   }
   
}
