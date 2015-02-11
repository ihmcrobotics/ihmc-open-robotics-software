package us.ihmc.darpaRoboticsChallenge.networkProcessor.modules;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;


public class RosModule
{
   private final KryoLocalPacketCommunicator rosModulePacketCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
         PacketDestination.ROS_MODULE.ordinal(), "RosModule");
   
   public RosModule()
   {
    
//      RosMainNode rosMainNode;
//      RosTfPublisher tfPublisher;
//      if (DRCConfigParameters.SEND_ROBOT_DATA_TO_ROS)
//      {
//         RosTfPublisher tfPublisher = new RosTfPublisher(rosMainNode);
//         RosRobotPosePublisher robotPosePublisher = new RosRobotPosePublisher(sensorSuitePacketCommunicator, rosMainNode, ppsTimestampOffsetProvider, robotPoseBuffer, sensorInformation, "atlas", tfPublisher);
//         new RosLocalizationUpdateSubscriber(rosMainNode, sensorSuitePacketCommunicator, ppsTimestampOffsetProvider);
//         multiSenseSensorManager.setRobotPosePublisher(robotPosePublisher);
//         new RosRobotJointStatePublisher(sensorSuitePacketCommunicator, rosMainNode, ppsTimestampOffsetProvider,"atlas");
//      }
//      
//      if (sensorInformation.getLidarParameters().length > 0)
//      {
//        SCSLidarDataReceiver scsLidarDataReceiver = new SCSLidarDataReceiver(depthDataProcessor, robotPoseBuffer, sensorSuitePacketCommunicator, ppsTimestampOffsetProvider, sdfFullRobotModel,
//               sensorInformation.getLidarParameters());
//        Thread lidarThread = new Thread(scsLidarDataReceiver, "scsLidarDataReceiver");
//        lidarThread.start();
//         
//         if (DRCConfigParameters.SEND_ROBOT_DATA_TO_ROS)
//         {
//            new RosSCSLidarPublisher(sensorSuitePacketCommunicator, rosMainNode, ppsTimestampOffsetProvider, sdfFullRobotModel, sensorInformation.getLidarParameters(), tfPublisher);
//         }
//      }
//      
//      if (DRCConfigParameters.SEND_ROBOT_DATA_TO_ROS)
//      {
//         rosMainNode = new RosMainNode(rosCoreURI,
//               "darpaRoboticsChallange/networkProcessor", true);
//
//         RosNativeNetworkProcessor rosNativeNetworkProcessor;
//         if (RosNativeNetworkProcessor.hasNativeLibrary())
//         {
//            rosNativeNetworkProcessor = RosNativeNetworkProcessor.getInstance(rosCoreURI.toString());
//            rosNativeNetworkProcessor.connect();
//         } else
//         {
//            rosNativeNetworkProcessor = null;
//         }
//
//         ROSNativeTransformTools rosTransformProvider = ROSNativeTransformTools.getInstance(sensorURI);
//         rosTransformProvider.connect();
//         tfPublisher = new RosTfPublisher(rosMainNode);
//         new RosRobotPosePublisher(sensorSuitePacketCommunicator, rosMainNode, ppsTimestampOffsetProvider, robotPoseBuffer, sensorInformation, "atlas", tfPublisher);
//         new RosRobotJointStatePublisher(sensorSuitePacketCommunicator, rosMainNode, ppsTimestampOffsetProvider, "atlas");
//         new RosLocalizationUpdateSubscriber(rosMainNode, sensorSuitePacketCommunicator, ppsTimestampOffsetProvider);
//
//         
//         RosFootstepServiceClient rosFootstepServiceClient = new RosFootstepServiceClient(sensorSuitePacketCommunicator, rosMainNode, physicalProperties.getAnkleHeight());
//         sensorSuitePacketCommunicator.attachListener(SnapFootstepPacket.class, rosFootstepServiceClient);
//         RosLocalizationServiceClient rosLocalizationServiceClient = new RosLocalizationServiceClient(rosMainNode);
//         sensorSuitePacketCommunicator.attachListener(LocalizationPacket.class, rosLocalizationServiceClient);
//         
//         FootstepPathPlannerService footstepPathPlannerService;
////         footstepPathPlannerService = new AStarPathPlannerService(rosMainNode, footstepParameters, physicalProperties.getAnkleHeight(), fieldObjectCommunicator);
////         footstepPathPlannerService = new DStarPathPlannerService(rosMainNode, footstepParameters, physicalProperties.getAnkleHeight(), fieldObjectCommunicator);
//         footstepPathPlannerService = new ADStarPathPlannerService(rosMainNode, footstepParameters, physicalProperties.getAnkleHeight(), sensorSuitePacketCommunicator);
//         sensorSuitePacketCommunicator.attachListener(FootstepPlanRequestPacket.class, footstepPathPlannerService);
//         
//         
////       RosFootstepServiceClient rosFootstepServiceClient = new RosFootstepServiceClient(networkingManager, rosMainNode, physicalProperties);
////       networkingManager.getControllerCommandHandler().attachListener(SnapFootstepPacket.class, rosFootstepServiceClient);
//       
//       if(sensorInformation.setupROSLocationService())
//       {
//          RosLocalizationServiceClient rosLocalizationServiceClient = new RosLocalizationServiceClient(rosMainNode);
//          sensorSuitePacketCommunicator.attachListener(LocalizationPacket.class, rosLocalizationServiceClient);
//       }
//         
//         rosMainNode.execute();
//      }
   }

   public PacketCommunicator getCommunicator()
   {
      return rosModulePacketCommunicator;
   }
   
}
