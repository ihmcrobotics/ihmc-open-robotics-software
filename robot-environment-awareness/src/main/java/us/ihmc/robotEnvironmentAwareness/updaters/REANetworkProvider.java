package us.ihmc.robotEnvironmentAwareness.updaters;

import controller_msgs.msg.dds.*;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.NewMessageListener;


public interface REANetworkProvider
{
   void registerMessager(Messager messager);

   void registerLidarScanHandler(Messager messager, NewMessageListener<LidarScanMessage> lidarScanHandler);

   void registerDepthPointCloudHandler(Messager messager, NewMessageListener<StereoVisionPointCloudMessage> depthPointCloudHandler);

   void registerStereoVisionPointCloudHandler(Messager messager, NewMessageListener<StereoVisionPointCloudMessage> stereoVisionPointCloudHandler);

   void registerStampedPosePacketHandler(Messager messager, NewMessageListener<StampedPosePacket> stampedPosePacketHandler);

   void registerCustomRegionsHandler(NewMessageListener<PlanarRegionsListMessage> customRegionsHandler);

   void registerPlanarRegionsListRequestHandler(NewMessageListener<RequestPlanarRegionsListMessage> requestHandler);

   void registerREAStateRequestHandler(NewMessageListener<REAStateRequestMessage> requestHandler);

   void registerREASensorDataFilterParametersHandler(NewMessageListener<REASensorDataFilterParametersMessage> parametersHandler);

   void update(RegionFeaturesProvider regionFeaturesProvider, boolean planarRegionsHaveBeenUpdated);

   void publishCurrentState();

   void stop();
}
