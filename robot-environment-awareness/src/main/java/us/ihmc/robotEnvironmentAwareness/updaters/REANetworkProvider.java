package us.ihmc.robotEnvironmentAwareness.updaters;

import controller_msgs.msg.dds.*;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.NewMessageListener;


public interface REANetworkProvider
{
   void registerMessager(Messager messager);

   default void registerLidarScanHandler(NewMessageListener<LidarScanMessage> lidarScanHandler)
   {
   }

   default void registerStereoVisionPointCloudHandler(NewMessageListener<StereoVisionPointCloudMessage> stereoVisionPointCloudHandler)
   {
   }

   default void registerLidarScanHandler(Messager messager, NewMessageListener<LidarScanMessage> lidarScanHandler)
   {
   }

   default void registerDepthPointCloudHandler(Messager messager, NewMessageListener<StereoVisionPointCloudMessage> depthPointCloudHandler)
   {
   }

   default void registerStereoVisionPointCloudHandler(Messager messager, NewMessageListener<StereoVisionPointCloudMessage> stereoVisionPointCloudHandler)
   {
   }

   default void registerStampedPosePacketHandler(Messager messager, NewMessageListener<StampedPosePacket> stampedPosePacketHandler)
   {
   }

   default void registerCustomRegionsHandler(NewMessageListener<PlanarRegionsListMessage> customRegionsHandler)
   {
   }

   default void registerPlanarRegionsListRequestHandler(NewMessageListener<RequestPlanarRegionsListMessage> requestHandler)
   {
   }

   default void registerREAStateRequestHandler(NewMessageListener<REAStateRequestMessage> requestHandler)
   {
   }

   default void registerREASensorDataFilterParametersHandler(NewMessageListener<REASensorDataFilterParametersMessage> parametersHandler)
   {
   }

   void update(RegionFeaturesProvider regionFeaturesProvider, boolean planarRegionsHaveBeenUpdated);

   void publishCurrentState();

   void stop();
}
