package us.ihmc.robotEnvironmentAwareness.communication;

import controller_msgs.msg.dds.FootstepDataMessage;
import ihmc_common_msgs.msg.dds.StampedPosePacket;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.*;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoxMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMFrameState;
import us.ihmc.robotEnvironmentAwareness.slam.SurfaceElementICPSLAMParameters;
import us.ihmc.ros2.ROS2Topic;

public class SLAMModuleAPI
{
   public static final ROS2Topic<Empty> CLEAR = PerceptionAPI.REALSENSE_SLAM_MODULE.withInput().withType(Empty.class).withSuffix("clear");
   public static final ROS2Topic<Empty> SHUTDOWN = PerceptionAPI.REALSENSE_SLAM_MODULE.withType(Empty.class).withSuffix("shutdown");

   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("SLAM"));
   private static final CategoryTheme SLAMModule = apiFactory.createCategoryTheme("SLAMModule");

   private static final CategoryTheme Module = apiFactory.createCategoryTheme("Module");
   private static final CategoryTheme UI = apiFactory.createCategoryTheme("UI");
   private static final CategoryTheme DepthCloud = apiFactory.createCategoryTheme("DepthCloud");
   private static final CategoryTheme SensorFrame = apiFactory.createCategoryTheme("SensorFrame");
   private static final CategoryTheme VelocityLimit = apiFactory.createCategoryTheme("VelocityLimit");
   private static final CategoryTheme OcTree = apiFactory.createCategoryTheme("OcTree");
   private static final CategoryTheme Normal = apiFactory.createCategoryTheme("Normal");
   private static final CategoryTheme Buffer = apiFactory.createCategoryTheme("Buffer");
   private static final CategoryTheme Custom = apiFactory.createCategoryTheme("Custom");
   private static final CategoryTheme Footstep = apiFactory.createCategoryTheme("Footstep");
   private static final CategoryTheme DataManager = apiFactory.createCategoryTheme("DataManager");
   private static final CategoryTheme Export = apiFactory.createCategoryTheme("Export");

   private static final TopicTheme Parameters = apiFactory.createTopicTheme("Parameters");
   private static final TopicTheme Data = apiFactory.createTopicTheme("Data");

   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");
   private static final TypedTopicTheme<Boolean> Request = apiFactory.createTypedTopicTheme("Request");
   private static final TypedTopicTheme<Boolean> Clear = apiFactory.createTypedTopicTheme("Clear");
   private static final TypedTopicTheme<Boolean> Show = apiFactory.createTypedTopicTheme("Show");
   private static final TypedTopicTheme<Boolean> Moving = apiFactory.createTypedTopicTheme("Moving");
   private static final TypedTopicTheme<Integer> Size = apiFactory.createTypedTopicTheme("Size");
   private static final TypedTopicTheme<String> Status = apiFactory.createTypedTopicTheme("Status");
   private static final TypedTopicTheme<Double> Value = apiFactory.createTypedTopicTheme("Value");
   private static final TypedTopicTheme<Boolean> Save = apiFactory.createTypedTopicTheme("Save");

   private static final TypedTopicTheme<String> Path = apiFactory.createTypedTopicTheme("Path");

   public static final Topic<Boolean> SaveConfiguration = Root.child(Export).topic(Save);

   public static final Topic<Boolean> RequestEntireModuleState = Root.child(Module).topic(Request);

   public static final Topic<Boolean> SLAMEnable = Root.child(Module).topic(Enable);
   public static final Topic<Boolean> SLAMClear = Root.child(Module).topic(Clear);

   public static final Topic<Boolean> ShowSLAMOctreeMap = Root.child(UI).child(OcTree).topic(Show);
   public static final Topic<Boolean> ShowSLAMOctreeNormalMap = Root.child(UI).child(OcTree).child(Normal).topic(Show);
   public static final Topic<Boolean> ShowLatestFrame = Root.child(UI).child(DepthCloud).topic(Show);
   public static final Topic<Boolean> ShowSLAMSensorTrajectory = Root.child(UI).child(SensorFrame).topic(Show);
   public static final Topic<Boolean> SLAMVizClear = Root.child(UI).topic(Clear);
   public static final Topic<Boolean> SensorPoseHistoryClear = Root.child(UI).child(SensorFrame).topic(Clear);

   public static final Topic<Boolean> ShowFootstepDataViz = Root.child(UI).child(Footstep).topic(Enable);
   public static final Topic<Boolean> ClearFootstepDataViz = Root.child(UI).child(Footstep).topic(Clear);
   public static final Topic<FootstepDataMessage> FootstepDataState = Root.child(UI).child(Footstep).topic(Data);

   public static final Topic<Boolean> SensorStatus = Root.child(Module).child(SensorFrame).topic(Moving);
   public static final Topic<String> SensorSpeed = topic("SensorSpeed");
   public static final Topic<Boolean> VelocityLimitStatus = Root.child(Module).child(VelocityLimit).topic(Moving);

   public static final Topic<SurfaceElementICPSLAMParameters> SLAMParameters = Root.child(Module).topic(Parameters);

   public static final Topic<String> FrameComputationTime = Root.child(Module).topic(Status);
   public static final Topic<String> SLAMComputationTime = topic("SLAMComputationTime");
   public static final Topic<String> AverageComputationTime = topic("AverageComputationTime");
   public static final Topic<String> ListenerComputationTime = topic("ListenerComputationTime");
   public static final Topic<String> TotalComputationTime = topic("TotalComputationTime");
   public static final Topic<String> QueuedBuffers = Root.child(Module).child(Buffer).topic(Status);

   public static final Topic<Boolean> NormalEstimationClear = Root.child(Normal).topic(Clear);
   public static final Topic<Boolean> NormalEstimationEnable = Root.child(Normal).topic(Enable);
   public static final Topic<NormalEstimationParameters> NormalEstimationParameters = topic("NormalEstimationParameters");
   public static final Topic<NormalEstimationParameters> FrameNormalEstimationParameters = topic("FrameNormalEstimationParameters");

   public static final Topic<StereoVisionPointCloudMessage> DepthPointCloudState = Root.child(UI).child(DepthCloud).topic(Data);
   public static final Topic<SLAMFrameState> IhmcSLAMFrameState = Root.child(UI).child(Buffer).topic(Data);
   public static final Topic<NormalOcTreeMessage> SLAMOctreeMapState = Root.child(UI).child(OcTree).topic(Data);
   public static final Topic<Integer> UISensorPoseHistoryFrames = Root.child(UI).child(SensorFrame).topic(Size);
   public static final Topic<StampedPosePacket> CustomizedFrameState = Root.child(UI).child(Custom).topic(Data);
   public static final Topic<Double> LatestFrameConfidenceFactor = Root.child(UI).child(SensorFrame).topic(Value);

   public static final Topic<Boolean> UIOcTreeBoundingBoxShow = topic("UIOcTreeBoundingBoxShow");
   public static final Topic<Boolean> OcTreeBoundingBoxEnable = topic("OcTreeBoundingBoxEnable");
   public static final Topic<Boolean> RequestBoundingBox = topic("RequestBoundingBox");
   public static final Topic<BoundingBoxParametersMessage> OcTreeBoundingBoxParameters = topic("OcTreeBoundingBoxParameters");
   public static final Topic<BoxMessage> OcTreeBoundingBoxState = topic("OcTreeBoundingBoxState");
   
   public static final Topic<StereoVisionPointCloudMessage> UIStereoSLAMPointCloud = topic("UIUncorrectedPointCloud");

   public static final Topic<Boolean> UIRawDataExportRequest = Root.child(UI).child(DataManager).child(DepthCloud).child(Export).topic(Request);
   public static final Topic<Boolean> UISLAMDataExportRequest = Root.child(UI).child(DataManager).child(Module).child(Export).topic(Request);
   
   public static final Topic<String> UIRawDataExportDirectory = Root.child(UI).child(DataManager).child(DepthCloud).child(Export).topic(Path);
   public static final Topic<String> UISLAMDataExportDirectory = Root.child(UI).child(DataManager).child(Module).child(Export).topic(Path);
   
   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static <T> Topic<T> topic(String name)
   {
      return Root.child(SLAMModule).topic(apiFactory.createTypedTopicTheme(name));
   }
}
