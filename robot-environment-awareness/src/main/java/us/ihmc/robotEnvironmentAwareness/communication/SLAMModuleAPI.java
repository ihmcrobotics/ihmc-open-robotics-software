package us.ihmc.robotEnvironmentAwareness.communication;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.StampedPosePacket;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TopicTheme;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.slam.RandomICPSLAMParameters;

public class SLAMModuleAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("SLAM"));

   private static final CategoryTheme Module = apiFactory.createCategoryTheme("Module");
   private static final CategoryTheme UI = apiFactory.createCategoryTheme("UI");
   private static final CategoryTheme StereoVision = apiFactory.createCategoryTheme("StereoVision");
   private static final CategoryTheme DepthCloud = apiFactory.createCategoryTheme("DepthCloud");
   private static final CategoryTheme SensorFrame = apiFactory.createCategoryTheme("SensorPose");
   private static final CategoryTheme PlanarRegions = apiFactory.createCategoryTheme("PlanarRegions");
   private static final CategoryTheme OcTree = apiFactory.createCategoryTheme("OcTree");
   private static final CategoryTheme Buffer = apiFactory.createCategoryTheme("Buffer");
   private static final CategoryTheme Custom = apiFactory.createCategoryTheme("Custom");

   private static final TopicTheme Parameters = apiFactory.createTopicTheme("Parameters");
   private static final TopicTheme Data = apiFactory.createTopicTheme("Data");
   
   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");
   private static final TypedTopicTheme<Boolean> Clear = apiFactory.createTypedTopicTheme("Clear");
   //private static final TypedTopicTheme<Boolean> Show = apiFactory.createTypedTopicTheme("Show");
   private static final TypedTopicTheme<Boolean> Moving = apiFactory.createTypedTopicTheme("Moving");
   private static final TypedTopicTheme<Integer> Size = apiFactory.createTypedTopicTheme("Size");
   private static final TypedTopicTheme<String> Status = apiFactory.createTypedTopicTheme("Status");
   
   public static final Topic<Boolean> SLAMEnable = Root.child(Module).topic(Enable);
   public static final Topic<Boolean> SLAMClear = Root.child(Module).topic(Clear);
   public static final Topic<PlanarRegionsListMessage> SLAMPlanarRegionsState = Root.child(Module).child(PlanarRegions).topic(Data);
   public static final Topic<RandomICPSLAMParameters> SLAMParameters = Root.child(Module).topic(Parameters);
   public static final Topic<Boolean> SensorStatus = Root.child(Module).child(SensorFrame).topic(Moving);
   
   public static final Topic<String> SLAMStatus = Root.child(Module).topic(Status);
   public static final Topic<String> QueuedBuffers = Root.child(Module).child(Buffer).topic(Status);

   public static final Topic<StereoVisionPointCloudMessage> StereoVisionPointCloudState = Root.child(UI).child(StereoVision).topic(Data);
   public static final Topic<StereoVisionPointCloudMessage> DepthPointCloudState = Root.child(UI).child(DepthCloud).topic(Data);
   public static final Topic<StereoVisionPointCloudMessage> IhmcSLAMFrameState = Root.child(UI).child(Buffer).topic(Data);
   public static final Topic<NormalOcTreeMessage> SLAMOctreeMapState = Root.child(UI).child(OcTree).topic(Data);
   public static final Topic<Integer> UISensorPoseHistoryFrames = Root.child(UI).child(SensorFrame).topic(Size);
   public static final Topic<StampedPosePacket> CustomizedFrameState = Root.child(UI).child(Custom).topic(Data);
   
   
   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}
