package us.ihmc.robotEnvironmentAwareness.communication;

import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TopicTheme;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoxMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.LineSegment3DMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomRegionMergeParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.IntersectionEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.SurfaceNormalFilterParameters;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.ColoringType;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.DisplayType;
import us.ihmc.robotEnvironmentAwareness.updaters.REAOcTreeBuffer.BufferType;

public class REAModuleAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final CategoryTheme Module = apiFactory.createCategoryTheme("Module");
   private static final CategoryTheme UI = apiFactory.createCategoryTheme("UserInterface");

   private static final CategoryTheme OcTree = apiFactory.createCategoryTheme("OcTree");
   private static final CategoryTheme Lidar = apiFactory.createCategoryTheme("Lidar");
   private static final CategoryTheme StereoVision = apiFactory.createCategoryTheme("StereoVision");
   private static final CategoryTheme BoundingBox = apiFactory.createCategoryTheme("BoundingBox");
   private static final CategoryTheme NormalEstimation = apiFactory.createCategoryTheme("NormalEstimation");
   private static final CategoryTheme PlanarRegions = apiFactory.createCategoryTheme("PlanarRegions");
   private static final CategoryTheme Segmentation = apiFactory.createCategoryTheme("Segmentation");
   private static final CategoryTheme Custom = apiFactory.createCategoryTheme("Custom");
   private static final CategoryTheme Intersection = apiFactory.createCategoryTheme("Intersection");
   private static final CategoryTheme ConcaveHull = apiFactory.createCategoryTheme("ConcaveHull");
   private static final CategoryTheme Polygonizer = apiFactory.createCategoryTheme("Polygonizer");
   private static final CategoryTheme Buffer = apiFactory.createCategoryTheme("Buffer");
   private static final CategoryTheme Range = apiFactory.createCategoryTheme("Range");
   private static final CategoryTheme Node = apiFactory.createCategoryTheme("Node");
   private static final CategoryTheme Request = apiFactory.createCategoryTheme("Request");
   private static final CategoryTheme DataExporter = apiFactory.createCategoryTheme("DataExporter");
   private static final CategoryTheme Message = apiFactory.createCategoryTheme("Message");
   private static final CategoryTheme Refreshing = apiFactory.createCategoryTheme("Refreshing");
   private static final CategoryTheme SurfaceNormal = apiFactory.createCategoryTheme("SurfaceNormal");

   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");
   private static final TypedTopicTheme<Boolean> Clear = apiFactory.createTypedTopicTheme("Clear");
   private static final TypedTopicTheme<Boolean> Show = apiFactory.createTypedTopicTheme("Show");
   private static final TypedTopicTheme<Boolean> Hide = apiFactory.createTypedTopicTheme("Hide");
   private static final TypedTopicTheme<Boolean> Save = apiFactory.createTypedTopicTheme("Save");
   private static final TypedTopicTheme<Boolean> Export = apiFactory.createTypedTopicTheme("Export");
   private static final TypedTopicTheme<Integer> Size = apiFactory.createTypedTopicTheme("Size");
   private static final TypedTopicTheme<Integer> Depth = apiFactory.createTypedTopicTheme("Depth");
   private static final TypedTopicTheme<String> Path = apiFactory.createTypedTopicTheme("Path");
   private static final TypedTopicTheme<Integer> Capacity = apiFactory.createTypedTopicTheme("Capacity");

   private static final TopicTheme Parameters = apiFactory.createTopicTheme("Parameters");
   private static final TopicTheme Min = apiFactory.createTopicTheme("Min");
   private static final TopicTheme Max = apiFactory.createTopicTheme("Max");
   private static final TopicTheme Data = apiFactory.createTopicTheme("Data");
   private static final TopicTheme Color = apiFactory.createTopicTheme("Color");
   private static final TopicTheme Display = apiFactory.createTopicTheme("Display");
   private static final TopicTheme Type = apiFactory.createTopicTheme("Type");

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("REA"));

   private static final Category ModuleCategory = Root.child(Module);
   private static final Category OcTreeCategory = ModuleCategory.child(OcTree);
   private static final Category PlanarRegionsCategory = ModuleCategory.child(PlanarRegions);

   public static final Topic<Boolean> OcTreeEnable = OcTreeCategory.topic(Enable);
   public static final Topic<Boolean> OcTreeClear = OcTreeCategory.topic(Clear);
   public static final Topic<Integer> OcTreeDepth = OcTreeCategory.topic(Depth);
   public static final Topic<Boolean> LidarBufferEnable = OcTreeCategory.child(Lidar).child(Buffer).topic(Enable);
   public static final Topic<Integer> LidarBufferOcTreeCapacity = OcTreeCategory.child(Lidar).child(Buffer).child(OcTree).topic(Capacity);
   public static final Topic<Integer> LidarBufferMessageCapacity = OcTreeCategory.child(Lidar).child(Buffer).child(Message).topic(Capacity);
   public static final Topic<Boolean> StereoVisionBufferEnable = OcTreeCategory.child(StereoVision).child(Buffer).topic(Enable);
   public static final Topic<Boolean> StereoVisionBufferRefreshingEnable = OcTreeCategory.child(StereoVision).child(Buffer).child(Refreshing).topic(Enable);
   public static final Topic<Integer> StereoVisionBufferOcTreeCapacity = OcTreeCategory.child(StereoVision).child(Buffer).child(OcTree).topic(Capacity);
   public static final Topic<Integer> StereoVisionBufferSize = OcTreeCategory.child(StereoVision).child(Buffer).topic(Size);
   public static final Topic<Integer> StereoVisionBufferMessageCapacity = OcTreeCategory.child(StereoVision).child(Buffer).child(Message).topic(Capacity);
   public static final Topic<Boolean> OcTreeBoundingBoxEnable = OcTreeCategory.child(BoundingBox).topic(Enable);
   public static final Topic<BoundingBoxParametersMessage> OcTreeBoundingBoxParameters = OcTreeCategory.child(BoundingBox).topic(Parameters);

   public static final Topic<Boolean> NormalEstimationEnable = ModuleCategory.child(NormalEstimation).topic(Enable);
   public static final Topic<Boolean> NormalEstimationClear = ModuleCategory.child(NormalEstimation).topic(Clear);
   public static final Topic<NormalEstimationParameters> NormalEstimationParameters = ModuleCategory.child(NormalEstimation).topic(Parameters);

   public static final Topic<Double> LidarMinRange = ModuleCategory.child(Lidar).child(Range).topic(Min);
   public static final Topic<Double> LidarMaxRange = ModuleCategory.child(Lidar).child(Range).topic(Max);

   public static final Topic<Boolean> PlanarRegionsSegmentationEnable = PlanarRegionsCategory.child(Segmentation).topic(Enable);
   public static final Topic<Boolean> PlanarRegionsSegmentationClear = PlanarRegionsCategory.child(Segmentation).topic(Clear);
   public static final Topic<PlanarRegionSegmentationParameters> PlanarRegionsSegmentationParameters = PlanarRegionsCategory.child(Segmentation)
                                                                                                                            .topic(Parameters);
   public static final Topic<Boolean> CustomRegionsMergingEnable = PlanarRegionsCategory.child(Custom).topic(Enable);
   public static final Topic<Boolean> CustomRegionsClear = PlanarRegionsCategory.child(Custom).topic(Clear);
   public static final Topic<CustomRegionMergeParameters> CustomRegionsMergingParameters = PlanarRegionsCategory.child(Custom).topic(Parameters);
   public static final Topic<Boolean> PlanarRegionsPolygonizerEnable = PlanarRegionsCategory.child(Polygonizer).topic(Enable);
   public static final Topic<Boolean> PlanarRegionsPolygonizerClear = PlanarRegionsCategory.child(Polygonizer).topic(Clear);
   public static final Topic<ConcaveHullFactoryParameters> PlanarRegionsConcaveHullParameters = PlanarRegionsCategory.child(ConcaveHull).topic(Parameters);
   public static final Topic<PolygonizerParameters> PlanarRegionsPolygonizerParameters = PlanarRegionsCategory.child(Polygonizer).topic(Parameters);
   public static final Topic<Boolean> PlanarRegionsIntersectionEnable = PlanarRegionsCategory.child(Intersection).topic(Enable);
   public static final Topic<IntersectionEstimationParameters> PlanarRegionsIntersectionParameters = PlanarRegionsCategory.child(Intersection)
                                                                                                                          .topic(Parameters);
   public static final Topic<SurfaceNormalFilterParameters> SurfaceNormalFilterParameters = OcTreeCategory.child(SurfaceNormal).topic(Parameters);

   public static final Topic<Integer> UIOcTreeDepth = Root.child(UI).child(OcTree).topic(Depth);
   public static final Topic<ColoringType> UIOcTreeColoringMode = Root.child(UI).child(OcTree).topic(Color);
   public static final Topic<DisplayType> UIOcTreeDisplayType = Root.child(UI).child(OcTree).topic(Display);
   public static final Topic<BufferType> UIOcTreeBufferType = Root.child(UI).child(OcTree).child(Buffer).topic(Type);
   public static final Topic<Boolean> UIPlanarRegionHideNodes = Root.child(UI).child(PlanarRegions).child(Node).topic(Hide);
   public static final Topic<Boolean> UIOcTreeBoundingBoxShow = Root.child(UI).child(OcTree).child(BoundingBox).topic(Show);
   public static final Topic<Boolean> UIOcTreeShowLidarBuffer = Root.child(UI).child(OcTree).child(Lidar).child(Buffer).topic(Show);
   public static final Topic<Boolean> UIOcTreeShowStereoVisionBuffer = Root.child(UI).child(OcTree).child(StereoVision).child(Buffer).topic(Show);
   public static final Topic<Boolean> UILidarScanShow = Root.child(UI).child(Lidar).topic(Show);
   public static final Topic<Boolean> UILidarScanClear = Root.child(UI).child(Lidar).topic(Clear);
   public static final Topic<Integer> UILidarScanSize = Root.child(UI).child(Lidar).topic(Size);
   public static final Topic<Boolean> UIStereoVisionShow = Root.child(UI).child(StereoVision).topic(Show);
   public static final Topic<Boolean> UIStereoVisionClear = Root.child(UI).child(StereoVision).topic(Clear);
   public static final Topic<Integer> UIStereoVisionSize = Root.child(UI).child(StereoVision).topic(Size);

   public static final Topic<Boolean> UISegmentationDataExportRequest = Root.child(UI).child(DataExporter).child(Segmentation).topic(Export);
   public static final Topic<String> UISegmentationDataExporterDirectory = Root.child(UI).child(DataExporter).child(Segmentation).topic(Path);
   public static final Topic<Boolean> UIPlanarRegionDataExportRequest = Root.child(UI).child(DataExporter).child(PlanarRegions).topic(Export);
   public static final Topic<String> UIPlanarRegionDataExporterDirectory = Root.child(UI).child(DataExporter).child(PlanarRegions).topic(Path);

   public static final Topic<LidarScanMessage> LidarScanState = ModuleCategory.child(Lidar).topic(Data);
   public static final Topic<StereoVisionPointCloudMessage> StereoVisionPointCloudState = ModuleCategory.child(StereoVision).topic(Data);
   public static final Topic<NormalOcTreeMessage> OcTreeState = OcTreeCategory.topic(Data);
   public static final Topic<NormalOcTreeMessage> LidarBufferState = OcTreeCategory.child(Lidar).child(Buffer).topic(Data);
   public static final Topic<NormalOcTreeMessage> StereoVisionBufferState = OcTreeCategory.child(StereoVision).child(Buffer).topic(Data);
   public static final Topic<PlanarRegionsListMessage> PlanarRegionsState = PlanarRegionsCategory.topic(Data);
   public static final Topic<PlanarRegionSegmentationMessage[]> PlanarRegionsSegmentationState = PlanarRegionsCategory.child(Segmentation).topic(Data);
   public static final Topic<LineSegment3DMessage[]> PlanarRegionsIntersectionState = PlanarRegionsCategory.child(Intersection).topic(Data);
   public static final Topic<BoxMessage> OcTreeBoundingBoxState = OcTreeCategory.child(BoundingBox).topic(Data);

   public static final Topic<Boolean> RequestEntireModuleState = ModuleCategory.child(Request).topic(Data);
   public static final Topic<Boolean> RequestOctree = OcTreeCategory.child(Request).topic(Data);
   public static final Topic<Boolean> RequestLidarBuffer = OcTreeCategory.child(Lidar).child(Buffer).child(Request).topic(Data);
   public static final Topic<Boolean> RequestStereoVisionBuffer = OcTreeCategory.child(StereoVision).child(Buffer).child(Request).topic(Data);
   public static final Topic<Boolean> RequestPlanarRegions = PlanarRegionsCategory.child(Request).topic(Data);
   public static final Topic<Boolean> RequestPlanarRegionsIntersections = PlanarRegionsCategory.child(Intersection).child(Request).topic(Data);
   public static final Topic<Boolean> RequestPlanarRegionSegmentation = PlanarRegionsCategory.child(Request).child(Segmentation).topic(Data);
   public static final Topic<Boolean> RequestBoundingBox = OcTreeCategory.child(BoundingBox).child(Request).topic(Data);

   public static final Topic<Boolean> SaveMainUpdaterConfiguration = OcTreeCategory.topic(Save);
   public static final Topic<Boolean> SaveBufferConfiguration = OcTreeCategory.child(Buffer).topic(Save);
   public static final Topic<Boolean> SaveRegionUpdaterConfiguration = PlanarRegionsCategory.topic(Save);

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}
