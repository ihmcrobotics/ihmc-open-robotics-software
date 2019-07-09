package us.ihmc.footstepPlanning.communication;

import java.util.List;

import controller_msgs.msg.dds.*;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlannerStatus;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TopicTheme;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapWithNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepPlannerMessagerAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final CategoryTheme PlanarRegion = apiFactory.createCategoryTheme("PlanarRegion");
   private static final CategoryTheme Start = apiFactory.createCategoryTheme("Start");
   private static final CategoryTheme Intermediate = apiFactory.createCategoryTheme("Intermediate");
   private static final CategoryTheme Goal = apiFactory.createCategoryTheme("Goal");
   private static final CategoryTheme Any = apiFactory.createCategoryTheme("Any");
   private static final CategoryTheme SelectedRegion = apiFactory.createCategoryTheme("SelectedRegion");
   private static final CategoryTheme PositionTheme = apiFactory.createCategoryTheme("PositionTheme");
   private static final CategoryTheme OrientationTheme = apiFactory.createCategoryTheme("OrientationTheme");
   private static final CategoryTheme EditMode = apiFactory.createCategoryTheme("EditMode");
   private static final CategoryTheme FootstepPlan = apiFactory.createCategoryTheme("FootstepPlan");
   private static final CategoryTheme GoHome = apiFactory.createCategoryTheme("GoHome");
   private static final CategoryTheme BodyPath = apiFactory.createCategoryTheme("BodyPath");
   private static final CategoryTheme VisibilityGraphs = apiFactory.createCategoryTheme("VisibilityGraphs");
   private static final CategoryTheme NodeChecking = apiFactory.createCategoryTheme("NodeChecking");
   private static final CategoryTheme UnitTest = apiFactory.createCategoryTheme("UnitTest");
   private static final CategoryTheme Cluster = apiFactory.createCategoryTheme("Cluster");
   private static final CategoryTheme Map = apiFactory.createCategoryTheme("Map");
   private static final CategoryTheme InterRegion = apiFactory.createCategoryTheme("InterRegion");
   private static final CategoryTheme Statistics = apiFactory.createCategoryTheme("Statistics");
   private static final CategoryTheme Raw = apiFactory.createCategoryTheme("Raw");
   private static final CategoryTheme Navigable = apiFactory.createCategoryTheme("Navigable");
   private static final CategoryTheme NonNavigable = apiFactory.createCategoryTheme("NonNavigable");
   private static final CategoryTheme OccupancyMap = apiFactory.createCategoryTheme("OccupancyMap");
   private static final CategoryTheme PlannerData = apiFactory.createCategoryTheme("PlannerData");
   private static final CategoryTheme FlatGround = apiFactory.createCategoryTheme("FlatGround");
   private static final CategoryTheme Preview = apiFactory.createCategoryTheme("Preview");

   private static final CategoryTheme Parameters = apiFactory.createCategoryTheme("Parameters");

   private static final TypedTopicTheme<Boolean> Show = apiFactory.createTypedTopicTheme("Show");
   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");
   private static final TypedTopicTheme<Boolean> Reset = apiFactory.createTypedTopicTheme("Reset");
   private static final TypedTopicTheme<Point3D> Position = apiFactory.createTypedTopicTheme("Position");
   private static final TypedTopicTheme<RobotSide> Side = apiFactory.createTypedTopicTheme("Side");
   private static final TypedTopicTheme<Quaternion> Orientation = apiFactory.createTypedTopicTheme("Orientation");
   private static final TypedTopicTheme<Boolean> ComputePath = apiFactory.createTypedTopicTheme("ComputePath");
   private static final TypedTopicTheme<Boolean> AbortPlanning = apiFactory.createTypedTopicTheme("AbortPlanning");
   private static final TypedTopicTheme<Double> PlannerTimeout = apiFactory.createTypedTopicTheme("PlannerTimeout");
   private static final TypedTopicTheme<Double> PlannerTimeTaken = apiFactory.createTypedTopicTheme("PlannerTimeTaken");
   private static final TypedTopicTheme<Double> PlannerHorizonLength = apiFactory.createTypedTopicTheme("PlannerHorizonLength");
   private static final TypedTopicTheme<Integer> PlannerRequestId = apiFactory.createTypedTopicTheme("PlannerRequestId");
   private static final TypedTopicTheme<Integer> ReceivedPlanId = apiFactory.createTypedTopicTheme("ReceivedPlanId");
   private static final TypedTopicTheme<FootstepPlannerType> FootstepPlannerType = apiFactory.createTypedTopicTheme("FootstepPlannerType");
   private static final TypedTopicTheme<FootstepPlanningResult> FootstepPlannerResult = apiFactory.createTypedTopicTheme("FootstepPlannerResult");
   private static final TypedTopicTheme<FootstepPlannerStatus> FootstepPlannerStatus = apiFactory.createTypedTopicTheme("FootstepPlannerStatus");
   private static final TypedTopicTheme<FootstepPlannerParameters> FootstepPlannerParameters = apiFactory.createTypedTopicTheme("FootstepPlannerParameters");
   private static final TypedTopicTheme<FootstepDataListMessage> FootstepDataListMessage = apiFactory.createTypedTopicTheme("FootstepDataListMessage");
   private static final TypedTopicTheme<GoHomeMessage> GoHomeMessage = apiFactory.createTypedTopicTheme("GoHomeMessage");
   private static final TypedTopicTheme<Boolean> IgnorePartialFootholds = apiFactory.createTypedTopicTheme("IgnorePartialFootholds");
   private static final TypedTopicTheme<Boolean> RenderShiftedWaypoints = apiFactory.createTypedTopicTheme("RenderShiftedWaypoints");
   private static final TypedTopicTheme<FootstepPlanningStatistics> PlannerStatistics = apiFactory.createTypedTopicTheme("PlannerStatistics");

   private static final TypedTopicTheme<WalkingControllerPreviewInputMessage> PreviewRequest = apiFactory.createTypedTopicTheme("WalkingPreviewInput");
   private static final TypedTopicTheme<WalkingControllerPreviewOutputMessage> PreviewResponse = apiFactory.createTypedTopicTheme("WalkingPreviewOutput");

   private static final TypedTopicTheme<VisibilityGraphsParameters> VisibilityGraphsParameters = apiFactory.createTypedTopicTheme("VisibilityGraphsParameters");
   private static final TypedTopicTheme<Boolean> Export = apiFactory.createTypedTopicTheme("Export");

   private static final TypedTopicTheme<String> Path = apiFactory.createTypedTopicTheme("Path");
   private static final TypedTopicTheme<FootstepNodeDataListMessage> NodeData = apiFactory.createTypedTopicTheme("NodeData");
   private static final TypedTopicTheme<FootstepPlannerOccupancyMapMessage> OccupancyMapData = apiFactory.createTypedTopicTheme("OccupancyMapData");

   private static final TypedTopicTheme<BipedalSupportPlanarRegionParametersMessage> BipedalSupportRegionsParameters = apiFactory.createTypedTopicTheme("bipedalSupportRegionParameters");

   private static final TopicTheme Data = apiFactory.createTopicTheme("Data");
   private static final TopicTheme RobotConfigurationData = apiFactory.createTopicTheme("RobotConfigurationData");

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("FootstepPlanning"));

   public static final Topic<RobotConfigurationData> RobotConfigurationDataTopic = Root.topic(RobotConfigurationData);
   public static final Topic<PlanarRegionsList> PlanarRegionDataTopic = Root.child(PlanarRegion).topic(Data);
   public static final Topic<Boolean> ShowPlanarRegionsTopic = Root.child(PlanarRegion).topic(Show);
   public static final Topic<Boolean> AcceptNewPlanarRegions = Root.child(PlanarRegion).topic(Enable);
   public static final Topic<PlanarRegion> SelectedRegionTopic = Root.child(SelectedRegion).topic(Data);

   public static final Topic<FootstepDataListMessage> FootstepPlanResponseTopic = Root.child(FootstepPlan).topic(Data);
   public static final Topic<FootstepDataListMessage> FootstepPlanToRobotTopic = Root.child(FootstepPlan).topic(FootstepDataListMessage);
   public static final Topic<Boolean> ShowFootstepPlanTopic = Root.child(FootstepPlan).topic(Show);

   public static final Topic<Boolean> ComputePathTopic = Root.child(FootstepPlan).topic(ComputePath);
   public static final Topic<Boolean> AbortPlanningTopic = Root.child(FootstepPlan).topic(AbortPlanning);
   public static final Topic<Boolean> RequestPlannerStatistics = Root.child(Statistics).topic(Show);
   public static final Topic<Boolean> AssumeFlatGround = Root.child(FlatGround).topic(Enable);
   public static final Topic<FootstepPlannerParameters> PlannerParametersTopic = Root.child(Parameters).topic(FootstepPlannerParameters);

   public static final Topic<GoHomeMessage> GoHomeTopic = Root.child(GoHome).topic(GoHomeMessage);
   public static final Topic<Boolean> IgnorePartialFootholdsTopic = Root.topic(IgnorePartialFootholds);

   public static final Topic<VisibilityGraphsParameters> VisibilityGraphsParametersTopic = Root.child(Parameters).topic(VisibilityGraphsParameters);
   public static final Topic<Double> PlannerTimeoutTopic = Root.child(FootstepPlan).topic(PlannerTimeout);
   public static final Topic<Double> PlannerTimeTakenTopic = Root.child(FootstepPlan).topic(PlannerTimeTaken);
   public static final Topic<Double> PlannerHorizonLengthTopic = Root.child(FootstepPlan).topic(PlannerHorizonLength);
   public static final Topic<FootstepPlannerType> PlannerTypeTopic = Root.child(FootstepPlan).topic(FootstepPlannerType);
   public static final Topic<FootstepPlanningResult> PlanningResultTopic = Root.child(FootstepPlan).topic(FootstepPlannerResult);
   public static final Topic<FootstepPlannerStatus> PlannerStatusTopic = Root.child(FootstepPlan).topic(FootstepPlannerStatus);
   public static final Topic<Integer> PlannerRequestIdTopic = Root.child(FootstepPlan).topic(PlannerRequestId);
   public static final Topic<Integer> ReceivedPlanIdTopic = Root.child(FootstepPlan).topic(ReceivedPlanId);

   public static final Topic<Boolean> EditModeEnabledTopic = Root.child(Any).child(EditMode).topic(Enable);
   public static final Topic<Boolean> StartPositionEditModeEnabledTopic = Root.child(Start).child(EditMode).child(PositionTheme).topic(Enable);
   public static final Topic<Boolean> GoalPositionEditModeEnabledTopic = Root.child(Goal).child(EditMode).child(PositionTheme).topic(Enable);

   public static final Topic<Boolean> StartOrientationEditModeEnabledTopic = Root.child(Start).child(EditMode).child(OrientationTheme).topic(Enable);
   public static final Topic<Boolean> GoalOrientationEditModeEnabledTopic = Root.child(Goal).child(EditMode).child(OrientationTheme).topic(Enable);

   public static final Topic<RobotSide> InitialSupportSideTopic = Root.child(Start).topic(Side);
   public static final Topic<Point3D> StartPositionTopic = Root.child(FootstepPlan).child(Start).topic(Position);
   public static final Topic<Point3D> GoalPositionTopic = Root.child(FootstepPlan).child(Goal).topic(Position);
   public static final Topic<Point3D> LowLevelGoalPositionTopic = Root.child(FootstepPlan).child(Intermediate).topic(Position);

   public static final Topic<Quaternion> StartOrientationTopic = Root.child(FootstepPlan).child(Start).topic(Orientation);
   public static final Topic<Quaternion> GoalOrientationTopic = Root.child(FootstepPlan).child(Goal).topic(Orientation);
   public static final Topic<Quaternion> LowLevelGoalOrientationTopic = Root.child(FootstepPlan).child(Intermediate).topic(Orientation);

   public static final Topic<BipedalSupportPlanarRegionParametersMessage> BipedalSupportRegionsParametersTopic = Root.topic(BipedalSupportRegionsParameters);

   public static final Topic<Boolean> GlobalResetTopic = Root.topic(Reset);

   public static final Topic<Boolean> EnableNodeChecking = Root.child(NodeChecking).topic(Enable);
   public static final Topic<Point3D> NodeCheckingPosition = Root.child(NodeChecking).topic(Position);
   public static final Topic<Quaternion> NodeCheckingOrientation = Root.child(NodeChecking).topic(Orientation);
   public static final Topic<Boolean> EnableNodeCheckingPositionEditing = Root.child(NodeChecking).child(PositionTheme).topic(Enable);

   public static final Topic<Boolean> exportUnitTestDataFile = Root.child(UnitTest).topic(Export);
   public static final Topic<String> exportUnitTestPath = Root.child(UnitTest).topic(Path);

   public static final Topic<List<? extends Point3DReadOnly>> BodyPathDataTopic = Root.child(BodyPath).topic(Data);

   public static final Topic<Boolean> ShowBodyPath = Root.child(BodyPath).topic(Show);

   public static final Topic<List<VisibilityMapWithNavigableRegion>> VisibilityMapWithNavigableRegionData = Root.child(VisibilityGraphs).topic(Data);

   public static final Topic<VisibilityMapHolder> StartVisibilityMap = Root.child(VisibilityGraphs).child(Start).child(Map).topic(Data);
   public static final Topic<VisibilityMapHolder> GoalVisibilityMap = Root.child(VisibilityGraphs).child(Goal).child(Map).topic(Data);
   public static final Topic<VisibilityMapHolder> InterRegionVisibilityMap = Root.child(VisibilityGraphs).child(InterRegion).child(Map).topic(Data);


   public static final Topic<Boolean> ShowClusterRawPoints = Root.child(VisibilityGraphs).child(Cluster).child(Raw).topic(Show);
   public static final Topic<Boolean> ShowClusterNavigableExtrusions = Root.child(VisibilityGraphs).child(Cluster).child(Navigable).topic(Show);
   public static final Topic<Boolean> ShowClusterNonNavigableExtrusions = Root.child(VisibilityGraphs).child(Cluster).child(NonNavigable).topic(Show);

   public static final Topic<Boolean> ShowStartVisibilityMap = Root.child(VisibilityGraphs).child(Start).child(Map).topic(Show);
   public static final Topic<Boolean> ShowGoalVisibilityMap = Root.child(VisibilityGraphs).child(Goal).child(Map).topic(Show);

   public static final Topic<Boolean> ShowInterRegionVisibilityMap = Root.child(VisibilityGraphs).child(InterRegion).child(Map).topic(Show);

   public static final Topic<Boolean> ShowNavigableRegionVisibilityMaps = Root.child(VisibilityGraphs).child(Map).topic(Show);
   public static final Topic<FootstepNodeDataListMessage> NodeDataTopic = Root.child(PlannerData).topic(NodeData);
   public static final Topic<Boolean> ShowNodeDataTopic = Root.child(PlannerData).topic(Show);
   public static final Topic<FootstepPlannerOccupancyMapMessage> OccupancyMapTopic = Root.child(OccupancyMap).topic(OccupancyMapData);
   public static final Topic<Boolean> ShowOccupancyMap = Root.child(OccupancyMap).topic(Show);
   public static final Topic<FootstepPlanningStatistics> PlannerStatisticsTopic = Root.topic(PlannerStatistics);

   public static final Topic<Boolean> RenderShiftedWaypointsTopic = Root.topic(RenderShiftedWaypoints);

   public static final Topic<WalkingControllerPreviewInputMessage> RequestWalkingPreview = Root.child(Preview).topic(PreviewRequest);
   public static final Topic<WalkingControllerPreviewOutputMessage> WalkingPreviewOutput = Root.child(Preview).topic(PreviewResponse);

   private static final TypedTopicTheme<Boolean> ValidNode = apiFactory.createTypedTopicTheme("ValidNode");
   private static final TypedTopicTheme<Pose3D> FootstepPose = apiFactory.createTypedTopicTheme("FootstepPose");

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}