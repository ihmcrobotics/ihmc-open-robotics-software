package us.ihmc.footstepPlanning.communication;

import controller_msgs.msg.dds.*;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlannerStatus;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapWithNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;

public class FootstepPlannerMessagerAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final Category Root = apiFactory.createRootCategory("FootstepPlannerRoot");
   private static final CategoryTheme FootstepPlanner = apiFactory.createCategoryTheme("FootstepPlanner");

   public static final Topic<RobotConfigurationData> RobotConfigurationDataTopic = topic("RobotConfigurationDataTopic");
   public static final Topic<PlanarRegionsList> PlanarRegionDataTopic = topic("PlanarRegionDataTopic");
   public static final Topic<Boolean> ShowPlanarRegionsTopic = topic("ShowPlanarRegionsTopic");
   public static final Topic<Boolean> AcceptNewPlanarRegions = topic("AcceptNewPlanarRegions");
   public static final Topic<PlanarRegion> SelectedRegionTopic = topic("SelectedRegionTopic");

   public static final Topic<FootstepDataListMessage> FootstepPlanResponseTopic = topic("FootstepPlanResponseTopic");
   public static final Topic<FootstepDataListMessage> FootstepPlanToRobotTopic = topic("FootstepPlanToRobotTopic");
   public static final Topic<Boolean> ShowFootstepPlanTopic = topic("ShowFootstepPlanTopic");

   public static final Topic<Boolean> ComputePathTopic = topic("ComputePathTopic");
   public static final Topic<Boolean> AbortPlanningTopic = topic("AbortPlanningTopic");
   public static final Topic<Boolean> RequestPlannerStatistics = topic("RequestPlannerStatistics");
   public static final Topic<Boolean> AssumeFlatGround = topic("AssumeFlatGround");
   public static final Topic<FootstepPlannerParametersReadOnly> PlannerParametersTopic = topic("PlannerParametersTopic");

   public static final Topic<GoHomeMessage> GoHomeTopic = topic("GoHomeTopic");
   public static final Topic<Boolean> IgnorePartialFootholdsTopic = topic("IgnorePartialFootholdsTopic");

   public static final Topic<VisibilityGraphsParametersReadOnly> VisibilityGraphsParametersTopic = topic("VisibilityGraphsParametersTopic");
   public static final Topic<Double> PlannerTimeoutTopic = topic("PlannerTimeoutTopic");
   public static final Topic<Double> PlannerTimeTakenTopic = topic("PlannerTimeTakenTopic");
   public static final Topic<Double> PlannerHorizonLengthTopic = topic("PlannerHorizonLengthTopic");
   public static final Topic<FootstepPlannerType> PlannerTypeTopic = topic("PlannerTypeTopic");
   public static final Topic<FootstepPlanningResult> PlanningResultTopic = topic("PlanningResultTopic");
   public static final Topic<FootstepPlannerStatus> PlannerStatusTopic = topic("PlannerStatusTopic");
   public static final Topic<Integer> PlannerRequestIdTopic = topic("PlannerRequestIdTopic");
   public static final Topic<Integer> ReceivedPlanIdTopic = topic("ReceivedPlanIdTopic");

   public static final Topic<Boolean> EditModeEnabledTopic = topic("EditModeEnabledTopic");
   public static final Topic<Boolean> StartPositionEditModeEnabledTopic = topic("StartPositionEditModeEnabledTopic");
   public static final Topic<Boolean> GoalPositionEditModeEnabledTopic = topic("GoalPositionEditModeEnabledTopic");

   public static final Topic<Boolean> StartOrientationEditModeEnabledTopic = topic("StartOrientationEditModeEnabledTopic");
   public static final Topic<Boolean> GoalOrientationEditModeEnabledTopic = topic("GoalOrientationEditModeEnabledTopic");

   public static final Topic<RobotSide> InitialSupportSideTopic = topic("InitialSupportSideTopic");
   public static final Topic<Point3D> StartPositionTopic = topic("StartPositionTopic");
   public static final Topic<Point3D> GoalPositionTopic = topic("GoalPositionTopic");
   public static final Topic<Point3D> LowLevelGoalPositionTopic = topic("LowLevelGoalPositionTopic");

   public static final Topic<Quaternion> StartOrientationTopic = topic("StartOrientationTopic");
   public static final Topic<Quaternion> GoalOrientationTopic = topic("GoalOrientationTopic");
   public static final Topic<Quaternion> LowLevelGoalOrientationTopic = topic("LowLevelGoalOrientationTopic");

   public static final Topic<Double> GoalDistanceProximity = topic("GoalDistanceProximity");
   public static final Topic<Double> GoalYawProximity = topic("GoalYawProximity");

   public static final Topic<BipedalSupportPlanarRegionParametersMessage> BipedalSupportRegionsParametersTopic = topic("BipedalSupportRegionsParametersTopic");

   public static final Topic<Boolean> GlobalResetTopic = topic("GlobalResetTopic");

   public static final Topic<Boolean> EnableNodeChecking = topic("EnableNodeChecking");
   public static final Topic<Point3D> NodeCheckingPosition = topic("NodeCheckingPosition");
   public static final Topic<Quaternion> NodeCheckingOrientation = topic("NodeCheckingOrientation");
   public static final Topic<Boolean> EnableNodeCheckingPositionEditing = topic("EnableNodeCheckingPositionEditing");

   public static final Topic<Boolean> exportUnitTestDataFile = topic("exportUnitTestDataFile");
   public static final Topic<String> exportUnitTestPath = topic("exportUnitTestPath");

   public static final Topic<List<? extends Pose3DReadOnly>> BodyPathDataTopic = topic("BodyPathDataTopic");

   public static final Topic<Boolean> ShowBodyPath = topic("ShowBodyPath");

   public static final Topic<List<VisibilityMapWithNavigableRegion>> VisibilityMapWithNavigableRegionData = topic("VisibilityMapWithNavigableRegionData");

   public static final Topic<VisibilityMapHolder> StartVisibilityMap = topic("StartVisibilityMap");
   public static final Topic<VisibilityMapHolder> GoalVisibilityMap = topic("GoalVisibilityMap");
   public static final Topic<VisibilityMapHolder> InterRegionVisibilityMap = topic("InterRegionVisibilityMap");

   public static final Topic<Boolean> ShowClusterRawPoints = topic("ShowClusterRawPoints");
   public static final Topic<Boolean> ShowClusterPreferredNavigableExtrusions = topic("ShowClusterPreferredNavigableExtrusions");
   public static final Topic<Boolean> ShowClusterPreferredNonNavigableExtrusions = topic("ShowClusterPreferredNonNavigableExtrusions");
   public static final Topic<Boolean> ShowClusterNavigableExtrusions = topic("ShowClusterNavigableExtrusions");
   public static final Topic<Boolean> ShowClusterNonNavigableExtrusions = topic("ShowClusterNonNavigableExtrusions");

   public static final Topic<Boolean> ShowStartVisibilityMap = topic("ShowStartVisibilityMap");
   public static final Topic<Boolean> ShowGoalVisibilityMap = topic("ShowGoalVisibilityMap");

   public static final Topic<Boolean> ShowInterRegionVisibilityMap = topic("ShowInterRegionVisibilityMap");

   public static final Topic<Boolean> ShowNavigableRegionVisibilityMaps = topic("ShowNavigableRegionVisibilityMaps");
   public static final Topic<FootstepNodeDataListMessage> NodeDataTopic = topic("NodeDataTopic");
   public static final Topic<Boolean> ShowNodeDataTopic = topic("ShowNodeDataTopic");
   public static final Topic<FootstepPlannerOccupancyMapMessage> OccupancyMapTopic = topic("OccupancyMapTopic");
   public static final Topic<Boolean> ShowOccupancyMap = topic("ShowOccupancyMap");
   public static final Topic<FootstepPlanningStatistics> PlannerStatisticsTopic = topic("PlannerStatisticsTopic");

   public static final Topic<Boolean> RenderShiftedWaypointsTopic = topic("RenderShiftedWaypointsTopic");

   public static final Topic<WalkingControllerPreviewInputMessage> RequestWalkingPreview = topic("RequestWalkingPreview");
   public static final Topic<WalkingControllerPreviewOutputMessage> WalkingPreviewOutput = topic("WalkingPreviewOutput");

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static final <T> Topic<T> topic(String name)
   {
      return Root.child(FootstepPlanner).topic(apiFactory.createTypedTopicTheme(name));
   }
}