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

   public static final Topic<RobotConfigurationData> RobotConfigurationData = topic("RobotConfigurationData");
   public static final Topic<PlanarRegionsList> PlanarRegionData = topic("PlanarRegionData");
   public static final Topic<Boolean> ShowPlanarRegions = topic("ShowPlanarRegions");
   public static final Topic<Boolean> AcceptNewPlanarRegions = topic("AcceptNewPlanarRegions");
   public static final Topic<PlanarRegion> SelectedRegion = topic("SelectedRegion");

   public static final Topic<FootstepDataListMessage> FootstepPlanResponse = topic("FootstepPlanResponse");
   public static final Topic<FootstepDataListMessage> FootstepPlanToRobot = topic("FootstepPlanToRobot");
   public static final Topic<Boolean> ShowFootstepPlan = topic("ShowFootstepPlan");

   public static final Topic<Boolean> ComputePath = topic("ComputePath");
   public static final Topic<Boolean> AbortPlanning = topic("AbortPlanning");
   public static final Topic<Boolean> RequestPlannerStatistics = topic("RequestPlannerStatistics");
   public static final Topic<Boolean> AssumeFlatGround = topic("AssumeFlatGround");
   public static final Topic<FootstepPlannerParametersReadOnly> PlannerParameters = topic("PlannerParameters");

   public static final Topic<GoHomeMessage> GoHomeTopic = topic("GoHome");
   public static final Topic<Boolean> IgnorePartialFootholds = topic("IgnorePartialFootholds");

   public static final Topic<VisibilityGraphsParametersReadOnly> VisibilityGraphsParameters = topic("VisibilityGraphsParameters");
   public static final Topic<Double> PlannerTimeout = topic("PlannerTimeout");
   public static final Topic<Double> PlannerBestEffortTimeout = topic("PlannerBestEffortTimeout");
   public static final Topic<Double> PlannerTimeTaken = topic("PlannerTimeTaken");
   public static final Topic<Double> PlannerHorizonLength = topic("PlannerHorizonLength");
   public static final Topic<FootstepPlannerType> PlannerType = topic("PlannerType");
   public static final Topic<FootstepPlanningResult> PlanningResult = topic("PlanningResult");
   public static final Topic<FootstepPlannerStatus> PlannerStatus = topic("PlannerStatus");
   public static final Topic<Integer> PlannerRequestId = topic("PlannerRequestId");
   public static final Topic<Integer> ReceivedPlanId = topic("ReceivedPlanId");

   public static final Topic<Boolean> EditModeEnabled = topic("EditModeEnabled");
   public static final Topic<Boolean> StartPositionEditModeEnabled = topic("StartPositionEditModeEnabled");
   public static final Topic<Boolean> GoalPositionEditModeEnabled = topic("GoalPositionEditModeEnabled");

   public static final Topic<Boolean> StartOrientationEditModeEnabled = topic("StartOrientationEditModeEnabled");
   public static final Topic<Boolean> GoalOrientationEditModeEnabled = topic("GoalOrientationEditModeEnabled");

   public static final Topic<RobotSide> InitialSupportSide = topic("InitialSupportSide");
   public static final Topic<Point3D> StartPosition = topic("StartPosition");
   public static final Topic<Point3D> GoalPosition = topic("GoalPosition");
   public static final Topic<Point3D> LowLevelGoalPosition = topic("LowLevelGoalPosition");

   public static final Topic<Quaternion> StartOrientation = topic("StartOrientation");
   public static final Topic<Quaternion> GoalOrientation = topic("GoalOrientation");
   public static final Topic<Quaternion> LowLevelGoalOrientation = topic("LowLevelGoalOrientation");

   public static final Topic<Double> GoalDistanceProximity = topic("GoalDistanceProximity");
   public static final Topic<Double> GoalYawProximity = topic("GoalYawProximity");

   public static final Topic<BipedalSupportPlanarRegionParametersMessage> BipedalSupportRegionsParameters = topic("BipedalSupportRegionsParameters");

   public static final Topic<Boolean> GlobalReset = topic("GlobalReset");

   public static final Topic<Boolean> EnableNodeChecking = topic("EnableNodeChecking");
   public static final Topic<Point3D> NodeCheckingPosition = topic("NodeCheckingPosition");
   public static final Topic<Quaternion> NodeCheckingOrientation = topic("NodeCheckingOrientation");
   public static final Topic<Boolean> EnableNodeCheckingPositionEditing = topic("EnableNodeCheckingPositionEditing");

   public static final Topic<Boolean> ExportUnitTestDataFile = topic("ExportUnitTestDataFile");
   public static final Topic<String> ExportUnitTestPath = topic("ExportUnitTestPath");

   public static final Topic<List<? extends Pose3DReadOnly>> BodyPathData = topic("BodyPathData");

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
   public static final Topic<FootstepNodeDataListMessage> NodeData = topic("NodeData");
   public static final Topic<Boolean> ShowNodeData = topic("ShowNodeData");
   public static final Topic<FootstepPlannerOccupancyMapMessage> OccupancyMap = topic("OccupancyMap");
   public static final Topic<Boolean> ShowOccupancyMap = topic("ShowOccupancyMap");
   public static final Topic<FootstepPlanningStatistics> PlannerStatistics = topic("PlannerStatistics");

   public static final Topic<Boolean> RenderShiftedWaypoints = topic("RenderShiftedWaypoints");

   public static final Topic<WalkingControllerPreviewInputMessage> RequestWalkingPreview = topic("RequestWalkingPreview");
   public static final Topic<WalkingControllerPreviewOutputMessage> WalkingPreviewOutput = topic("WalkingPreviewOutput");

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static final <T> Topic<T> topic(String name)
   {
      return Root.child(FootstepPlanner).topic(apiFactory.createTypedTopicTheme(name));
   }
}