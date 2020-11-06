package us.ihmc.footstepPlanning.communication;

import controller_msgs.msg.dds.*;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.BodyPathPlanningResult;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerOccupancyMap;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.log.FootstepPlannerEdgeData;
import us.ihmc.footstepPlanning.log.FootstepPlannerIterationData;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.footstepPlanning.log.VariableDescriptor;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapWithNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;
import java.util.Map;

public class FootstepPlannerMessagerAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final Category Root = apiFactory.createRootCategory("FootstepPlannerRoot");
   private static final CategoryTheme FootstepPlanner = apiFactory.createCategoryTheme("FootstepPlanner");

   // Robot state
   public static final Topic<RobotConfigurationData> RobotConfigurationData = topic("RobotConfigurationData");
   public static final Topic<DataSetName> DataSetSelected = topic("DataSetSelected");
   public static final Topic<ConvexPolygon2D> LeftFootStartSupportPolygon = topic("LeftFootStartSupportPolygon");
   public static final Topic<ConvexPolygon2D> RightFootStartSupportPolygon = topic("RightFootStartSupportPolygon");
   public static final Topic<Pose3DReadOnly> LeftFootPose = topic("LeftStartPose");
   public static final Topic<Pose3DReadOnly> RightFootPose = topic("RightStartPose");
   public static final Topic<Point3D> StartMidFootPosition = topic("StartMidFootPosition");
   public static final Topic<Quaternion> StartMidFootOrientation = topic("StartMidFootOrientation");
   public static final Topic<FootstepStatusMessage> FootstepStatusMessage = topic("FootstepStatusMessage");
   public static final Topic<Pair<RobotSide, double[]>> RequestedArmJointAngles = topic("RequestedArmJointAngles");

   // REA data
   public static final Topic<PlanarRegionsList> PlanarRegionData = topic("PlanarRegionData");
   public static final Topic<Boolean> AcceptNewPlanarRegions = topic("AcceptNewPlanarRegions");

   // UI control
   public static final Topic<Boolean> IgnorePartialFootholds = topic("IgnorePartialFootholds");
   public static final Topic<Boolean> GlobalReset = topic("GlobalReset");
   public static final Topic<Boolean> ComputePath = topic("ComputePath");
   public static final Topic<Boolean> HaltPlanning = topic("HaltPlanning");
   public static final Topic<Boolean> PostProcessPlan = topic("PostProcessPlan");
   public static final Topic<Boolean> BindStartToRobot = topic("BindStartToRobot");

   // Override planned path
   public static final Topic<Boolean> OverrideStepTimings = topic("overrideStepTimings");
   public static final Topic<Double> ManualSwingTime = topic("manualSwingTime");
   public static final Topic<Double> ManualTransferTime = topic("manualTransferTime");
   public static final Topic<Pair<Integer, Double>> OverrideSpecificSwingTime = topic("OverrideSpecificSwingTime");

   public static final Topic<Boolean> OverrideSwingHeight = topic("overrideSwingHeight");
   public static final Topic<Double> ManualSwingHeight = topic("manualSwingHeight");

   public static final Topic<Pair<Integer, FootstepDataMessage>> SelectedFootstep = topic("SelectedFootstep"); // table or click >>> ManualFootstepAdjustmentListener
   public static final Topic<Pair<Integer, Pose3D>> ManuallyAdjustmentedStep = topic("ManualStepAdjustment"); // ManualFootstepAdjustmentListener >>> UIFootstepPlanManager
   public static final Topic<Pair<Integer, FootstepDataMessage>> FootstepToUpdateViz = topic("FootstepToUpdateViz"); // UIFootstepPlanManager >>> FootstepPathMeshViewer
   public static final Topic<UIStepAdjustmentFrame> FootstepAdjustmentFrame = topic("FootstepAdjustmentFrame"); // ManualFootstepAdjustmentListener >>> table

   // Parameters
   public static final Topic<FootstepPlannerParametersReadOnly> PlannerParameters = topic("PlannerParameters");
   public static final Topic<VisibilityGraphsParametersReadOnly> VisibilityGraphsParameters = topic("VisibilityGraphsParameters");
   public static final Topic<SwingPlannerParametersReadOnly> SwingPlannerParameters = topic("SwingPlannerParameters");
   public static final Topic<BipedalSupportPlanarRegionParametersMessage> BipedalSupportRegionsParameters = topic("BipedalSupportRegionsParameters");

   // Graphics control
   public static final Topic<Boolean> ShowRobot = topic("ShowRobot");
   public static final Topic<Boolean> ShowPlanarRegions = topic("ShowPlanarRegions");
   public static final Topic<Boolean> ShowStart = topic("ShowStart");
   public static final Topic<Boolean> ShowGoal = topic("ShowGoal");
   public static final Topic<Boolean> ShowCoordinateSystem = topic("ShowCoordinateSystem");

   public static final Topic<Boolean> ShowBodyPath = topic("ShowBodyPath");
   public static final Topic<Boolean> ShowClusterRawPoints = topic("ShowClusterRawPoints");
   public static final Topic<Boolean> ShowClusterPreferredNavigableExtrusions = topic("ShowClusterPreferredNavigableExtrusions");
   public static final Topic<Boolean> ShowClusterPreferredNonNavigableExtrusions = topic("ShowClusterPreferredNonNavigableExtrusions");
   public static final Topic<Boolean> ShowClusterNavigableExtrusions = topic("ShowClusterNavigableExtrusions");
   public static final Topic<Boolean> ShowClusterNonNavigableExtrusions = topic("ShowClusterNonNavigableExtrusions");
   public static final Topic<Boolean> ShowStartVisibilityMap = topic("ShowStartVisibilityMap");
   public static final Topic<Boolean> ShowGoalVisibilityMap = topic("ShowGoalVisibilityMap");
   public static final Topic<Boolean> ShowInterRegionVisibilityMap = topic("ShowInterRegionVisibilityMap");
   public static final Topic<Boolean> ShowNavigableRegionVisibilityMaps = topic("ShowNavigableRegionVisibilityMaps");

   public static final Topic<Boolean> ShowFootstepPlan = topic("ShowFootstepPlan");
   public static final Topic<Boolean> ShowOccupancyMap = topic("ShowOccupancyMap");
   public static final Topic<Boolean> ShowPostProcessingInfo = topic("ShowPostProcessingInfo");
   public static final Topic<Boolean> ShowLogGraphics = topic("ShowLogGraphics");
   public static final Topic<Boolean> RenderShiftedWaypoints = topic("RenderShiftedWaypoints");

   // Goal editing
   public static final Topic<Boolean> EditModeEnabled = topic("EditModeEnabled");
   public static final Topic<Boolean> GoalPositionEditModeEnabled = topic("GoalPositionEditModeEnabled");
   public static final Topic<Boolean> GoalOrientationEditModeEnabled = topic("GoalOrientationEditModeEnabled");
   public static final Topic<PlanarRegion> SelectedRegion = topic("SelectedRegion");

   // Request field information
   public static final Topic<Integer> PlannerRequestId = topic("PlannerRequestId");
   public static final Topic<Double> PlannerTimeout = topic("PlannerTimeout");
   public static final Topic<Integer> MaxIterations = topic("MaxIterations");
   public static final Topic<Double> PlannerHorizonLength = topic("PlannerHorizonLength");
   public static final Topic<RobotSide> InitialSupportSide = topic("InitialSupportSide");
   public static final Topic<Boolean> SnapGoalSteps = topic("SnapGoalSteps");
   public static final Topic<Boolean> AbortIfGoalStepSnapFails = topic("AbortIfGoalStepSnapFails");
   public static final Topic<Pose3DReadOnly> LeftFootGoalPose = topic("LeftFootGoalPose");
   public static final Topic<Pose3DReadOnly> RightFootGoalPose = topic("RightFootGoalPose");
   public static final Topic<Point3D> GoalMidFootPosition = topic("GoalMidFootPosition");
   public static final Topic<Quaternion> GoalMidFootOrientation = topic("GoalMidFootOrientation");
   public static final Topic<Double> GoalDistanceProximity = topic("GoalDistanceProximity");
   public static final Topic<Double> GoalYawProximity = topic("GoalYawProximity");
   public static final Topic<Double> RequestedFootstepPlanHeading = topic("FootstepPlanHeading");

   public static final Topic<Boolean> AssumeFlatGround = topic("AssumeFlatGround");
   public static final Topic<Boolean> PlanBodyPath = topic("PlanBodyPath");
   public static final Topic<Boolean> PerformAStarSearch = topic("PerformAStarSearch");
   public static final Topic<SwingPlannerType> RequestedSwingPlannerType = topic("RequestedSwingPlannerType");

   // Robot control
   public static final Topic<GoHomeMessage> GoHomeTopic = topic("GoHome");

   // Body path planner output
   public static final Topic<List<? extends Pose3DReadOnly>> BodyPathData = topic("BodyPathData");
   public static final Topic<List<VisibilityMapWithNavigableRegion>> VisibilityMapWithNavigableRegionData = topic("VisibilityMapWithNavigableRegionData");
   public static final Topic<VisibilityMapHolder> StartVisibilityMap = topic("StartVisibilityMap");
   public static final Topic<VisibilityMapHolder> GoalVisibilityMap = topic("GoalVisibilityMap");
   public static final Topic<VisibilityMapHolder> InterRegionVisibilityMap = topic("InterRegionVisibilityMap");

   // Footstep planner output
   public static final Topic<Boolean> SendPlan = topic("SendPlan");
   public static final Topic<FootstepDataListMessage> FootstepPlanResponse = topic("FootstepPlanResponse");
   public static final Topic<FootstepDataListMessage> FootstepPlanToRobot = topic("FootstepPlanToRobot");
   public static final Topic<Point3D> LowLevelGoalPosition = topic("LowLevelGoalPosition");
   public static final Topic<Quaternion> LowLevelGoalOrientation = topic("LowLevelGoalOrientation");
   public static final Topic<PlannerOccupancyMap> OccupancyMap = topic("OccupancyMap");
   public static final Topic<FootstepPlanningTimingsMessage> PlannerTimings = topic("PlannerTimings");
   public static final Topic<BodyPathPlanningResult> BodyPathPlanningResultTopic = topic("BodyPathPlanningResult");
   public static final Topic<FootstepPlanningResult> FootstepPlanningResultTopic = topic("FootstepPlanningResult");
   public static final Topic<Integer> ReceivedPlanId = topic("ReceivedPlanId");
   public static final Topic<String> PlannerExceptionStackTrace = topic("PlannerExceptionStackTrace");

   // IK
   public static final Topic<Boolean> IKEnabled = topic("IKEnabled");
   public static final Topic<UserInterfaceIKMode> SelectedIKMode = topic("SelectedIKMode");
   public static final Topic<double[]> IKSolution = topic("IKSolution");
   public static final Topic<ArmTrajectoryMessage> ArmTrajectoryMessageTopic = topic("ArmTrajectoryMessageTopic");
   public static final Topic<HandTrajectoryMessage> HandTrajectoryMessageTopic = topic("HandTrajectoryMessageTopic");
   public static final Topic<FootTrajectoryMessage> FootTrajectoryMessageTopic = topic("FootTrajectoryMessageTopic");
   public static final Topic<ChestTrajectoryMessage> ChestTrajectoryMessageTopic = topic("ChestTrajectoryMessageTopic");
   public static final Topic<SpineTrajectoryMessage> SpineTrajectoryMessageTopic = topic("SpineTrajectoryMessageTopic");
   public static final Topic<HeadTrajectoryMessage> HeadTrajectoryMessageTopic = topic("HeadTrajectoryMessageTopic");
   public static final Topic<NeckTrajectoryMessage> NeckTrajectoryMessageTopic = topic("NeckTrajectoryMessageTopic");

   // Logging
   public static final Topic<Boolean> RequestGenerateLog = topic("RequestGenerateLog");
   public static final Topic<FootstepPlannerLogLoader.LoadRequestType> RequestLoadLog = topic("RequestLoadLog");
   public static final Topic<String> GenerateLogStatus = topic("GenerateLogStatus");
   public static final Topic<String> LoadLogStatus = topic("LoadLogStatus");
   public static final Topic<Triple<Map<GraphEdge<FootstepGraphNode>, FootstepPlannerEdgeData>, List<FootstepPlannerIterationData>, List<VariableDescriptor>>> GraphData = topic("GraphData");
   public static final Topic<Pair<DiscreteFootstep, FootstepSnapData>> StartOfSwingStepToVisualize = topic("StartOfSwingStepToVisualize");
   public static final Topic<Pair<DiscreteFootstep, FootstepSnapData>> StanceStepToVisualize = topic("StanceStepToVisualize");
   public static final Topic<Pair<DiscreteFootstep, FootstepSnapData>> TouchdownStepToVisualize = topic("TouchdownStepToVisualize");
   public static final Topic<RigidBodyTransform> LoggedIdealStep = topic("LoggedIdealStep");

   public static final Topic<Boolean> ShowLoggedStartOfSwingStep = topic("ShowLoggedStartOfSwingStep");
   public static final Topic<Boolean> ShowLoggedStanceStep = topic("ShowLoggedStanceStep");
   public static final Topic<Boolean> ShowLoggedUnsnappedCandidateStep = topic("ShowLoggedUnsnappedCandidateStep");
   public static final Topic<Boolean> ShowLoggedSnappedCandidateStep = topic("ShowLoggedSnappedCandidateStep");
   public static final Topic<Boolean> ShowLoggedWiggledCandidateStep = topic("ShowLoggedWiggledCandidateStep");
   public static final Topic<Boolean> ShowLoggedIdealStep = topic("ShowLoggedIdealStep");

   // Test dashboard, only displayed if launched from test class
   public static final Topic<List<DataSet>> TestDataSets = topic("TestDataSets");
   public static final Topic<DataSet> TestDataSetSelected = topic("TestDataSetSelected");

   // Walking preview
   public static final Topic<WalkingControllerPreviewInputMessage> RequestWalkingPreview = topic("RequestWalkingPreview");
   public static final Topic<WalkingControllerPreviewOutputMessage> WalkingPreviewOutput = topic("WalkingPreviewOutput");

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static <T> Topic<T> topic(String name)
   {
      return Root.child(FootstepPlanner).topic(apiFactory.createTypedTopicTheme(name));
   }
}