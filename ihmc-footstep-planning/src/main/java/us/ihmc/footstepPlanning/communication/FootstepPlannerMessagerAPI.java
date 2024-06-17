package us.ihmc.footstepPlanning.communication;

import controller_msgs.msg.dds.*;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.OcTreeKeyListMessage;
import toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.BodyPathPlanningResult;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.bodyPath.BodyPathLatticePoint;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.log.*;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.HeightMapDataSetName;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
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
   public static final Topic<controller_msgs.msg.dds.RobotConfigurationData> RobotConfigurationData = topic("RobotConfigurationData");
   public static final Topic<DataSetName> DataSetSelected = topic("DataSetSelected");
   public static final Topic<HeightMapDataSetName> HeightMapDataSetSelected = topic("HeightMapDataSetSelected");
   public static final Topic<ConvexPolygon2D> LeftFootStartSupportPolygon = topic("LeftFootStartSupportPolygon");
   public static final Topic<ConvexPolygon2D> RightFootStartSupportPolygon = topic("RightFootStartSupportPolygon");
   public static final Topic<Pose3DReadOnly> LeftFootPose = topic("LeftStartPose");
   public static final Topic<Pose3DReadOnly> RightFootPose = topic("RightStartPose");
   public static final Topic<Point3D> StartMidFootPosition = topic("StartMidFootPosition");
   public static final Topic<Quaternion> StartMidFootOrientation = topic("StartMidFootOrientation");
   public static final Topic<FootstepStatusMessage> FootstepStatusMessage = topic("FootstepStatusMessage");
   public static final Topic<Pair<RobotSide, double[]>> RequestedArmJointAngles = topic("RequestedArmJointAngles");

   // Perception data
   public static final Topic<PlanarRegionsList> PlanarRegionData = topic("PlanarRegionData");
   public static final Topic<HeightMapMessage> HeightMapData = topic("HeightMapData");
   public static final Topic<Boolean> AcceptNewPlanarRegions = topic("AcceptNewPlanarRegions");
   public static final Topic<OcTreeKeyListMessage> OcTreeData = topic("OcTreeData");

   // UI control
   public static final Topic<Boolean> IgnorePartialFootholds = topic("IgnorePartialFootholds");
   public static final Topic<Boolean> GlobalReset = topic("GlobalReset");
   public static final Topic<Boolean> ComputePath = topic("ComputePath");
   public static final Topic<Boolean> HaltPlanning = topic("HaltPlanning");
   public static final Topic<Boolean> BindStartToRobot = topic("BindStartToRobot");
   public static final Topic<Boolean> ReplanSwing = topic("ReplanSwing");

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
   public static final Topic<DefaultFootstepPlannerParametersReadOnly> PlannerParameters = topic("PlannerParameters");
   public static final Topic<AStarBodyPathPlannerParametersReadOnly> AStarBodyPathPlannerParameters = topic("AStarBodyPathPlannerParameters");
   public static final Topic<VisibilityGraphsParametersReadOnly> VisibilityGraphsParameters = topic("VisibilityGraphsParameters");
   public static final Topic<SwingPlannerParametersReadOnly> SwingPlannerParameters = topic("SwingPlannerParameters");
   public static final Topic<BipedalSupportPlanarRegionParametersMessage> BipedalSupportRegionsParameters = topic("BipedalSupportRegionsParameters");

   // Graphics control
   public static final Topic<Boolean> ShowRobot = topic("ShowRobot");
   public static final Topic<Boolean> ShowPlanarRegions = topic("ShowPlanarRegions");
   public static final Topic<Boolean> ShowOcTree = topic("ShowOcTree");
   public static final Topic<Boolean> ShowStart = topic("ShowStart");
   public static final Topic<Boolean> ShowGoal = topic("ShowGoal");
   public static final Topic<Boolean> ShowCoordinateSystem = topic("ShowCoordinateSystem");

   public static final Topic<Boolean> ShowBodyPath = topic("ShowBodyPath");
   public static final Topic<Boolean> ShowClusterRawPoints = topic("ShowClusterRawPoints");
   public static final Topic<Boolean> ShowClusterNavigableExtrusions = topic("ShowClusterNavigableExtrusions");
   public static final Topic<Boolean> ShowClusterNonNavigableExtrusions = topic("ShowClusterNonNavigableExtrusions");
   public static final Topic<Boolean> ShowStartVisibilityMap = topic("ShowStartVisibilityMap");
   public static final Topic<Boolean> ShowGoalVisibilityMap = topic("ShowGoalVisibilityMap");
   public static final Topic<Boolean> ShowInterRegionVisibilityMap = topic("ShowInterRegionVisibilityMap");
   public static final Topic<Boolean> ShowNavigableRegionVisibilityMaps = topic("ShowNavigableRegionVisibilityMaps");

   public static final Topic<Boolean> ShowFootstepPlan = topic("ShowFootstepPlan");
   public static final Topic<Boolean> ShowPostProcessingInfo = topic("ShowPostProcessingInfo");
   public static final Topic<Boolean> ShowLogGraphics = topic("ShowLogGraphics");
   public static final Topic<Boolean> ShowBodyPathLogGraphics = topic("ShowBodyPathLogGraphics");
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
   public static final Topic<Boolean> PlanNarrowPassage = topic("PlanNarrowPassage");
   public static final Topic<Boolean> PerformAStarSearch = topic("PerformAStarSearch");
   public static final Topic<SwingPlannerType> RequestedSwingPlannerType = topic("RequestedSwingPlannerType");
   public static final Topic<Boolean> PlanSingleStep = topic("PlanSingleStep");

   public static final Topic<FootstepPlan> ReferencePlan = topic("ReferencePlan");

   // Robot control
   public static final Topic<GoHomeMessage> GoHomeTopic = topic("GoHome");

   // Body path planner output
   public static final Topic<Pair<List<? extends Pose3DReadOnly>, List<? extends Point3DReadOnly>>> BodyPathData = topic("BodyPathData");

   // Footstep planner output
   public static final Topic<Boolean> SendPlan = topic("SendPlan");
   public static final Topic<FootstepDataListMessage> FootstepPlanResponse = topic("FootstepPlanResponse"); // Planner >> UI
   public static final Topic<FootstepDataListMessage> FootstepPlanToRobot = topic("FootstepPlanToRobot"); // UI >> Robot (if operator adjusts path or overrides swing times, etc.)
   public static final Topic<Point3D> LowLevelGoalPosition = topic("LowLevelGoalPosition");
   public static final Topic<Quaternion> LowLevelGoalOrientation = topic("LowLevelGoalOrientation");
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

   // Height map navigation
   public static final Topic<Boolean> StartHeightMapNavigation = topic("StartHeightMapNavigation");
   public static final Topic<Boolean> StopHeightMapNavigation = topic("StopHeightMapNavigation");
   public static final Topic<PlanarRegionsList> GPUREARegions = topic("GPUREARegions");
   public static final Topic<Boolean> ApproveStep = topic("ApproveStep");
   public static final Topic<Boolean> ReplanStep = topic("ReplanStep");
   public static final Topic<Boolean> WriteHeightMapLog = topic("WriteHeightMapLog");
   public static final Topic<Boolean> ResendLastStep = topic("ResendLastStep");
   public static final Topic<Boolean> ReconnectRos1Node = topic("ReconnectRos1Node");

   // Logging
   public static final Topic<Boolean> RequestGenerateLog = topic("RequestGenerateLog");
   public static final Topic<FootstepPlannerLogLoader.LoadRequestType> RequestLoadLog = topic("RequestLoadLog");
   public static final Topic<String> GenerateLogStatus = topic("GenerateLogStatus");
   public static final Topic<String> LoadLogStatus = topic("LoadLogStatus");
   public static final Topic<Triple<Map<GraphEdge<FootstepGraphNode>, FootstepPlannerEdgeData>, List<FootstepPlannerIterationData>, List<VariableDescriptor>>> GraphData = topic("GraphData");
   public static final Topic<Triple<Map<GraphEdge<BodyPathLatticePoint>, AStarBodyPathEdgeData>, List<AStarBodyPathIterationData>, List<VariableDescriptor>>> BodyPathGraphData = topic("BodyPathGraphData");
   public static final Topic<Pair<DiscreteFootstep, FootstepSnapData>> StartOfSwingStepToVisualize = topic("StartOfSwingStepToVisualize");
   public static final Topic<Pair<DiscreteFootstep, FootstepSnapData>> StanceStepToVisualize = topic("StanceStepToVisualize");
   public static final Topic<Pair<DiscreteFootstep, FootstepSnapData>> TouchdownStepToVisualize = topic("TouchdownStepToVisualize");
   public static final Topic<RigidBodyTransform> LoggedIdealStep = topic("LoggedIdealStep");
   public static final Topic<RigidBodyTransform> LoggedNominalIdealStep = topic("LoggedNominalIdealStep");
   public static final Topic<List<Box3D>> LoggedCollisionBoxes = topic("LoggedCollisionBoxes");

   public static final Topic<Boolean> ShowLoggedStartOfSwingStep = topic("ShowLoggedStartOfSwingStep");
   public static final Topic<Boolean> ShowLoggedStanceStep = topic("ShowLoggedStanceStep");
   public static final Topic<Boolean> ShowLoggedUnsnappedCandidateStep = topic("ShowLoggedUnsnappedCandidateStep");
   public static final Topic<Boolean> ShowLoggedSnappedCandidateStep = topic("ShowLoggedSnappedCandidateStep");
   public static final Topic<Boolean> ShowLoggedWiggledCandidateStep = topic("ShowLoggedWiggledCandidateStep");
   public static final Topic<Boolean> ShowLoggedIdealStep = topic("ShowLoggedIdealStep");
   public static final Topic<Boolean> ShowLoggedNominalIdealStep = topic("ShowLoggedNominalIdealStep");
   public static final Topic<Boolean> ShowBodyBox = topic("ShowBodyBox");
   public static final Topic<Boolean> ShowHeightMap = topic("ShowHeightMap");

   public static final Topic<Pair<BodyPathLatticePoint, Double>> BodyPathStartNodeToVisualize = topic("BodyPathStartNodeToVisualize");
   public static final Topic<Pair<BodyPathLatticePoint, Double>> BodyPathCandidateNodeToVisualize = topic("BodyPathCandidateNodeToVisualize");
   public static final Topic<Boolean> ShowBodyPathPlanData = topic("ShowBodyPathPlanData");

   // Test dashboard, only displayed if launched from test class
   public static final Topic<List<DataSet>> TestDataSets = topic("TestDataSets");
   public static final Topic<DataSet> TestDataSetSelected = topic("TestDataSetSelected");

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static <T> Topic<T> topic(String name)
   {
      return Root.child(FootstepPlanner).topic(apiFactory.createTypedTopicTheme(name));
   }
}