package us.ihmc.footstepPlanning.ui;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.*;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepPlannerUserInterfaceAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final CategoryTheme PlanarRegion = apiFactory.createCategoryTheme("PlanarRegion");
   private static final CategoryTheme Start = apiFactory.createCategoryTheme("Start");
   private static final CategoryTheme Goal = apiFactory.createCategoryTheme("Goal");
   private static final CategoryTheme PositionTheme = apiFactory.createCategoryTheme("PositionTheme");
   private static final CategoryTheme OrientationTheme = apiFactory.createCategoryTheme("OrientationTheme");
   private static final CategoryTheme EditMode = apiFactory.createCategoryTheme("EditMode");
   private static final CategoryTheme FootstepPlan = apiFactory.createCategoryTheme("FootstepPlan");
   private static final CategoryTheme NodeChecking = apiFactory.createCategoryTheme("NodeChecking");

   private static final TopicTheme Parameters = apiFactory.createTopicTheme("Parameters");

   private static final TypedTopicTheme<Boolean> Show = apiFactory.createTypedTopicTheme("Show");
   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");
   private static final TypedTopicTheme<Boolean> Reset = apiFactory.createTypedTopicTheme("Reset");
   private static final TypedTopicTheme<Point3D> Position = apiFactory.createTypedTopicTheme("Position");
   private static final TypedTopicTheme<RobotSide> Side = apiFactory.createTypedTopicTheme("Side");
   private static final TypedTopicTheme<Quaternion> Orientation = apiFactory.createTypedTopicTheme("Orientation");
   private static final TypedTopicTheme<Boolean> ComputePath = apiFactory.createTypedTopicTheme("ComputePath");
   private static final TypedTopicTheme<Double> PlannerTimeout = apiFactory.createTypedTopicTheme("PlannerTimeout");
   private static final TypedTopicTheme<Double> PlannerHorizonLength = apiFactory.createTypedTopicTheme("PlannerHorizonLength");
   private static final TypedTopicTheme<FootstepPlannerType> FootstepPlannerType = apiFactory.createTypedTopicTheme("FootstepPlannerType");
   private static final TypedTopicTheme<FootstepPlanningResult> FootstepPlannerResult = apiFactory.createTypedTopicTheme("FootstepPlannerResult");
   private static final TypedTopicTheme<FootstepPlannerParameters> FootstepPlannerParameters = apiFactory.createTypedTopicTheme("FootstepPlannerParameters");

   private static final TypedTopicTheme<Double> NodeCheckerCliffHeight = apiFactory.createTypedTopicTheme("NodeCheckerCliffHeight");
   private static final TypedTopicTheme<Double> NodeCheckerCliffMinDistance = apiFactory.createTypedTopicTheme("NodeCheckerCliffMinDistance");
   private static final TypedTopicTheme<Boolean> ValidNode = apiFactory.createTypedTopicTheme("ValidNode");
   private static final TypedTopicTheme<Pose3D> FootstepPose = apiFactory.createTypedTopicTheme("FootstepPose");

   private static final TopicTheme Data = apiFactory.createTopicTheme("Data");

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("FootstepPlanning"));

   public static final Topic<PlanarRegionsList> PlanarRegionDataTopic = Root.child(PlanarRegion).topic(Data);
   public static final Topic<Boolean> ShowPlanarRegionsTopic = Root.child(PlanarRegion).topic(Show);

   public static final Topic<FootstepPlan> FootstepPlanTopic = Root.child(FootstepPlan).topic(Data);

   public static final Topic<Boolean> ComputePathTopic = Root.child(FootstepPlan).topic(ComputePath);
   public static final Topic<FootstepPlannerParameters> PlannerParametersTopic = Root.child(FootstepPlan).topic(FootstepPlannerParameters);
   public static final Topic<Double> PlannerTimeoutTopic = Root.child(FootstepPlan).topic(PlannerTimeout);
   public static final Topic<Double> PlannerHorizonLengthTopic = Root.child(FootstepPlan).topic(PlannerHorizonLength);
   public static final Topic<FootstepPlannerType> PlannerTypeTopic = Root.child(FootstepPlan).topic(FootstepPlannerType);
   public static final Topic<FootstepPlanningResult> PlanningResultTopic = Root.child(FootstepPlan).topic(FootstepPlannerResult);

   public static final Topic<Boolean> StartPositionEditModeEnabledTopic = Root.child(Start).child(EditMode).child(PositionTheme).topic(Enable);
   public static final Topic<Boolean> GoalPositionEditModeEnabledTopic = Root.child(Goal).child(EditMode).child(PositionTheme).topic(Enable);

   public static final Topic<Boolean> StartOrientationEditModeEnabledTopic = Root.child(Start).child(EditMode).child(OrientationTheme).topic(Enable);
   public static final Topic<Boolean> GoalOrientationEditModeEnabledTopic = Root.child(Goal).child(EditMode).child(OrientationTheme).topic(Enable);

   public static final Topic<RobotSide> InitialSupportSideTopic = Root.child(Start).topic(Side);
   public static final Topic<Point3D> StartPositionTopic = Root.child(Start).topic(Position);
   public static final Topic<Point3D> GoalPositionTopic = Root.child(Goal).topic(Position);

   public static final Topic<Quaternion> StartOrientationTopic = Root.child(Start).topic(Orientation);
   public static final Topic<Quaternion> GoalOrientationTopic = Root.child(Goal).topic(Orientation);

   public static final Topic<Boolean> GlobalResetTopic = Root.topic(Reset);

   public static final Topic<Boolean> EnableNodeChecking = Root.child(NodeChecking).topic(Enable);
   public static final Topic<Point3D> NodeCheckingPosition = Root.child(NodeChecking).topic(Position);
   public static final Topic<Quaternion> NodeCheckingOrientation = Root.child(NodeChecking).topic(Orientation);
   public static final Topic<Double> CliffHeight = Root.child(NodeChecking).topic(NodeCheckerCliffHeight);
   public static final Topic<Double> MinDistanceToCliff = Root.child(NodeChecking).topic(NodeCheckerCliffMinDistance);
   public static final Topic<Boolean> ValidNodeTopic = Root.child(NodeChecking).topic(ValidNode);
   public static final Topic<Pose3D> FootstepPoseTopic = Root.child(NodeChecking).topic(FootstepPose);

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}