package us.ihmc.quadrupedUI;

import java.util.List;

import controller_msgs.msg.dds.QuadrupedFootstepStatusMessage;
import controller_msgs.msg.dds.QuadrupedTeleopDesiredPose;
import controller_msgs.msg.dds.QuadrupedTeleopDesiredVelocity;
import controller_msgs.msg.dds.QuadrupedTimedStepListMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.SoleTrajectoryMessage;
import controller_msgs.msg.dds.VideoPacket;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TopicTheme;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlan;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlannerStatus;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlannerTargetType;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlannerType;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlanningResult;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedUIMessagerAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final CategoryTheme Robot = apiFactory.createCategoryTheme("Robot");
   private static final CategoryTheme Controller = apiFactory.createCategoryTheme("Controller");
   private static final CategoryTheme Status = apiFactory.createCategoryTheme("Status");
   private static final CategoryTheme Command = apiFactory.createCategoryTheme("Command");
   private static final CategoryTheme BodyTeleop = apiFactory.createCategoryTheme("BodyTeleop");
   private static final CategoryTheme StepTeleop = apiFactory.createCategoryTheme("StepTeleop");
   private static final CategoryTheme HeightTeleop = apiFactory.createCategoryTheme("HeightTeleop");
   private static final CategoryTheme Joystick = apiFactory.createCategoryTheme("Joystick");
   private static final CategoryTheme Parameters = apiFactory.createCategoryTheme("Parameters");
   private static final CategoryTheme BodyPathPlanning = apiFactory.createCategoryTheme("BodyPathPlanning");
   private static final CategoryTheme FootstepPlanning = apiFactory.createCategoryTheme("FootstepPlanning");
   private static final CategoryTheme XGait = apiFactory.createCategoryTheme("XGait");
   private static final CategoryTheme Environment = apiFactory.createCategoryTheme("Environment");
   private static final CategoryTheme Review = apiFactory.createCategoryTheme("Review");
   private static final CategoryTheme Result = apiFactory.createCategoryTheme("Result");
   private static final CategoryTheme Edit = apiFactory.createCategoryTheme("Edit");
   private static final CategoryTheme PlanarRegions = apiFactory.createCategoryTheme("PlanarRegions");
   private static final CategoryTheme FlatGround = apiFactory.createCategoryTheme("FlatGround");
   private static final CategoryTheme Position = apiFactory.createCategoryTheme("Position");
   private static final CategoryTheme Orientation = apiFactory.createCategoryTheme("Orientation");
   private static final CategoryTheme Left = apiFactory.createCategoryTheme("Left");
   private static final CategoryTheme Right = apiFactory.createCategoryTheme("Right");
   private static final CategoryTheme Camera = apiFactory.createCategoryTheme("Camera");

   private static final CategoryTheme Start = apiFactory.createCategoryTheme("Start");
   private static final CategoryTheme LowLevelGoal = apiFactory.createCategoryTheme("LowLevelGoal");
   private static final CategoryTheme Goal = apiFactory.createCategoryTheme("Goal");


   private static final TopicTheme Type = apiFactory.createTopicTheme("Type");

   private static final TypedTopicTheme<Boolean> Reset = apiFactory.createTypedTopicTheme("Reset");
   private static final TypedTopicTheme<RobotConfigurationData> RobotConfigurationData = apiFactory.createTypedTopicTheme("RobotConfigurationData");
   private static final TypedTopicTheme<FullQuadrupedRobotModel> RobotModel = apiFactory.createTypedTopicTheme("RobotModel");
   private static final TypedTopicTheme<HighLevelControllerName> ControllerState = apiFactory.createTypedTopicTheme("ControllerState");
   private static final TypedTopicTheme<QuadrupedSteppingStateEnum> SteppingState = apiFactory.createTypedTopicTheme("SteppingState");
   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");
   private static final TypedTopicTheme<Boolean> Begin = apiFactory.createTypedTopicTheme("Start");
   private static final TypedTopicTheme<Boolean> Stop = apiFactory.createTypedTopicTheme("Stop");
   private static final TypedTopicTheme<Boolean> Pause = apiFactory.createTypedTopicTheme("Pause");
   private static final TypedTopicTheme<Double> DesiredBodyHeight = apiFactory.createTypedTopicTheme("BodyHeight");
   private static final TypedTopicTheme<QuadrupedXGaitSettingsReadOnly> XGaitSettings = apiFactory.createTypedTopicTheme("XGaitSettings");
   private static final TypedTopicTheme<FootstepPlannerParameters> FootstepPlannerParameters = apiFactory.createTypedTopicTheme("FootstepPlannerParameters");
   private static final TypedTopicTheme<VisibilityGraphsParameters> VisibilityGraphsParameters = apiFactory.createTypedTopicTheme("VisibilityGraphsParameters");
   private static final TypedTopicTheme<QuadrupedFootstepStatusMessage> FootstepStatusMessage = apiFactory.createTypedTopicTheme("FootstepStatusMessage");
   private static final TypedTopicTheme<QuadrupedTimedStepListMessage> StepsListMessage = apiFactory.createTypedTopicTheme("StepsListMessage");
   private static final TypedTopicTheme<SoleTrajectoryMessage> SoleTrajectoryMessage = apiFactory.createTypedTopicTheme("SoleTrajectoryMessage");
   private static final TypedTopicTheme<QuadrupedTeleopDesiredPose> DesiredTeleopBodyPoseMessage = apiFactory.createTypedTopicTheme("TeleopDesiredPose");
   private static final TypedTopicTheme<QuadrupedTeleopDesiredVelocity> DesiredTeleopVelocityMessage = apiFactory.createTypedTopicTheme("DesiredTeleopVelocityMessage");
   private static final TypedTopicTheme<RobotQuadrant> LoadBearingRequest = apiFactory.createTypedTopicTheme("LoadBearingRequest");

   private static final TypedTopicTheme<Boolean> Show = apiFactory.createTypedTopicTheme("Show");
   private static final TypedTopicTheme<Integer> Id = apiFactory.createTypedTopicTheme("Id");
   private static final TypedTopicTheme<PlanarRegion> PlanarRegion = apiFactory.createTypedTopicTheme("PlanarRegion");
   private static final TypedTopicTheme<PlanarRegionsList> PlanarRegionsList = apiFactory.createTypedTopicTheme("PlanarRegionsList");
   private static final TypedTopicTheme<FootstepPlanningResult> PlanningResult = apiFactory.createTypedTopicTheme("PlanningResult");
   private static final TypedTopicTheme<FootstepPlannerStatus> PlannerStatus = apiFactory.createTypedTopicTheme("PlannerStatus");
   private static final TypedTopicTheme<Number> Fraction = apiFactory.createTypedTopicTheme("Fraction");
   private static final TypedTopicTheme<Point3D> Point = apiFactory.createTypedTopicTheme("Point");
   private static final TypedTopicTheme<QuadrantDependentList<Point3D>> QuadrantPoint = apiFactory.createTypedTopicTheme("QuadrantPoint");
   private static final TypedTopicTheme<FootstepPlannerTargetType> FootstepPlannerTargetType = apiFactory.createTypedTopicTheme("FootstepPlannerTargetType");
   private static final TypedTopicTheme<Quaternion> Quaternion = apiFactory.createTypedTopicTheme("Quaternion");
   private static final TypedTopicTheme<RobotQuadrant> RobotQuadrant = apiFactory.createTypedTopicTheme("RobotQuadrant");
   private static final TypedTopicTheme<FootstepPlan> FootstepPlan = apiFactory.createTypedTopicTheme("FootstepPlan");
   private static final TypedTopicTheme<List<? extends Point3DReadOnly>> BodyPathPlan = apiFactory.createTypedTopicTheme("BodyPathPlan");
   private static final TypedTopicTheme<Double> Time = apiFactory.createTypedTopicTheme("Time");
   private static final TypedTopicTheme<Double> Length = apiFactory.createTypedTopicTheme("Length");
   private static final TypedTopicTheme<VideoPacket> Video = apiFactory.createTypedTopicTheme("Video");

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("QuadrupedUI"));

   /* Parameters */
   public static final Topic<QuadrupedXGaitSettingsReadOnly> XGaitSettingsTopic = Root.child(Parameters).child(XGait).topic(XGaitSettings);
   public static final Topic<FootstepPlannerParameters> FootstepPlannerParametersTopic = Root.child(Parameters).child(FootstepPlanning).topic(FootstepPlannerParameters);
   public static final Topic<VisibilityGraphsParameters> VisibilityGraphsParametersTopic = Root.child(Parameters).child(FootstepPlanning).topic(VisibilityGraphsParameters);

   public static final Topic<Boolean> GlobalResetTopic = Root.topic(Reset);

   public static final Topic<RobotConfigurationData> RobotConfigurationDataTopic = Root.child(Robot).topic(RobotConfigurationData);
   public static final Topic<FullQuadrupedRobotModel> RobotModelTopic = Root.child(Robot).topic(RobotModel);

   /* Environment */
   public static final Topic<Boolean> PlanarRegionDataClearTopic = Root.child(Environment).child(PlanarRegions).topic(Reset);
   public static final Topic<PlanarRegionsList> PlanarRegionDataTopic = Root.child(Environment).child(PlanarRegions).topic(PlanarRegionsList);
   public static final Topic<Boolean> AcceptNewPlanarRegionsTopic = Root.child(Environment).child(PlanarRegions).topic(Enable);
   public static final Topic<Boolean> AssumeFlatGroundTopic = Root.child(Environment).child(PlanarRegions).child(FlatGround).topic(Enable);
   public static final Topic<Boolean> ShowPlanarRegionsTopic = Root.child(Environment).child(PlanarRegions).topic(Show);
   public static final Topic<VideoPacket> LeftCameraVideo = Root.child(Camera).child(Left).topic(Video);
   public static final Topic<VideoPacket> RightCameraVideo = Root.child(Camera).child(Right).topic(Video);

   /* Status */
   public static final Topic<QuadrupedFootstepStatusMessage> FootstepStatusMessageTopic = Root.child(Controller).child(Status).topic(FootstepStatusMessage);
   public static final Topic<HighLevelControllerName> CurrentControllerNameTopic = Root.child(Controller).child(Status).topic(ControllerState);
   public static final Topic<HighLevelControllerName> DesiredControllerNameTopic = Root.child(Controller).child(Command).topic(ControllerState);
   public static final Topic<QuadrupedSteppingStateEnum> CurrentSteppingStateNameTopic = Root.child(Controller).child(Status).topic(SteppingState);
   public static final Topic<QuadrupedSteppingStateEnum> DesiredSteppingStateNameTopic = Root.child(Controller).child(Command).topic(SteppingState);

   /* Teleop */
   public static final Topic<Double> DesiredBodyHeightTopic = Root.child(Command).topic(DesiredBodyHeight);
   public static final Topic<Boolean> EnableBodyTeleopTopic = Root.child(Command).child(BodyTeleop).topic(Enable);
   public static final Topic<Boolean> EnableStepTeleopTopic = Root.child(Command).child(StepTeleop).topic(Enable);
   public static final Topic<Boolean> EnableHeightTeleopTopic = Root.child(Command).child(HeightTeleop).topic(Enable);
   public static final Topic<Boolean> EnableJoystickTopic = Root.child(Command).child(Joystick).topic(Enable);
   public static final Topic<QuadrupedTimedStepListMessage> ManualStepsListMessageTopic = Root.child(Command).child(StepTeleop).topic(StepsListMessage);
   public static final Topic<SoleTrajectoryMessage> SoleTrajectoryMessageTopic = Root.child(Command).child(StepTeleop).topic(SoleTrajectoryMessage);
   public static final Topic<Boolean> PauseWalkingTopic = Root.child(Command).child(StepTeleop).topic(Pause);
   public static final Topic<Boolean> AbortWalkingTopic = Root.child(Command).child(StepTeleop).topic(Stop);
   public static final Topic<RobotQuadrant> LoadBearingRequestTopic = Root.child(Command).child(StepTeleop).topic(LoadBearingRequest);

   public static final Topic<QuadrupedTeleopDesiredVelocity> DesiredTeleopVelocity = Root.child(Command).child(StepTeleop).topic(DesiredTeleopVelocityMessage);
   public static final Topic<QuadrupedTeleopDesiredPose> DesiredTeleopBodyPoseTopic = Root.child(Command).child(BodyTeleop).topic(DesiredTeleopBodyPoseMessage);

   /* Footstep Planning */
   public static final Topic<FootstepPlannerType> PlannerTypeTopic = Root.child(FootstepPlanning).child(Command).topic(Type);
   public static final Topic<Integer> PlannerRequestIdTopic = Root.child(FootstepPlanning).child(Command).topic(Id);
   public static final Topic<Integer> ReceivedPlanIdTopic = Root.child(FootstepPlanning).child(Result).topic(Id);
   public static final Topic<Boolean> ShowFootstepPlanTopic = Root.child(FootstepPlanning).child(Result).topic(Show);
   public static final Topic<PlanarRegion> SelectedRegionTopic = Root.child(FootstepPlanning).child(Edit).topic(PlanarRegion);

   public static final Topic<FootstepPlanningResult> PlanningResultTopic = Root.child(FootstepPlanning).child(Result).topic(PlanningResult);
   public static final Topic<FootstepPlannerStatus> PlannerStatusTopic = Root.child(FootstepPlanning).child(Status).topic(PlannerStatus);
   public static final Topic<FootstepPlan> FootstepPlanTopic = Root.child(FootstepPlanning).child(Result).topic(FootstepPlan);

   public static final Topic<Boolean> ShowFootstepPreviewTopic = Root.child(FootstepPlanning).child(Result).child(Review).topic(Show);
   public static final Topic<Number> PlannerPlaybackFractionTopic = Root.child(FootstepPlanning).child(Result).child(Review).topic(Fraction);
   public static final Topic<Point3D> StartPositionTopic = Root.child(FootstepPlanning).child(Command).child(Start).child(Position).topic(Point);
   public static final Topic<FootstepPlannerTargetType> StartTargetTypeTopic = Root.child(FootstepPlanning).child(Command).child(Start).child(Position).topic(FootstepPlannerTargetType);
   public static final Topic<QuadrantDependentList<Point3D>> StartFeetPositionTopic = Root.child(FootstepPlanning).child(Command).child(Start).child(Position).topic(QuadrantPoint);
   public static final Topic<Quaternion> StartOrientationTopic = Root.child(FootstepPlanning).child(Command).child(Start).child(Orientation).topic(Quaternion);
   public static final Topic<Point3D> LowLevelGoalPositionTopic = Root.child(FootstepPlanning).child(Command).child(LowLevelGoal).child(Position).topic(Point);
   public static final Topic<Quaternion> LowLevelGoalOrientationTopic = Root.child(FootstepPlanning).child(Command).child(LowLevelGoal).child(Orientation).topic(Quaternion);
   public static final Topic<RobotQuadrant> InitialSupportQuadrantTopic = Root.child(FootstepPlanning).child(Command).child(Start).topic(RobotQuadrant);
   public static final Topic<Point3D> GoalPositionTopic = Root.child(FootstepPlanning).child(Command).child(Goal).child(Position).topic(Point);
   public static final Topic<Quaternion> GoalOrientationTopic = Root.child(FootstepPlanning).child(Command).child(Goal).child(Orientation).topic(Quaternion);
   public static final Topic<Boolean> ComputePathTopic = Root.child(FootstepPlanning).child(Command).topic(Begin);
   public static final Topic<Boolean> AbortPlanningTopic = Root.child(FootstepPlanning).child(Command).topic(Stop);
   public static final Topic<Boolean> EditModeEnabledTopic = Root.child(FootstepPlanning).child(Edit).topic(Enable);
   public static final Topic<Boolean> StartPositionEditModeEnabledTopic = Root.child(FootstepPlanning).child(Start).child(Position).child(Edit).topic(Enable);
   public static final Topic<Boolean> StartOrientationEditModeEnabledTopic = Root.child(FootstepPlanning).child(Start).child(Orientation).child(Edit).topic(Enable);
   public static final Topic<Boolean> GoalPositionEditModeEnabledTopic = Root.child(FootstepPlanning).child(Goal).child(Position).child(Edit).topic(Enable);
   public static final Topic<Boolean> GoalOrientationEditModeEnabledTopic = Root.child(FootstepPlanning).child(Goal).child(Orientation).child(Edit).topic(Enable);
   public static final Topic<Double> PlannerTimeoutTopic = Root.child(FootstepPlanning).child(Command).topic(Time);
   public static final Topic<Double> PlannerHorizonLengthTopic = Root.child(FootstepPlanning).child(Command).topic(Length);
   public static final Topic<Double> PlannerTimeTakenTopic = Root.child(FootstepPlanning).child(Result).topic(Time);
   public static final Topic<QuadrupedTimedStepListMessage> FootstepPlannerTimedStepsTopic = Root.child(FootstepPlanning).child(Command).topic(StepsListMessage);

   /* Body Path Planning */
   public static final Topic<Boolean> ShowBodyPathTopic = Root.child(BodyPathPlanning).child(Result).topic(Show);
   public static final Topic<List<? extends Point3DReadOnly>> BodyPathDataTopic = Root.child(BodyPathPlanning).child(Result).topic(BodyPathPlan);



   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();

}
