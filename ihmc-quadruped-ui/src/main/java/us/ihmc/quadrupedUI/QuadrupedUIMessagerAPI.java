package us.ihmc.quadrupedUI;

import controller_msgs.msg.dds.QuadrupedFootstepStatusMessage;
import controller_msgs.msg.dds.QuadrupedTimedStepListMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.*;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlan;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlannerStatus;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlannerType;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlanningResult;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
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
   private static final CategoryTheme FootstepPlanning = apiFactory.createCategoryTheme("FootstepPlanning");
   private static final CategoryTheme XGait = apiFactory.createCategoryTheme("XGait");
   private static final CategoryTheme Environment = apiFactory.createCategoryTheme("Environment");
   private static final CategoryTheme Review = apiFactory.createCategoryTheme("Review");
   private static final CategoryTheme Result = apiFactory.createCategoryTheme("Result");
   private static final CategoryTheme Edit = apiFactory.createCategoryTheme("Edit");
   private static final CategoryTheme PlanarRegions = apiFactory.createCategoryTheme("PlanarRegions");
   private static final CategoryTheme FlatGround = apiFactory.createCategoryTheme("FlatGround");

   private static final CategoryTheme Start = apiFactory.createCategoryTheme("Start");
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
   private static final TypedTopicTheme<Double> DesiredBodyHeight = apiFactory.createTypedTopicTheme("BodyHeight");
   private static final TypedTopicTheme<QuadrupedXGaitSettingsReadOnly> XGaitSettings = apiFactory.createTypedTopicTheme("XGaitSettings");
   private static final TypedTopicTheme<QuadrupedFootstepStatusMessage> FootstepStatusMessage = apiFactory.createTypedTopicTheme("FootstepStatusMessage");
   private static final TypedTopicTheme<QuadrupedTimedStepListMessage> StepsListMessage = apiFactory.createTypedTopicTheme("StepsListMessage");

   private static final TypedTopicTheme<Boolean> Show = apiFactory.createTypedTopicTheme("Show");
   private static final TypedTopicTheme<Integer> Id = apiFactory.createTypedTopicTheme("Id");
   private static final TypedTopicTheme<PlanarRegionsList> PlanarRegionsList = apiFactory.createTypedTopicTheme("PlanarRegionsList");
   private static final TypedTopicTheme<FootstepPlanningResult> PlanningResult = apiFactory.createTypedTopicTheme("PlanningResult");
   private static final TypedTopicTheme<FootstepPlannerStatus> PlannerStatus = apiFactory.createTypedTopicTheme("PlannerStatus");
   private static final TypedTopicTheme<Number> Fraction = apiFactory.createTypedTopicTheme("Fraction");
   private static final TypedTopicTheme<Point3D> Position = apiFactory.createTypedTopicTheme("Position");
   private static final TypedTopicTheme<Quaternion> Orientation = apiFactory.createTypedTopicTheme("Orientation");
   private static final TypedTopicTheme<RobotQuadrant> RobotQuadrant = apiFactory.createTypedTopicTheme("RobotQuadrant");
   private static final TypedTopicTheme<FootstepPlan> FootstepPlan = apiFactory.createTypedTopicTheme("FootstepPlan");
   private static final TypedTopicTheme<Double> Time = apiFactory.createTypedTopicTheme("Time");
   private static final TypedTopicTheme<Double> Length = apiFactory.createTypedTopicTheme("Length");

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("QuadrupedUI"));

   public static final Topic<QuadrupedXGaitSettingsReadOnly> XGaitSettingsTopic = Root.child(Parameters).child(XGait).topic(XGaitSettings);

   public static final Topic<Boolean> GlobalResetTopic = Root.topic(Reset);

   public static final Topic<RobotConfigurationData> RobotConfigurationDataTopic = Root.child(Robot).topic(RobotConfigurationData);
   public static final Topic<FullQuadrupedRobotModel> RobotModelTopic = Root.child(Robot).topic(RobotModel);

   /* Environment */
   public static final Topic<PlanarRegionsList> PlanarRegionDataTopic = Root.child(Environment).child(PlanarRegions).topic(PlanarRegionsList);
   public static final Topic<Boolean> AcceptNewPlanarRegionsTopic = Root.child(Environment).child(PlanarRegions).topic(Enable);
   public static final Topic<Boolean> AssumeFlatGroundTopic = Root.child(Environment).child(PlanarRegions).child(FlatGround).topic(Enable);


   /* Status */
   public static final Topic<QuadrupedFootstepStatusMessage> FootstepStatusMessageTopic = Root.child(Controller).child(Status).topic(FootstepStatusMessage);
   public static final Topic<HighLevelControllerName> CurrentControllerNameTopic = Root.child(Controller).child(Status).topic(ControllerState);
   public static final Topic<HighLevelControllerName> DesiredControllerNameTopic = Root.child(Controller).child(Command).topic(ControllerState);
   public static final Topic<QuadrupedSteppingStateEnum> DesiredSteppingStateNameTopic = Root.child(Controller).child(Command).topic(SteppingState);

   /* Teleop */
   public static final Topic<Double> DesiredBodyHeightTopic = Root.child(Command).topic(DesiredBodyHeight);
   public static final Topic<Boolean> EnableBodyTeleopTopic = Root.child(Command).child(BodyTeleop).topic(Enable);
   public static final Topic<Boolean> EnableStepTeleopTopic = Root.child(Command).child(StepTeleop).topic(Enable);
   public static final Topic<Boolean> EnableHeightTeleopTopic = Root.child(Command).child(HeightTeleop).topic(Enable);
   public static final Topic<Boolean> EnableJoystickTopic = Root.child(Command).child(Joystick).topic(Enable);
   public static final Topic<QuadrupedTimedStepListMessage> ManualStepsListMessageTopic = Root.child(Command).child(StepTeleop).topic(StepsListMessage);



   /* Footstep Planning */
   public static final Topic<FootstepPlannerType> PlannerTypeTopic = Root.child(FootstepPlanning).child(Command).topic(Type);
   public static final Topic<Integer> PlannerRequestIdTopic = Root.child(FootstepPlanning).child(Command).topic(Id);
   public static final Topic<Integer> ReceivedPlanIdTopic = Root.child(FootstepPlanning).child(Result).topic(Id);
   public static final Topic<Boolean> ShowFootstepPlanTopic = Root.child(FootstepPlanning).child(Result).topic(Show);

   public static final Topic<FootstepPlanningResult> PlanningResultTopic = Root.child(FootstepPlanning).child(Result).topic(PlanningResult);
   public static final Topic<FootstepPlannerStatus> PlannerStatusTopic = Root.child(FootstepPlanning).child(Status).topic(PlannerStatus);
   public static final Topic<FootstepPlan> FootstepPlanTopic = Root.child(FootstepPlanning).child(Result).topic(FootstepPlan);

   public static final Topic<Boolean> ShowFootstepPreviewTopic = Root.child(FootstepPlanning).child(Result).child(Review).topic(Show);
   public static final Topic<Number> PlannerPlaybackFractionTopic = Root.child(FootstepPlanning).child(Result).child(Review).topic(Fraction);
   public static final Topic<Point3D> StartPositionTopic = Root.child(FootstepPlanning).child(Command).child(Start).topic(Position);
   public static final Topic<Quaternion> StartOrientationTopic = Root.child(FootstepPlanning).child(Command).child(Start).topic(Orientation);
   public static final Topic<RobotQuadrant> InitialSupportQuadrantTopic = Root.child(FootstepPlanning).child(Command).child(Start).topic(RobotQuadrant);
   public static final Topic<Point3D> GoalPositionTopic = Root.child(FootstepPlanning).child(Command).child(Goal).topic(Position);
   public static final Topic<Quaternion> GoalOrientationTopic = Root.child(FootstepPlanning).child(Command).child(Goal).topic(Orientation);
   public static final Topic<Boolean> ComputePathTopic = Root.child(FootstepPlanning).child(Command).topic(Begin);
   public static final Topic<Boolean> AbortPlanningTopic = Root.child(FootstepPlanning).child(Command).topic(Stop);
   public static final Topic<Boolean> EditModeEnabledTopic = Root.child(FootstepPlanning).child(Edit).topic(Enable);
   public static final Topic<Boolean> StartPositionEditModeEnabledTopic = Root.child(FootstepPlanning).child(Start).child(Edit).topic(Enable);
   public static final Topic<Boolean> GoalPositionEditModeEnabledTopic = Root.child(FootstepPlanning).child(Goal).child(Edit).topic(Enable);
   public static final Topic<Double> PlannerTimeoutTopic = Root.child(FootstepPlanning).child(Command).topic(Time);
   public static final Topic<Double> PlannerHorizonLengthTopic = Root.child(FootstepPlanning).child(Command).topic(Length);
   public static final Topic<Double> PlannerTimeTakenTopic = Root.child(FootstepPlanning).child(Result).topic(Time);



   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();

}
