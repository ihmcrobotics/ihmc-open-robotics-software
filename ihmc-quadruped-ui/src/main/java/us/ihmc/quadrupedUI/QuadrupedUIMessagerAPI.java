package us.ihmc.quadrupedUI;

import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.QuadrupedBodyTrajectoryMessage;
import controller_msgs.msg.dds.QuadrupedFootstepStatusMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.*;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotModels.FullQuadrupedRobotModel;

public class QuadrupedUIMessagerAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final CategoryTheme Robot = apiFactory.createCategoryTheme("Robot");
   private static final CategoryTheme Controller = apiFactory.createCategoryTheme("Controller");
   private static final CategoryTheme Status = apiFactory.createCategoryTheme("Status");
   private static final CategoryTheme Command = apiFactory.createCategoryTheme("Command");
   private static final CategoryTheme BodyControl = apiFactory.createCategoryTheme("BodyControl");
   private static final CategoryTheme FootstepControl = apiFactory.createCategoryTheme("FootstepControl");
   private static final CategoryTheme Planning = apiFactory.createCategoryTheme("Planning");
   private static final CategoryTheme XGait = apiFactory.createCategoryTheme("XGait");


   private static final TypedTopicTheme<RobotConfigurationData> RobotConfigurationData = apiFactory.createTypedTopicTheme("RobotConfigurationData");
   private static final TypedTopicTheme<FullQuadrupedRobotModel> RobotModel = apiFactory.createTypedTopicTheme("RobotModel");
   private static final TypedTopicTheme<QuadrupedReferenceFrames> ReferenceFrames = apiFactory.createTypedTopicTheme("ReferenceFrames");
   private static final TypedTopicTheme<HighLevelControllerName> ControllerState = apiFactory.createTypedTopicTheme("ControllerState");
   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");
   private static final TypedTopicTheme<Double> BodyHeight = apiFactory.createTypedTopicTheme("BodyHeight");
   private static final TypedTopicTheme<QuadrupedBodyTrajectoryMessage> BodyTrajectoryMessage = apiFactory.createTypedTopicTheme("BodyTrajectoryMessage");
   private static final TypedTopicTheme<QuadrupedXGaitSettingsReadOnly> XGaitSettings = apiFactory.createTypedTopicTheme("XGaitSettings");
   private static final TypedTopicTheme<QuadrupedFootstepStatusMessage> FootstepStatusMessage = apiFactory.createTypedTopicTheme("FootstepStatusMessage");

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("QuadrupedUI"));

   public static final Topic<RobotConfigurationData> RobotConfigurationDataTopic = Root.child(Robot).topic(RobotConfigurationData);
   public static final Topic<FullQuadrupedRobotModel> RobotModelTopic = Root.child(Robot).topic(RobotModel);
   public static final Topic<QuadrupedReferenceFrames> ReferenceFramesTopic = Root.child(Robot).topic(ReferenceFrames);
   public static final Topic<HighLevelControllerName> CurrentControllerNameTopic = Root.child(Controller).child(Status).topic(ControllerState);
   public static final Topic<HighLevelControllerName> DesiredControllerNameTopic = Root.child(Command).topic(ControllerState);
   public static final Topic<Double> DesiredBodyHeightTopic = Root.child(Command).topic(BodyHeight);
   public static final Topic<QuadrupedBodyTrajectoryMessage> BodyTrajectoryMessageTopic = Root.child(Command).topic(BodyTrajectoryMessage);
   public static final Topic<Boolean> EnableBodyControlTopic = Root.child(Command).child(BodyControl).topic(Enable);
   public static final Topic<Boolean> EnableFootstepControlTopic = Root.child(Command).child(FootstepControl).topic(Enable);
   public static final Topic<QuadrupedXGaitSettingsReadOnly> XGaitSettingsTopic = Root.child(Planning).child(XGait).topic(XGaitSettings);
   public static final Topic<QuadrupedFootstepStatusMessage> FootstepStatusMessageTopic = Root.child(Controller).child(Status).topic(FootstepStatusMessage);

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();

}
