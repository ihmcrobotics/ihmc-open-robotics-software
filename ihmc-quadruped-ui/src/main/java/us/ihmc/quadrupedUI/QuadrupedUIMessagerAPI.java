package us.ihmc.quadrupedUI;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.*;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotModels.FullQuadrupedRobotModel;

import javax.naming.ldap.Control;

public class QuadrupedUIMessagerAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final CategoryTheme Robot = apiFactory.createCategoryTheme("Robot");
   private static final CategoryTheme Controller = apiFactory.createCategoryTheme("Controller");
   private static final CategoryTheme Command = apiFactory.createCategoryTheme("Command");

   private static final TypedTopicTheme<RobotConfigurationData> RobotConfigurationData = apiFactory.createTypedTopicTheme("RobotConfigurationData");
   private static final TypedTopicTheme<FullQuadrupedRobotModel> RobotModel = apiFactory.createTypedTopicTheme("RobotModel");
   private static final TypedTopicTheme<QuadrupedReferenceFrames> ReferenceFrames = apiFactory.createTypedTopicTheme("ReferenceFrames");
   private static final TypedTopicTheme<HighLevelControllerName> ControllerState = apiFactory.createTypedTopicTheme("ControllerState");

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("QuadrupedUI"));

   public static final Topic<RobotConfigurationData> RobotConfigurationDataTopic = Root.child(Robot).topic(RobotConfigurationData);
   public static final Topic<FullQuadrupedRobotModel> RobotModelTopic = Root.child(Robot).topic(RobotModel);
   public static final Topic<QuadrupedReferenceFrames> ReferenceFramesTopic = Root.child(Robot).topic(ReferenceFrames);
   public static final Topic<HighLevelControllerName> CurrentControllerNameTopic = Root.child(Controller).topic(ControllerState);
   public static final Topic<HighLevelControllerName> DesiredControllerNameTopic = Root.child(Command).topic(ControllerState);

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();

}
