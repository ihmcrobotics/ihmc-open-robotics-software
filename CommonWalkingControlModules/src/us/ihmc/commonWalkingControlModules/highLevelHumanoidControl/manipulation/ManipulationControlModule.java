package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmDesiredAccelerationsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EndEffectorLoadBearingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandComplianceControlParametersCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.EndEffector;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.LoadBearingRequest;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * @author twan
 *         Date: 5/13/13
 */
public class ManipulationControlModule
{
   public static final boolean HOLD_POSE_IN_JOINT_SPACE = true;
   public static final double TO_DEFAULT_CONFIGURATION_TRAJECTORY_TIME = 2.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final List<YoGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<YoGraphicReferenceFrame>();

   private final BooleanYoVariable hasBeenInitialized = new BooleanYoVariable("hasBeenInitialized", registry);
   private final SideDependentList<HandControlModule> handControlModules;

   private final FullHumanoidRobotModel fullRobotModel;

   public ManipulationControlModule(ArmControllerParameters armControllerParameters, HighLevelHumanoidControllerToolbox momentumBasedController,
         YoVariableRegistry parentRegistry)
   {
      fullRobotModel = momentumBasedController.getFullRobotModel();

      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      createFrameVisualizers(yoGraphicsListRegistry, fullRobotModel, "HandControlFrames", true);

      handControlModules = new SideDependentList<HandControlModule>();

      YoPIDGains jointspaceControlGains = armControllerParameters.createJointspaceControlGains(registry);
      YoSE3PIDGainsInterface taskspaceGains = armControllerParameters.createTaskspaceControlGains(registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         HandControlModule individualHandControlModule = new HandControlModule(robotSide, momentumBasedController, armControllerParameters,
               jointspaceControlGains, taskspaceGains, registry);
         handControlModules.put(robotSide, individualHandControlModule);
      }

      parentRegistry.addChild(registry);
   }

   public void setWeights(double jointspaceWeight, double taskspaceWeight, double userControlModeWeight)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         HandControlModule handControlModule = handControlModules.get(robotSide);
         handControlModule.setJointspaceWeight(jointspaceWeight);
         handControlModule.setTaskspaceWeight(taskspaceWeight);
         handControlModule.setUserModeWeight(userControlModeWeight);
      }
   }

   public void setWeights(double jointspaceWeight, Vector3D angularTaskspaceWeight, Vector3D linearTaskspaceWeight, double userControlModeWeight)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         HandControlModule handControlModule = handControlModules.get(robotSide);
         handControlModule.setJointspaceWeight(jointspaceWeight);
         handControlModule.setTaskspaceWeights(angularTaskspaceWeight, linearTaskspaceWeight);
         handControlModule.setUserModeWeight(userControlModeWeight);
      }
   }

   private void createFrameVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry, FullHumanoidRobotModel fullRobotModel, String listName, boolean enable)
   {
      YoGraphicsList list = new YoGraphicsList(listName);
      if (yoGraphicsListRegistry != null)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            ReferenceFrame handPositionControlFrame = fullRobotModel.getHandControlFrame(robotSide);
            if (handPositionControlFrame != null)
            {
               YoGraphicReferenceFrame dynamicGraphicReferenceFrame = new YoGraphicReferenceFrame(handPositionControlFrame, registry, 0.1);
               dynamicGraphicReferenceFrames.add(dynamicGraphicReferenceFrame);
               list.add(dynamicGraphicReferenceFrame);
            }
         }
         yoGraphicsListRegistry.registerYoGraphicsList(list);

         if (!enable)
            list.hideYoGraphics();
      }
   }

   public void initialize()
   {
      doControl();
   }

   public void doControl()
   {
      // Important especially when switching between high level states. In such case, we don't want the arm to go to home position
      if (!hasBeenInitialized.getBooleanValue())
      {
         for (RobotSide robotSide : RobotSide.values)
            handControlModules.get(robotSide).initialize();
         goToDefaultState();
         hasBeenInitialized.set(true);
      }

      updateGraphics();

      for (RobotSide robotSide : RobotSide.values)
      {
         handControlModules.get(robotSide).doControl();
      }
   }

   public void handleEndEffectorLoadBearingCommand(EndEffectorLoadBearingCommand command)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         LoadBearingRequest request = command.getRequest(robotSide, EndEffector.HAND);
         if (request == LoadBearingRequest.LOAD)
            handControlModules.get(robotSide).requestLoadBearing();
         else if (request == LoadBearingRequest.UNLOAD)
            handControlModules.get(robotSide).holdPositionInJointspace();
      }
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      if (!command.isStopAllTrajectory())
         return;
      for (RobotSide robotSide : RobotSide.values)
         handControlModules.get(robotSide).holdPositionInJointspace();
   }

   public void handleHandTrajectoryCommands(List<HandTrajectoryCommand> commands)
   {
      for (int i = 0; i < commands.size(); i++)
      {
         HandTrajectoryCommand command = commands.get(i);
         RobotSide robotSide = command.getRobotSide();
         handControlModules.get(robotSide).handleHandTrajectoryCommand(command);
      }
   }

   public void handleArmTrajectoryCommands(List<ArmTrajectoryCommand> commands)
   {
      for (int i = 0; i < commands.size(); i++)
      {
         ArmTrajectoryCommand command = commands.get(i);
         RobotSide robotSide = command.getRobotSide();
         handControlModules.get(robotSide).handleArmTrajectoryCommand(command);
      }
   }

   public void handleArmDesiredAccelerationsCommands(List<ArmDesiredAccelerationsCommand> commands)
   {
      for (int i = 0; i < commands.size(); i++)
      {
         ArmDesiredAccelerationsCommand command = commands.get(i);
         RobotSide robotSide = command.getRobotSide();
         handControlModules.get(robotSide).handleArmDesiredAccelerationsCommand(command);
      }
   }

   public void handleHandComplianceControlParametersCommands(List<HandComplianceControlParametersCommand> commands)
   {
      for (int i = 0; i < commands.size(); i++)
      {
         HandComplianceControlParametersCommand command = commands.get(i);
         RobotSide robotSide = command.getRobotSide();
         handControlModules.get(robotSide).handleHandComplianceControlParametersCommand(command);
      }
   }

   public void goToDefaultState()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         goToDefaultState(robotSide, TO_DEFAULT_CONFIGURATION_TRAJECTORY_TIME);
      }
   }

   public void goToDefaultState(RobotSide robotSide, double trajectoryTime)
   {
      handControlModules.get(robotSide).goHome(trajectoryTime);
   }

   public void initializeDesiredToCurrent()
   {
      hasBeenInitialized.set(true);
      for (RobotSide side : RobotSide.values)
      {
         handControlModules.get(side).holdPositionInJointspace();
      }
   }

   public void prepareForLocomotion()
   {
      holdCurrentArmConfiguration();
   }

   public void freeze()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         HandControlModule handControlModule = handControlModules.get(robotSide);
         handControlModule.holdPositionInJointspace();
         handControlModule.resetJointIntegrators();
      }
   }

   public void holdCurrentArmConfiguration()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         holdArmCurrentConfiguration(robotSide);
      }
   }

   private void holdArmCurrentConfiguration(RobotSide robotSide)
   {
      HandControlModule handControlModule = handControlModules.get(robotSide);
      if (handControlModule.isControllingPoseInWorld())
      {
         if (HOLD_POSE_IN_JOINT_SPACE)
            handControlModule.holdPositionInJointspace();
         else
            handControlModule.holdPositionInChest();
      }
   }

   private void updateGraphics()
   {
      for (int i = 0; i < dynamicGraphicReferenceFrames.size(); i++)
      {
         dynamicGraphicReferenceFrames.get(i).update();
      }
   }

   public boolean isAtLeastOneHandLoadBearing()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (handControlModules.get(robotSide).isLoadBearing())
            return true;
      }
      return false;
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand(RobotSide robotSide)
   {
      return handControlModules.get(robotSide).getInverseDynamicsCommand();
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand(RobotSide robotSide)
   {
      return handControlModules.get(robotSide).getFeedbackControlCommand();
   }

   public LowLevelOneDoFJointDesiredDataHolderReadOnly getLowLevelJointDesiredData(RobotSide robotSide)
   {
      return handControlModules.get(robotSide).getLowLevelJointDesiredData();
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      for (RobotSide robotSide : RobotSide.values)
      {
         FeedbackControlCommandList template = handControlModules.get(robotSide).createFeedbackControlTemplate();
         for (int i = 0; i < template.getNumberOfCommands(); i++)
            ret.addCommand(template.getCommand(i));
      }
      return ret;
   }
}
