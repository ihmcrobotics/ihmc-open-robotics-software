package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ArmDesiredAccelerationsControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ArmTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.EndEffectorLoadBearingControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.GoHomeControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HandComplianceControlParametersControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HandTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.StopAllTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.EndEffector;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.LoadBearingRequest;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

/**
 * @author twan
 *         Date: 5/13/13
 */
public class ManipulationControlModule
{
   public static final boolean HOLD_POSE_IN_JOINT_SPACE = true;
   private static final double TO_DEFAULT_CONFIGURATION_TRAJECTORY_TIME = 2.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final List<YoGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<YoGraphicReferenceFrame>();

   private final BooleanYoVariable hasBeenInitialized = new BooleanYoVariable("hasBeenInitialized", registry);
   private final SideDependentList<HandControlModule> handControlModules;

   private final FullHumanoidRobotModel fullRobotModel;

   public ManipulationControlModule(ArmControllerParameters armControllerParameters, MomentumBasedController momentumBasedController,
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

   public void handleEndEffectorLoadBearingMessage(EndEffectorLoadBearingControllerCommand message)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         LoadBearingRequest request = message.getRequest(robotSide, EndEffector.HAND);
         if (request == LoadBearingRequest.LOAD)
            handControlModules.get(robotSide).requestLoadBearing();
         else if (request == LoadBearingRequest.UNLOAD)
            handControlModules.get(robotSide).holdPositionInBase();
      }
   }

   public void handleStopAllTrajectoryMessage(StopAllTrajectoryControllerCommand message)
   {
      if (!message.isStopAllTrajectory())
         return;
      for (RobotSide robotSide : RobotSide.values)
         handControlModules.get(robotSide).holdPositionInJointSpace();
   }

   public void handleGoHomeMessage(GoHomeControllerCommand message)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (message.getRequest(robotSide, BodyPart.ARM))
            goToDefaultState(robotSide, message.getTrajectoryTime());
      }
   }

   public void handleHandTrajectoryMessages(List<HandTrajectoryControllerCommand> messages)
   {
      for (int i = 0; i < messages.size(); i++)
      {
         HandTrajectoryControllerCommand message = messages.get(i);
         RobotSide robotSide = message.getRobotSide();
         handControlModules.get(robotSide).handleHandTrajectoryMessage(message);
      }
   }

   public void handleArmTrajectoryMessages(List<ArmTrajectoryControllerCommand> messages)
   {
      for (int i = 0; i < messages.size(); i++)
      {
         ArmTrajectoryControllerCommand message = messages.get(i);
         RobotSide robotSide = message.getRobotSide();
         handControlModules.get(robotSide).handleArmTrajectoryMessage(message);
      }
   }

   public void handleArmDesiredAccelerationsMessages(List<ArmDesiredAccelerationsControllerCommand> messages)
   {
      for (int i = 0; i < messages.size(); i++)
      {
         ArmDesiredAccelerationsControllerCommand message = messages.get(i);
         RobotSide robotSide = message.getRobotSide();
         handControlModules.get(robotSide).handleArmDesiredAccelerationsMessage(message);
      }
   }

   public void handleHandComplianceControlParametersMessages(List<HandComplianceControlParametersControllerCommand> messages)
   {
      for (int i = 0; i < messages.size(); i++)
      {
         HandComplianceControlParametersControllerCommand message = messages.get(i);
         RobotSide robotSide = message.getRobotSide();
         handControlModules.get(robotSide).handleHandComplianceControlParametersMessage(message);
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
      handControlModules.get(robotSide).goToDefaultState(trajectoryTime);
   }

   public void initializeDesiredToCurrent()
   {
      hasBeenInitialized.set(true);
      for (RobotSide side : RobotSide.values)
      {
         handControlModules.get(side).holdPositionInJointSpace();
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
         handControlModule.holdPositionInJointSpace();
         handControlModule.resetJointIntegrators();
      }
   }

   public void holdCurrentArmConfiguration()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         holdArmCurrentConfiguration(handControlModules.get(robotSide));
      }
   }

   private void holdArmCurrentConfiguration(HandControlModule handControlModule)
   {
      if (handControlModule.isControllingPoseInWorld())
      {
         if (HOLD_POSE_IN_JOINT_SPACE)
            handControlModule.holdPositionInJointSpace();
         else
            handControlModule.holdPositionInBase();
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
