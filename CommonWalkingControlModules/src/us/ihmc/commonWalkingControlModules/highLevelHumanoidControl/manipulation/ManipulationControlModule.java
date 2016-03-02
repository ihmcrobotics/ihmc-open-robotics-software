package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableArmDesiredAccelerationsMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableArmTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableGoHomeMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableHandComplianceControlParametersMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableHandTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.packetConsumers.EndEffectorLoadBearingMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.StopAllTrajectoryMessageSubscriber;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.EndEffector;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.LoadBearingRequest;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.lists.RecyclingArrayList;
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

   private final EndEffectorLoadBearingMessageSubscriber effectorLoadBearingMessageSubscriber;
   private final StopAllTrajectoryMessageSubscriber stopAllTrajectoryMessageSubscriber;

   public ManipulationControlModule(VariousWalkingProviders variousWalkingProviders, ArmControllerParameters armControllerParameters,
         MomentumBasedController momentumBasedController, YoVariableRegistry parentRegistry)
   {
      fullRobotModel = momentumBasedController.getFullRobotModel();

      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      createFrameVisualizers(yoGraphicsListRegistry, fullRobotModel, "HandControlFrames", true);

      effectorLoadBearingMessageSubscriber = variousWalkingProviders.getEndEffectorLoadBearingMessageSubscriber();
      stopAllTrajectoryMessageSubscriber = variousWalkingProviders.getStopAllTrajectoryMessageSubscriber();

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
         handleLoadBearing(robotSide);
         handleStopAllTrajectoryMessages(robotSide);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         handControlModules.get(robotSide).doControl();
      }
   }

   private void handleLoadBearing(RobotSide robotSide)
   {
      if (effectorLoadBearingMessageSubscriber != null)
      {
         LoadBearingRequest request = effectorLoadBearingMessageSubscriber.pollMessage(EndEffector.HAND, robotSide);

         if (request == LoadBearingRequest.LOAD)
         {
            handControlModules.get(robotSide).requestLoadBearing();
         }
         else if (request == LoadBearingRequest.UNLOAD)
         {
            handControlModules.get(robotSide).holdPositionInBase();
         }
      }
   }

   private void handleStopAllTrajectoryMessages(RobotSide robotSide)
   {
      if (stopAllTrajectoryMessageSubscriber == null)
         return;

      HandControlModule handControlModule = handControlModules.get(robotSide);
      if (stopAllTrajectoryMessageSubscriber.pollMessage(handControlModule))
         handControlModule.holdPositionInJointSpace();
   }

   public void handleGoHomeMessage(ModifiableGoHomeMessage message)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (message.getRequest(robotSide, BodyPart.ARM))
            goToDefaultState(robotSide, message.getTrajectoryTime());
      }
   }

   public void handleHandTrajectoryMessages(RecyclingArrayList<ModifiableHandTrajectoryMessage> messages)
   {
      for (int i = 0; i < messages.size(); i++)
      {
         ModifiableHandTrajectoryMessage message = messages.get(i);
         RobotSide robotSide = message.getRobotSide();
         handControlModules.get(robotSide).handleHandTrajectoryMessage(message);
      }
   }

   public void handleArmTrajectoryMessages(RecyclingArrayList<ModifiableArmTrajectoryMessage> messages)
   {
      for (int i = 0; i < messages.size(); i++)
      {
         ModifiableArmTrajectoryMessage message = messages.get(i);
         RobotSide robotSide = message.getRobotSide();
         handControlModules.get(robotSide).handleArmTrajectoryMessage(message);
      }
   }

   public void handleArmDesiredAccelerationsMessages(RecyclingArrayList<ModifiableArmDesiredAccelerationsMessage> messages)
   {
      for (int i = 0; i < messages.size(); i++)
      {
         ModifiableArmDesiredAccelerationsMessage message = messages.get(i);
         RobotSide robotSide = message.getRobotSide();
         handControlModules.get(robotSide).handleArmDesiredAccelerationsMessage(message);
      }
   }

   public void handleHandComplianceControlParametersMessages(RecyclingArrayList<ModifiableHandComplianceControlParametersMessage> messages)
   {
      for (int i = 0; i < messages.size(); i++)
      {
         ModifiableHandComplianceControlParametersMessage message = messages.get(i);
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
