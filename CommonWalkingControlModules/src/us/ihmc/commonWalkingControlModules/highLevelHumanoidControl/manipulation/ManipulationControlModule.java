package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableArmTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableHandTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.packetConsumers.ArmDesiredAccelerationsMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.EndEffectorLoadBearingMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.GoHomeMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandComplianceControlParametersSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.StopAllTrajectoryMessageSubscriber;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.EndEffector;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.LoadBearingRequest;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
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

   private final ArmDesiredAccelerationsMessageSubscriber armDesiredAccelerationsMessageSubscriber;
   private final EndEffectorLoadBearingMessageSubscriber effectorLoadBearingMessageSubscriber;
   private final StopAllTrajectoryMessageSubscriber stopAllTrajectoryMessageSubscriber;
   private final HandComplianceControlParametersSubscriber handComplianceControlParametersSubscriber;
   private final GoHomeMessageSubscriber goHomeMessageSubscriber;

   private final BooleanYoVariable isIgnoringInputs = new BooleanYoVariable("isManipulationIgnoringInputs", registry);
   private final DoubleYoVariable startTimeForIgnoringInputs = new DoubleYoVariable("startTimeForIgnoringManipulationInputs", registry);
   private final DoubleYoVariable durationForIgnoringInputs = new DoubleYoVariable("durationForIgnoringManipulationInputs", registry);

   private final DoubleYoVariable yoTime;

   public ManipulationControlModule(VariousWalkingProviders variousWalkingProviders, ArmControllerParameters armControllerParameters,
         MomentumBasedController momentumBasedController, YoVariableRegistry parentRegistry)
   {
      fullRobotModel = momentumBasedController.getFullRobotModel();
      this.yoTime = momentumBasedController.getYoTime();

      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      createFrameVisualizers(yoGraphicsListRegistry, fullRobotModel, "HandControlFrames", true);

      armDesiredAccelerationsMessageSubscriber = variousWalkingProviders.getArmDesiredAccelerationsMessageSubscriber();
      effectorLoadBearingMessageSubscriber = variousWalkingProviders.getEndEffectorLoadBearingMessageSubscriber();
      stopAllTrajectoryMessageSubscriber = variousWalkingProviders.getStopAllTrajectoryMessageSubscriber();
      handComplianceControlParametersSubscriber = variousWalkingProviders.getHandComplianceControlParametersSubscriber();
      goHomeMessageSubscriber = variousWalkingProviders.getGoHomeMessageSubscriber();

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

      if (yoTime.getDoubleValue() - startTimeForIgnoringInputs.getDoubleValue() < durationForIgnoringInputs.getDoubleValue())
      {
         isIgnoringInputs.set(true);
         goHomeMessageSubscriber.clearMessagesInQueue();
      }
      else
      {
         isIgnoringInputs.set(false);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         handleLoadBearing(robotSide);
         handleArmDesiredAccelerationsMessages(robotSide);
         handleStopAllTrajectoryMessages(robotSide);
         handleGoHomeMessages(robotSide);

         handleCompliantControlRequests(robotSide);
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

   private void handleArmDesiredAccelerationsMessages(RobotSide robotSide)
   {
      if (armDesiredAccelerationsMessageSubscriber == null || !armDesiredAccelerationsMessageSubscriber.isNewControlMessageAvailable(robotSide))
         return;

      ArmDesiredAccelerationsMessage message = armDesiredAccelerationsMessageSubscriber.pollMessage(robotSide);
      handControlModules.get(robotSide).handleArmDesiredAccelerationsMessage(message);
   }

   private void handleStopAllTrajectoryMessages(RobotSide robotSide)
   {
      if (stopAllTrajectoryMessageSubscriber == null)
         return;

      HandControlModule handControlModule = handControlModules.get(robotSide);
      if (stopAllTrajectoryMessageSubscriber.pollMessage(handControlModule))
         handControlModule.holdPositionInJointSpace();
   }

   private void handleGoHomeMessages(RobotSide robotSide)
   {
      if (goHomeMessageSubscriber == null)
         return;

      if (goHomeMessageSubscriber.isNewMessageAvailable(BodyPart.ARM, robotSide))
         goToDefaultState(robotSide, goHomeMessageSubscriber.pollMessage(BodyPart.ARM, robotSide));
   }

   private void handleCompliantControlRequests(RobotSide robotSide)
   {
      if (handComplianceControlParametersSubscriber != null && handComplianceControlParametersSubscriber.checkForNewRequest(robotSide))
      {
         if (handComplianceControlParametersSubscriber.isResetRequested(robotSide))
         {
            handControlModules.get(robotSide).setEnableCompliantControl(false, null, null, null, null, Double.NaN, Double.NaN);
         }
         else
         {
            boolean[] enableLinearCompliance = handComplianceControlParametersSubscriber.getEnableLinearCompliance(robotSide);
            boolean[] enableAngularCompliance = handComplianceControlParametersSubscriber.getEnableAngularCompliance(robotSide);
            Vector3d desiredForce = handComplianceControlParametersSubscriber.getDesiredForce(robotSide);
            Vector3d desiredTorque = handComplianceControlParametersSubscriber.getDesiredTorque(robotSide);
            double forceDeadzone = handComplianceControlParametersSubscriber.getForceDeadzone(robotSide);
            double torqueDeadzone = handComplianceControlParametersSubscriber.getTorqueDeadzone(robotSide);
            handControlModules.get(robotSide).setEnableCompliantControl(true, enableLinearCompliance, enableAngularCompliance, desiredForce, desiredTorque,
                  forceDeadzone, torqueDeadzone);
         }
      }
   }

   public void handleHandTrajectoryMessage(ModifiableHandTrajectoryMessage handTrajectoryMessage)
   {
      RobotSide robotSide = handTrajectoryMessage.getRobotSide();
      handControlModules.get(robotSide).handleHandTrajectoryMessage(handTrajectoryMessage);
   }

   public void handleArmTrajectoryMessage(ModifiableArmTrajectoryMessage armTrajectoryMessage)
   {
      RobotSide robotSide = armTrajectoryMessage.getRobotSide();
      handControlModules.get(robotSide).handleArmTrajectoryMessage(armTrajectoryMessage);
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

   public void ignoreInputsForGivenDuration(double duration)
   {
      startTimeForIgnoringInputs.set(yoTime.getDoubleValue());
      durationForIgnoringInputs.set(duration);
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
