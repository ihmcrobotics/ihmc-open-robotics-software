package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.commonWalkingControlModules.capturePoint.JumpingMomentumRateControlModule;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class JumpingControllerState extends HighLevelControllerState
{
   private final static HighLevelControllerName controllerState = HighLevelControllerName.WALKING;

   private final WholeBodyControllerCore controllerCore;
   private final JumpingMomentumRateControlModule momentumRateControlModule;
   private final JumpingHumanoidController jumpingController;

   private final ExecutionTimer controllerCoreTimer = new ExecutionTimer("controllerCoreTimer", 1.0, registry);

   private boolean requestIntegratorReset = false;
   private final YoBoolean yoRequestingIntegratorReset = new YoBoolean("RequestingIntegratorReset", registry);

   private final JumpingControllerToolbox controllerToolbox;

   public JumpingControllerState(JumpingControlManagerFactory managerFactory, JumpingControllerToolbox controllerToolbox,
                                 HighLevelControllerParameters highLevelControllerParameters, WalkingControllerParameters walkingControllerParameters,
                                 CoPTrajectoryParameters copTrajectoryParameters)
   {
      super(controllerState, highLevelControllerParameters, MultiBodySystemTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJoint.class));
      this.controllerToolbox = controllerToolbox;

      // create walking controller
      jumpingController = new JumpingHumanoidController(managerFactory, walkingControllerParameters, copTrajectoryParameters, controllerToolbox);

      // create controller core
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      JointBasics[] jointsToOptimizeFor = controllerToolbox.getControlledJoints();

      List<ContactablePlaneBody> contactableBodies = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
         contactableBodies.add(controllerToolbox.getContactableFeet().get(robotSide));

      FloatingJointBasics rootJoint = fullRobotModel.getRootJoint();
      ReferenceFrame centerOfMassFrame = controllerToolbox.getCenterOfMassFrame();
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controllerToolbox.getControlDT(), controllerToolbox.getGravityZ(), rootJoint,
                                                                            jointsToOptimizeFor, centerOfMassFrame,
                                                                            walkingControllerParameters.getMomentumOptimizationSettings(),
                                                                            controllerToolbox.getYoGraphicsListRegistry(), registry);
      toolbox.setJointPrivilegedConfigurationParameters(walkingControllerParameters.getJointPrivilegedConfigurationParameters());
      toolbox.setFeedbackControllerSettings(walkingControllerParameters.getFeedbackControllerSettings());
      toolbox.setupForInverseDynamicsSolver(contactableBodies);
      FeedbackControllerTemplate template = managerFactory.createFeedbackControlTemplate();
      // IMPORTANT: Cannot allow dynamic construction in a real-time environment such as this controller. This needs to be false.
      template.setAllowDynamicControllerConstruction(false);
      JointDesiredOutputList lowLevelControllerOutput = new JointDesiredOutputList(controlledJoints);
      controllerCore = new WholeBodyControllerCore(toolbox, template, lowLevelControllerOutput, registry);

      RigidBodyBasics elevator = fullRobotModel.getElevator();
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();

      momentumRateControlModule = new JumpingMomentumRateControlModule(referenceFrames, elevator, walkingControllerParameters, registry, yoGraphicsListRegistry);

      registry.addChild(jumpingController.getYoVariableRegistry());
   }

   public void initialize()
   {
      controllerCore.initialize();
      jumpingController.initialize();
      momentumRateControlModule.reset();
      requestIntegratorReset = true;
   }

   @Override
   public void doAction(double timeInState)
   {
      jumpingController.doAction();

      momentumRateControlModule.setInputFromWalkingStateMachine(jumpingController.getJumpingMomentumRateControlModuleInput());
      momentumRateControlModule.computeControllerCoreCommands();

      ControllerCoreCommand controllerCoreCommand = jumpingController.getControllerCoreCommand();
      controllerCoreCommand.addInverseDynamicsCommand(momentumRateControlModule.getMomentumRateCommand());

      JointDesiredOutputList stateSpecificJointSettings = getStateSpecificJointSettings();

      if (requestIntegratorReset)
      {
         stateSpecificJointSettings.requestIntegratorReset();
         requestIntegratorReset = false;
         yoRequestingIntegratorReset.set(true);
      }
      else
      {
         yoRequestingIntegratorReset.set(false);
      }

      controllerCoreCommand.addInverseDynamicsCommand(getAccelerationIntegrationCommand());
      controllerCoreCommand.completeLowLevelJointData(stateSpecificJointSettings);

      controllerCoreTimer.startMeasurement();
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();
      controllerCoreTimer.stopMeasurement();

      momentumRateControlModule.setInputFromControllerCore(controllerCore.getControllerCoreOutput());
      momentumRateControlModule.computeAchievedCMP();
   }

   @Override
   public void onEntry()
   {
      initialize();
   }

   @Override
   public void onExit()
   {
      controllerToolbox.reportChangeOfRobotMotionStatus(RobotMotionStatus.UNKNOWN);
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return controllerCore.getOutputForLowLevelController();
   }

   @Override
   public RootJointDesiredConfigurationDataReadOnly getOutputForRootJoint()
   {
      return controllerCore.getOutputForRootJoint();
   }

   @Override
   public boolean isJointLoadBearing(String jointName)
   {
      return jumpingController.isJointLoadBearing(jointName);
   }
}
