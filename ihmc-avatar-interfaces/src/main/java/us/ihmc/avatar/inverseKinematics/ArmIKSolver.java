package us.ihmc.avatar.inverseKinematics;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxOptimizationSettings;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointTorqueSoftLimitWeightCalculator;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

/**
 * An attempt to use the WholeBodyControllerCore directly to get IK solutions for
 * a humanoid robot arm.
 *
 * TODO:
 *   - Collision constraints
 *   - Resetting from updated synced robot
 *
 * @author Duncan Calvert
 */
public class ArmIKSolver
{
   /** The gains of the solver depend on this dt, it shouldn't change based on the actual thread scheduled period. */
   public static final double CONTROL_DT = 0.001;
   public static final double GRAVITY = 9.81;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final RigidBodyBasics chest;
   private final RigidBodyBasics hand;
   private final OneDoFJointBasics[] oneDoFJoints;
   // TODO: Mess with these settings
   private final KinematicsToolboxOptimizationSettings optimizationSettings = new KinematicsToolboxOptimizationSettings();
   private final WholeBodyControllerCore controllerCore;
   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand();
   private final DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
   private final WeightMatrix6D weightMatrix = new WeightMatrix6D();
   private final ReferenceFrame handControlDesiredFrame;
   private final FramePose3D handControlDesiredPose = new FramePose3D();
   private final SpatialVectorReadOnly zeroVector6D = new SpatialVector(ReferenceFrame.getWorldFrame());
   private final FramePose3D controlFramePose = new FramePose3D();
   private final FullHumanoidRobotModel solutionRobot;
   private final OneDoFJointBasics[] solutionOneDoFJoints;
   private final ModifiableReferenceFrame centerOfMassFrame;

   public ArmIKSolver(RobotSide side,
                      DRCRobotModel robotModel,
                      FullHumanoidRobotModel syncedRobot,
                      FullHumanoidRobotModel solutionRobot,
                      ReferenceFrame handControlDesiredFrame)
   {
      this.solutionRobot = solutionRobot;
      this.handControlDesiredFrame = handControlDesiredFrame;
      RigidBodyBasics syncedChest = syncedRobot.getChest();
      OneDoFJointBasics syncedFirstArmJoint = syncedRobot.getArmJoint(side, robotModel.getJointMap().getArmJointNames()[0]);
      solutionOneDoFJoints = FullRobotModelUtils.getArmJoints(syncedRobot, side, robotModel.getJointMap().getArmJointNames());

      RigidBody detachedArmOnlyRobot = MultiBodySystemMissingTools.getDetachedCopyOfSubtree(syncedChest, syncedFirstArmJoint);
      SixDoFJoint rootSixDoFJoint = (SixDoFJoint) detachedArmOnlyRobot.getChildrenJoints().get(0);

      // Remove fingers
      hand = MultiBodySystemTools.findRigidBody(detachedArmOnlyRobot, robotModel.getJointMap().getHandName(side));
      hand.getChildrenJoints().clear();

      chest = MultiBodySystemTools.findRigidBody(detachedArmOnlyRobot, robotModel.getJointMap().getChestName());

      JointBasics[] controlledJoints = MultiBodySystemMissingTools.getSubtreeJointArray(JointBasics.class, detachedArmOnlyRobot);

      centerOfMassFrame = new ModifiableReferenceFrame();
      YoGraphicsListRegistry yoGraphicsListRegistry = null; // opt out
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(CONTROL_DT,
                                                                            GRAVITY,
                                                                            rootSixDoFJoint,
                                                                            controlledJoints,
                                                                            centerOfMassFrame.getReferenceFrame(),
                                                                            optimizationSettings,
                                                                            yoGraphicsListRegistry,
                                                                            registry);
      toolbox.setJointPrivilegedConfigurationParameters(new JointPrivilegedConfigurationParameters());
      JointTorqueSoftLimitWeightCalculator jointTorqueMinimizationWeightCalculator = new JointTorqueSoftLimitWeightCalculator(toolbox.getJointIndexHandler());
      jointTorqueMinimizationWeightCalculator.setParameters(0.0, 0.001, 0.10);
      toolbox.setupForInverseKinematicsSolver(jointTorqueMinimizationWeightCalculator);

      List<RigidBodyBasics> controllableRigidBodies = new ArrayList<>();
      controllableRigidBodies.add(chest);
      controllableRigidBodies.add(hand);

      oneDoFJoints = MultiBodySystemMissingTools.getSubtreeJointArray(OneDoFJointBasics.class, detachedArmOnlyRobot);

      FeedbackControllerTemplate controllerCoreTemplate = new FeedbackControllerTemplate();
      controllerCoreTemplate.setAllowDynamicControllerConstruction(true);
      controllerCoreTemplate.enableCenterOfMassFeedbackController();

      int numberOfControllersPerBody = 1;
      controllableRigidBodies.stream().forEach(rigidBody -> controllerCoreTemplate.enableSpatialFeedbackController(rigidBody, numberOfControllersPerBody));

      SubtreeStreams.fromChildren(OneDoFJointBasics.class, chest).forEach(controllerCoreTemplate::enableOneDoFJointFeedbackController);

      JointDesiredOutputList lowLevelControllerOutput = new JointDesiredOutputList(oneDoFJoints);
      controllerCore = new WholeBodyControllerCore(toolbox, controllerCoreTemplate, lowLevelControllerOutput, registry);

      controllerCoreCommand.setControllerCoreMode(WholeBodyControllerCoreMode.INVERSE_KINEMATICS);

      LogTools.info("Created WBCC!");
   }

   public void update()
   {
      controllerCoreCommand.clear();

      centerOfMassFrame.update(transformToParent -> chest.getBodyFixedFrame().getTransformToRoot());

      handControlDesiredPose.setToZero(handControlDesiredFrame);
      handControlDesiredPose.changeFrame(ReferenceFrame.getWorldFrame());

      controlFramePose.setToZero(hand.getBodyFixedFrame());

      spatialFeedbackControlCommand.set(chest, hand);
      spatialFeedbackControlCommand.setGains(gains);
      spatialFeedbackControlCommand.setSelectionMatrix(selectionMatrix);
      spatialFeedbackControlCommand.setWeightMatrixForSolver(weightMatrix);
      spatialFeedbackControlCommand.setInverseKinematics(handControlDesiredPose, zeroVector6D);
      spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(controlFramePose);
      spatialFeedbackControlCommand.setPrimaryBase(chest);

      controllerCoreCommand.addFeedbackControlCommand(spatialFeedbackControlCommand);


      controllerCore.compute(controllerCoreCommand);

      // TODO: Calculate solution quality

      ControllerCoreOutput controllerCoreOutput = controllerCore.getControllerCoreOutput();

      JointDesiredOutputListReadOnly output = controllerCoreOutput.getLowLevelOneDoFJointDesiredDataHolder();
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         if (output.hasDataForJoint(oneDoFJoints[i]))
         {
            JointDesiredOutputReadOnly jointDesiredOutput = output.getJointDesiredOutput(oneDoFJoints[i]);
            solutionOneDoFJoints[i].setQ(jointDesiredOutput.getDesiredPosition());
         }
      }

   }
}
