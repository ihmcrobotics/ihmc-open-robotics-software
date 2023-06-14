package us.ihmc.avatar.inverseKinematics;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsSolutionQualityCalculator;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxOptimizationSettings;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.*;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointTorqueSoftLimitWeightCalculator;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
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
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.wholeBodyController.HandTransformTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

/**
 * An attempt to use the WholeBodyControllerCore directly to get IK solutions for
 * a humanoid robot arm.
 *
 * TODO:
 *   - Collision constraints
 *
 * @author Duncan Calvert
 */
public class ArmIKSolver
{
   /** The gains of the solver depend on this dt, it shouldn't change based on the actual thread scheduled period. */
   public static final double CONTROL_DT = 0.001;
   public static final double GRAVITY = 9.81;
   private static final int INVERSE_KINEMATICS_CALCULATIONS_PER_UPDATE = 10;
   private static final double GLOBAL_PROPORTIONAL_GAIN = 1200.0;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final ReferenceFrame armWorldFrame = ReferenceFrame.getWorldFrame();
   private final RigidBodyBasics workChest;
   private final RigidBodyBasics workHand;
   // TODO: Mess with these settings
   private final KinematicsToolboxOptimizationSettings optimizationSettings = new KinematicsToolboxOptimizationSettings();
   private final InverseKinematicsOptimizationSettingsCommand activeOptimizationSettings = new InverseKinematicsOptimizationSettingsCommand();
   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final WholeBodyControllerCore controllerCore;
   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand();
   private final DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
   private final WeightMatrix6D weightMatrix = new WeightMatrix6D();
   private final ReferenceFrame handControlDesiredFrame;
   private final FramePose3D handControlDesiredPose = new FramePose3D();
   private final RigidBodyTransform handControlDesiredPoseToChestCoMTransform = new RigidBodyTransform();
   private final SpatialVectorReadOnly zeroVector6D = new SpatialVector(armWorldFrame);
   private final FramePose3D controlFramePose = new FramePose3D();
   private final OneDoFJointBasics[] syncedOneDoFJoints;
   private final OneDoFJointBasics[] workingOneDoFJoints;
   private final OneDoFJointBasics[] solutionOneDoFJoints;
   private final KinematicsSolutionQualityCalculator solutionQualityCalculator = new KinematicsSolutionQualityCalculator();
   private final FeedbackControllerDataHolderReadOnly feedbackControllerDataHolder;
   private final double totalRobotMass;
   private final RigidBodyBasics syncedChest;

   public ArmIKSolver(RobotSide side,
                      DRCRobotModel robotModel,
                      FullHumanoidRobotModel syncedRobot,
                      FullHumanoidRobotModel solutionRobot,
                      ReferenceFrame handControlDesiredFrame)
   {
      this.handControlDesiredFrame = handControlDesiredFrame;
      syncedChest = syncedRobot.getChest();
      OneDoFJointBasics syncedFirstArmJoint = syncedRobot.getArmJoint(side, robotModel.getJointMap().getArmJointNames()[0]);
      syncedOneDoFJoints = FullRobotModelUtils.getArmJoints(syncedRobot, side, robotModel.getJointMap().getArmJointNames());
      solutionOneDoFJoints = FullRobotModelUtils.getArmJoints(solutionRobot, side, robotModel.getJointMap().getArmJointNames());

      workChest = MultiBodySystemMissingTools.getDetachedCopyOfSubtree(syncedChest, armWorldFrame, syncedFirstArmJoint);
      SixDoFJoint rootSixDoFJoint = null;
      totalRobotMass = TotalMassCalculator.computeSubTreeMass(workChest);

      // Remove fingers
      workHand = MultiBodySystemTools.findRigidBody(workChest, robotModel.getJointMap().getHandName(side));
      workHand.getChildrenJoints().clear();

      RigidBodyTransform handControlToBodyFixedCoMFrameTransform = new RigidBodyTransform();
      HandTransformTools.getHandControlToBodyFixedCoMFrameTransform(syncedRobot, side, handControlToBodyFixedCoMFrameTransform);
      controlFramePose.setToZero(workHand.getBodyFixedFrame());
      controlFramePose.set(handControlToBodyFixedCoMFrameTransform);

      JointBasics[] controlledJoints = MultiBodySystemMissingTools.getSubtreeJointArray(JointBasics.class, workChest);

      CenterOfMassReferenceFrame centerOfMassFrame = null; // we don't need this
      YoGraphicsListRegistry yoGraphicsListRegistry = null; // opt out
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(CONTROL_DT,
                                                                            GRAVITY,
                                                                            rootSixDoFJoint,
                                                                            controlledJoints,
                                                                            centerOfMassFrame,
                                                                            optimizationSettings,
                                                                            yoGraphicsListRegistry,
                                                                            registry);
      toolbox.setJointPrivilegedConfigurationParameters(new JointPrivilegedConfigurationParameters());
      JointTorqueSoftLimitWeightCalculator jointTorqueMinimizationWeightCalculator = new JointTorqueSoftLimitWeightCalculator(toolbox.getJointIndexHandler());
      jointTorqueMinimizationWeightCalculator.setParameters(0.0, 0.001, 0.10);
      toolbox.setupForInverseKinematicsSolver(jointTorqueMinimizationWeightCalculator);

      List<RigidBodyBasics> controllableRigidBodies = new ArrayList<>();
      controllableRigidBodies.add(workChest);
      controllableRigidBodies.add(workHand);

      workingOneDoFJoints = MultiBodySystemMissingTools.getSubtreeJointArray(OneDoFJointBasics.class, workChest);

      FeedbackControllerTemplate controllerCoreTemplate = new FeedbackControllerTemplate();
      controllerCoreTemplate.setAllowDynamicControllerConstruction(true);
      controllerCoreTemplate.enableCenterOfMassFeedbackController();

      int numberOfControllersPerBody = 1;
      controllableRigidBodies.forEach(rigidBody -> controllerCoreTemplate.enableSpatialFeedbackController(rigidBody, numberOfControllersPerBody));

      SubtreeStreams.fromChildren(OneDoFJointBasics.class, workChest).forEach(controllerCoreTemplate::enableOneDoFJointFeedbackController);

      JointDesiredOutputList lowLevelControllerOutput = new JointDesiredOutputList(workingOneDoFJoints);
      controllerCore = new WholeBodyControllerCore(toolbox, controllerCoreTemplate, lowLevelControllerOutput, registry);
      feedbackControllerDataHolder = controllerCore.getWholeBodyFeedbackControllerDataHolder();

      controllerCoreCommand.setControllerCoreMode(WholeBodyControllerCoreMode.INVERSE_KINEMATICS);

      gains.setPositionProportionalGains(50.0);
      gains.setOrientationProportionalGains(50.0);
      weightMatrix.setLinearWeights(20.0, 20.0, 20.0);
      weightMatrix.setAngularWeights(20.0, 20.0, 20.0);
      selectionMatrix.setLinearAxisSelection(true, true, true);
      selectionMatrix.setAngularAxisSelection(true, true, true);

      privilegedConfigurationCommand.setDefaultWeight(0.025);
      privilegedConfigurationCommand.setDefaultConfigurationGain(50.0);

      for (OneDoFJointBasics workingOneDoFJoint : workingOneDoFJoints)
      {
         privilegedConfigurationCommand.addJoint(workingOneDoFJoint, workingOneDoFJoint.getName().contains("ELBOW") ? 1.04 : 0.0);
      }
   }

   public void copyActualToWork()
   {
      MultiBodySystemMissingTools.copyOneDoFJointsConfiguration(syncedOneDoFJoints, workingOneDoFJoints);
   }

   public void solve()
   {
      double gain = 1200.0;
      double weight = 20.0;
      gains.setPositionProportionalGains(gain);
      gains.setOrientationProportionalGains(gain);
      weightMatrix.setLinearWeights(weight, weight, weight);
      weightMatrix.setAngularWeights(weight, weight, weight);

      // Get the hand desired pose, but put it in the world of the detached arm
      handControlDesiredPose.setToZero(handControlDesiredFrame);
      handControlDesiredPose.changeFrame(syncedChest.getParentJoint().getFrameAfterJoint());
      handControlDesiredPose.get(handControlDesiredPoseToChestCoMTransform);

      workChest.getBodyFixedFrame().getTransformToParent().transform(handControlDesiredPoseToChestCoMTransform);

      handControlDesiredPose.setToZero(armWorldFrame);
      handControlDesiredPose.set(handControlDesiredPoseToChestCoMTransform);

      copyActualToWork();
      workChest.updateFramesRecursively();

      double quality = 0.0;
      for (int i = 0; i < INVERSE_KINEMATICS_CALCULATIONS_PER_UPDATE; i++)
      {
         controllerCoreCommand.clear();

         controllerCoreCommand.addInverseKinematicsCommand(activeOptimizationSettings);

         controllerCoreCommand.addInverseKinematicsCommand(privilegedConfigurationCommand);

         spatialFeedbackControlCommand.set(workChest, workHand);
         spatialFeedbackControlCommand.setGains(gains);
         spatialFeedbackControlCommand.setSelectionMatrix(selectionMatrix);
         spatialFeedbackControlCommand.setWeightMatrixForSolver(weightMatrix);
         spatialFeedbackControlCommand.setInverseKinematics(handControlDesiredPose, zeroVector6D);
         spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(controlFramePose);
         controllerCoreCommand.addFeedbackControlCommand(spatialFeedbackControlCommand);

         controllerCore.compute(controllerCoreCommand);

         quality = solutionQualityCalculator.calculateSolutionQuality(feedbackControllerDataHolder, totalRobotMass, 1.0 / GLOBAL_PROPORTIONAL_GAIN);

         ControllerCoreOutput controllerCoreOutput = controllerCore.getControllerCoreOutput();

         TDoubleArrayList angles = new TDoubleArrayList();

         JointDesiredOutputListReadOnly output = controllerCoreOutput.getLowLevelOneDoFJointDesiredDataHolder();
         for (int j = 0; j < workingOneDoFJoints.length; j++)
         {
            if (output.hasDataForJoint(workingOneDoFJoints[j]))
            {
               JointDesiredOutputReadOnly jointDesiredOutput = output.getJointDesiredOutput(workingOneDoFJoints[j]);
               solutionOneDoFJoints[j].setQ(jointDesiredOutput.getDesiredPosition());
               angles.add(jointDesiredOutput.getDesiredPosition());
               solutionOneDoFJoints[j].setQd(jointDesiredOutput.getDesiredVelocity());
               workingOneDoFJoints[j].setQ(jointDesiredOutput.getDesiredPosition());
               workingOneDoFJoints[j].setQd(jointDesiredOutput.getDesiredVelocity());
               if (jointDesiredOutput.hasDesiredTorque())
               {
                  solutionOneDoFJoints[j].setTau(jointDesiredOutput.getDesiredTorque());
                  workingOneDoFJoints[j].setTau(jointDesiredOutput.getDesiredTorque());
               }
            }
         }
         workChest.updateFramesRecursively();
      }
   }

   public void copyWorkToDesired()
   {
      MultiBodySystemMissingTools.copyOneDoFJointsConfiguration(workingOneDoFJoints, solutionOneDoFJoints);
   }

   public void setDesiredToCurrent()
   {
      MultiBodySystemMissingTools.copyOneDoFJointsConfiguration(syncedOneDoFJoints, solutionOneDoFJoints);
   }
}
