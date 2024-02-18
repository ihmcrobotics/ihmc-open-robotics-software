package us.ihmc.avatar.inverseKinematics;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsSolutionQualityCalculator;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxOptimizationSettings;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.*;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointTorqueSoftLimitWeightCalculator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Uses the WholeBodyControllerCore directly to get IK solutions for
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
   public static final double GOOD_QUALITY_MAX = 1.0;
   private static final int INVERSE_KINEMATICS_CALCULATIONS_PER_UPDATE = 70;
   public static final double DEFAULT_POSITION_GAIN = 1200.0;
   public static final double DEFAULT_POSITION_WEIGHT = 20.0;
   public static final double DEFAULT_ORIENTATION_GAIN = 100.0;
   public static final double DEFAULT_ORIENTATION_WEIGHT = 1.0;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final RigidBodyBasics workChest;
   private final HumanoidIKSolverControlledBody hand;
   // TODO: Mess with these settings
   private final KinematicsToolboxOptimizationSettings optimizationSettings = new KinematicsToolboxOptimizationSettings();
   private final InverseKinematicsOptimizationSettingsCommand activeOptimizationSettings = new InverseKinematicsOptimizationSettingsCommand();
   private final WholeBodyControllerCore controllerCore;
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand();
   private final OneDoFJointBasics[] sourceOneDoFJoints;
   private final OneDoFJointBasics[] workingOneDoFJoints;
   private final KinematicsSolutionQualityCalculator solutionQualityCalculator = new KinematicsSolutionQualityCalculator();
   private final FeedbackControllerDataHolderReadOnly feedbackControllerDataHolder;
   private double quality;

   /**
    * @param sourceFullRobotModel The robot model to clone the joint tree from and copy initial values from in {@link #copySourceToWork}
    */
   public ArmIKSolver(RobotSide side, HumanoidJointNameMap jointNameMap, FullHumanoidRobotModel sourceFullRobotModel)
   {
      RigidBodyBasics sourceChest = sourceFullRobotModel.getChest();
      OneDoFJointBasics sourceFirstArmJoint = sourceFullRobotModel.getArmJoint(side, jointNameMap.getArmJointNames()[0]);
      ArmJointName[] armJointNames = jointNameMap.getArmJointNames(side);
      sourceOneDoFJoints = FullRobotModelUtils.getArmJoints(sourceFullRobotModel, side, armJointNames);

      // We clone a detached chest and single arm for the WBCC to work with. We just want to find arm joint angles.
      workChest = MultiBodySystemMissingTools.getDetachedCopyOfSubtree(sourceChest,
                                                                       ReferenceFrame.getWorldFrame(),
                                                                       sourceFirstArmJoint,
                                                                       sourceFullRobotModel.getHand(side).getName());

      hand = HumanoidIKSolverControlledBody.createHand(workChest, sourceFullRobotModel, jointNameMap, side);
      // Remove fingers
      hand.getWorkBody().getChildrenJoints().clear();

      workingOneDoFJoints = MultiBodySystemMissingTools.getSubtreeJointArray(OneDoFJointBasics.class, workChest);

      SixDoFJoint rootSixDoFJoint = null; // don't need this for fixed base single limb IK
      CenterOfMassReferenceFrame centerOfMassFrame = null; // we don't need this
      YoGraphicsListRegistry yoGraphicsListRegistry = null; // opt out
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(CONTROL_DT,
                                                                            GRAVITY,
                                                                            rootSixDoFJoint,
                                                                            workingOneDoFJoints,
                                                                            centerOfMassFrame,
                                                                            optimizationSettings,
                                                                            yoGraphicsListRegistry,
                                                                            registry);

      JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters = new JointPrivilegedConfigurationParameters();
      toolbox.setJointPrivilegedConfigurationParameters(jointPrivilegedConfigurationParameters);

      JointTorqueSoftLimitWeightCalculator jointTorqueMinimizationWeightCalculator = new JointTorqueSoftLimitWeightCalculator(toolbox.getJointIndexHandler());
      jointTorqueMinimizationWeightCalculator.setParameters(0.0, 0.001, 0.10);
      toolbox.setupForInverseKinematicsSolver(jointTorqueMinimizationWeightCalculator);

      // No reason to preallocate our feedback controllers here
      FeedbackControllerTemplate controllerCoreTemplate = new FeedbackControllerTemplate();
      controllerCoreTemplate.setAllowDynamicControllerConstruction(true);

      JointDesiredOutputList lowLevelControllerOutput = new JointDesiredOutputList(workingOneDoFJoints);

      controllerCore = new WholeBodyControllerCore(toolbox, controllerCoreTemplate, lowLevelControllerOutput, registry);

      feedbackControllerDataHolder = controllerCore.getWholeBodyFeedbackControllerDataHolder();

      controllerCoreCommand.setControllerCoreMode(WholeBodyControllerCoreMode.INVERSE_KINEMATICS);
   }

   public void copySourceToWork()
   {
      MultiBodySystemMissingTools.copyOneDoFJointsConfiguration(sourceOneDoFJoints, workingOneDoFJoints);
   }

   /**
    * Computes joint angles from given chest frame such that the hand control frame
    * matches the given hand control frame, which is usually the palm.
    */
   public void update(ReferenceFrame chestFrame, ReferenceFrame handControlDesiredFrame)
   {
      hand.update(chestFrame, handControlDesiredFrame);
   }

   public boolean getDesiredHandControlPoseChanged()
   {
      return hand.getDesiredBodyControlPoseChanged();
   }

   public void solve(FrameVector3DReadOnly desiredAngularVelocity, FrameVector3DReadOnly desiredLinearVelocity)
   {
      // Perform the position only solution which iterates many times
      solve();

      // Populate the spatial velocity for the IK command list
      SpatialVelocityCommand spatialVelocityCommand = hand.buildSpatialVelocityCommand(desiredAngularVelocity, desiredLinearVelocity);

      // Populate the commands list with the settings and spatial velocity
      controllerCoreCommand.clear();
      controllerCoreCommand.addInverseKinematicsCommand(activeOptimizationSettings);
      controllerCoreCommand.addInverseKinematicsCommand(spatialVelocityCommand);

      // Use this to compute the desired velocity.
      controllerCore.compute(controllerCoreCommand);

      // Feed the solution velocity back into the working model and compute once
      ControllerCoreOutput controllerCoreOutput = controllerCore.getControllerCoreOutput();
      JointDesiredOutputListReadOnly output = controllerCoreOutput.getLowLevelOneDoFJointDesiredDataHolder();
      for (int j = 0; j < workingOneDoFJoints.length; j++)
      {
         if (output.hasDataForJoint(workingOneDoFJoints[j]))
         {
            JointDesiredOutputReadOnly jointDesiredOutput = output.getJointDesiredOutput(workingOneDoFJoints[j]);
            double desiredVelocity = jointDesiredOutput.getDesiredVelocity();
            workingOneDoFJoints[j].setQd(desiredVelocity);
            if (jointDesiredOutput.hasDesiredTorque())
            {
               workingOneDoFJoints[j].setTau(jointDesiredOutput.getDesiredTorque());
            }
         }
      }
   }

   public void solve()
   {
      copySourceToWork();
      workChest.updateFramesRecursively();

      SpatialFeedbackControlCommand spatialFeedbackControlCommand = hand.buildSpatialFeedbackControlCommand();

      for (int i = 0; i < INVERSE_KINEMATICS_CALCULATIONS_PER_UPDATE; i++)
      {
         controllerCoreCommand.clear();
         controllerCoreCommand.addInverseKinematicsCommand(activeOptimizationSettings);
         controllerCoreCommand.addFeedbackControlCommand(spatialFeedbackControlCommand);

         controllerCore.compute(controllerCoreCommand);

         // Feed the solution output back into the working joints to be the start state of the next compute.
         ControllerCoreOutput controllerCoreOutput = controllerCore.getControllerCoreOutput();
         JointDesiredOutputListReadOnly output = controllerCoreOutput.getLowLevelOneDoFJointDesiredDataHolder();
         boolean jointIsMoving = false;
         for (int j = 0; j < workingOneDoFJoints.length; j++)
         {
            if (output.hasDataForJoint(workingOneDoFJoints[j]))
            {
               JointDesiredOutputReadOnly jointDesiredOutput = output.getJointDesiredOutput(workingOneDoFJoints[j]);
               double desiredVelocity = jointDesiredOutput.getDesiredVelocity();
               // we are wanting to move, so things haven't converged
               if (Math.abs(desiredVelocity) > 1e-3)
                  jointIsMoving = true;
               // we need to integrate the velocity to get the desired poistion
               double integratedPosition = workingOneDoFJoints[j].getQ() + CONTROL_DT * desiredVelocity;
               // if the desired position is vastly different than the one integrated internally, don't it, and fall back to the output of the IK.
               if (AngleTools.computeAngleDifferenceMinusPiToPi(integratedPosition, jointDesiredOutput.getDesiredPosition()) > Math.toRadians(45.0))
                  integratedPosition = jointDesiredOutput.getDesiredPosition();
               workingOneDoFJoints[j].setQ(integratedPosition);
               workingOneDoFJoints[j].setQd(desiredVelocity);
               if (jointDesiredOutput.hasDesiredTorque())
               {
                  workingOneDoFJoints[j].setTau(jointDesiredOutput.getDesiredTorque());
               }
            }
         }

         // none of the joints are moving, so terminate the search early.
         if (!jointIsMoving)
            break;

         workChest.updateFramesRecursively();
      }

      double totalRobotMass = 0.0; // We don't need this parameter
      quality = solutionQualityCalculator.calculateSolutionQuality(feedbackControllerDataHolder, totalRobotMass, 1.0);
      if (quality > GOOD_QUALITY_MAX)
      {
         LogTools.debug("Bad quality solution: {} Try upping the gains, giving more iteration, or setting a more acheivable goal.", quality);
      }
   }

   public double getQuality()
   {
      return quality;
   }

   public OneDoFJointBasics[] getSolutionOneDoFJoints()
   {
      return workingOneDoFJoints;
   }
}
