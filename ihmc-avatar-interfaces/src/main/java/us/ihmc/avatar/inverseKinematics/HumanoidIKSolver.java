package us.ihmc.avatar.inverseKinematics;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsSolutionQualityCalculator;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxOptimizationSettings;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointTorqueSoftLimitWeightCalculator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;

/**
 * Used for finding IK solutions for various sets of end effectors.
 * Can solve:
 * - Chest to one hand
 * - Chest to both hands
 * - Pelvis to one hand
 * - Pelvis to both hands
 * - Pelvis to one foot
 * - Pelvis to two feet
 *
 * For example, solving chest to one arm, pelvis to one
 *
 * Uses the WholeBodyControllerCore for solving.
 *
 * This class won't deal with stance feet i.e. controlling bodies to stay in the same place.
 * That's for the user to handle.
 *
 * TODO: Include ability to control chest joints angle to hold the chest
 */
public class HumanoidIKSolver
{
   /** The gains of the solver depend on this dt, it shouldn't change based on the actual thread scheduled period. */
   public static final double CONTROL_DT = 0.001;
   public static final double GRAVITY = 9.81;
   public static final double GOOD_QUALITY_MAX = 1.0;
   private static final int INVERSE_KINEMATICS_CALCULATIONS_PER_UPDATE = 70;

   private RigidBodyBasics workPelvis;
   private final OneDoFJointBasics[] sourceOneDoFJoints;
   private final OneDoFJointBasics[] workingOneDoFJoints;
   private final ReferenceFrame centerOfMassFrame;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   // TODO: Mess with these settings
   private final KinematicsToolboxOptimizationSettings optimizationSettings = new KinematicsToolboxOptimizationSettings();
   private final InverseKinematicsOptimizationSettingsCommand activeOptimizationSettings = new InverseKinematicsOptimizationSettingsCommand();
   private final WholeBodyControllerCore controllerCore;
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand();
   private final ArrayList<SpatialFeedbackControlCommand> spatialFeedbackControlCommands = new ArrayList<>();
   private final ArrayList<SpatialVelocityCommand> spatialVelocityCommands = new ArrayList<>();
   private final KinematicsSolutionQualityCalculator solutionQualityCalculator = new KinematicsSolutionQualityCalculator();
   private final FeedbackControllerDataHolderReadOnly feedbackControllerDataHolder;
   private double quality;

   private final SideDependentList<HumanoidIKSolverControlledBody> hands = new SideDependentList<>();
   private final HumanoidIKSolverControlledBody chest;
   private final HumanoidIKSolverControlledBody pelvis;
   private final SideDependentList<HumanoidIKSolverControlledBody> feet = new SideDependentList<>();

   public HumanoidIKSolver(HumanoidJointNameMap jointNameMap, FullHumanoidRobotModel sourceFullRobotModel)
   {
      sourceOneDoFJoints = MultiBodySystemMissingTools.getSubtreeJointArray(OneDoFJointBasics.class, sourceFullRobotModel.getPelvis());

      workPelvis = MultiBodySystemMissingTools.getDetachedCopyOfSubtreeWithElevator(sourceFullRobotModel.getPelvis());
      workingOneDoFJoints = MultiBodySystemMissingTools.getSubtreeJointArray(OneDoFJointBasics.class, workPelvis);
      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", ReferenceFrame.getWorldFrame(), workPelvis);

      YoGraphicsListRegistry yoGraphicsListRegistry = null; // opt out
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(CONTROL_DT,
                                                                            GRAVITY,
                                                                            (SixDoFJoint) workPelvis.getParentJoint(),
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

      for (RobotSide side : RobotSide.values)
         if (jointNameMap.getHandName(side) != null) // Account for one handed robots
            hands.set(side, HumanoidIKSolverControlledBody.createHand(workPelvis, sourceFullRobotModel, jointNameMap, side));
      chest = HumanoidIKSolverControlledBody.createChest(workPelvis, sourceFullRobotModel, jointNameMap);
      pelvis = HumanoidIKSolverControlledBody.createPelvis(workPelvis, sourceFullRobotModel, jointNameMap);
      for (RobotSide side : RobotSide.values)
         feet.set(side, HumanoidIKSolverControlledBody.createFoot(workPelvis, sourceFullRobotModel, jointNameMap, side));
   }

   public void copySourceToWork()
   {
      MultiBodySystemMissingTools.copyOneDoFJointsConfiguration(sourceOneDoFJoints, workingOneDoFJoints);
   }

   public SideDependentList<HumanoidIKSolverControlledBody> getHands()
   {
      return hands;
   }

   public SideDependentList<HumanoidIKSolverControlledBody> getFeet()
   {
      return feet;
   }

   public HumanoidIKSolverControlledBody getChest()
   {
      return chest;
   }

   public HumanoidIKSolverControlledBody getPelvis()
   {
      return pelvis;
   }

   /**
    * Solve for a zero-velocity solution configuration.
    */
   public void solve()
   {
      copySourceToWork();
      workPelvis.updateFramesRecursively();
      centerOfMassFrame.update();

      spatialFeedbackControlCommands.clear();
      for (RobotSide side : RobotSide.values)
      {
         if (hands.get(side).getActive())
            spatialFeedbackControlCommands.add(hands.get(side).buildSpatialFeedbackControlCommand());
         if (feet.get(side).getActive())
            spatialFeedbackControlCommands.add(feet.get(side).buildSpatialFeedbackControlCommand());
      }
      if (chest.getActive())
         spatialFeedbackControlCommands.add(chest.buildSpatialFeedbackControlCommand());
      if (pelvis.getActive())
         spatialFeedbackControlCommands.add(pelvis.buildSpatialFeedbackControlCommand());

      for (int i = 0; i < INVERSE_KINEMATICS_CALCULATIONS_PER_UPDATE; i++)
      {
         controllerCoreCommand.clear();
         controllerCoreCommand.addInverseKinematicsCommand(activeOptimizationSettings);
         for (SpatialFeedbackControlCommand spatialFeedbackControlCommand : spatialFeedbackControlCommands)
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

         workPelvis.updateFramesRecursively();
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
