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
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointTorqueSoftLimitWeightCalculator;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
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
 * A humanoid whole body IK solver using the WholeBodyControllerCore for solving.
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

   private final RigidBodyBasics workElevator;
   private final SixDoFJoint workPelvisSixDoFJoint;
   private final FramePose3D pelvisFramePose = new FramePose3D();
   private final OneDoFJointBasics[] sourceOneDoFJoints;
   private final OneDoFJointBasics[] workingOneDoFJoints;
   private final ReferenceFrame centerOfMassFrame;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   // TODO: Mess with these settings
   private final KinematicsToolboxOptimizationSettings optimizationSettings = new KinematicsToolboxOptimizationSettings();
   private final InverseKinematicsOptimizationSettingsCommand activeOptimizationSettings = new InverseKinematicsOptimizationSettingsCommand();
   private final WholeBodyControllerCore controllerCore;
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand();
   private final CenterOfMassFeedbackControlCommand centerOfMassFeedbackControlCommand = new CenterOfMassFeedbackControlCommand();
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

      workElevator = MultiBodySystemMissingTools.getDetachedCopyOfSubtreeWithElevator(sourceFullRobotModel.getPelvis());
      workPelvisSixDoFJoint = (SixDoFJoint) workElevator.getChildrenJoints().get(0);
      workingOneDoFJoints = MultiBodySystemMissingTools.getSubtreeJointArray(OneDoFJointBasics.class, workElevator);
      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", ReferenceFrame.getWorldFrame(), workElevator);

      YoGraphicsListRegistry yoGraphicsListRegistry = null; // opt out
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(CONTROL_DT,
                                                                            GRAVITY,
                                                                            workPelvisSixDoFJoint,
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
            hands.set(side, HumanoidIKSolverControlledBody.createHand(workElevator, sourceFullRobotModel, jointNameMap, side));
      chest = HumanoidIKSolverControlledBody.createChest(workElevator, sourceFullRobotModel, jointNameMap);
      pelvis = HumanoidIKSolverControlledBody.createPelvis(workElevator, sourceFullRobotModel, jointNameMap);
      for (RobotSide side : RobotSide.values)
         feet.set(side, HumanoidIKSolverControlledBody.createFoot(workElevator, sourceFullRobotModel, jointNameMap, side));
   }

   public void copySourceToWork(ReferenceFrame sourcePelvisFrame)
   {
      pelvisFramePose.setFromReferenceFrame(sourcePelvisFrame);
      workPelvisSixDoFJoint.setJointConfiguration(pelvisFramePose);
      MultiBodySystemMissingTools.copyOneDoFJointsConfiguration(sourceOneDoFJoints, workingOneDoFJoints);
   }

   /**
    * Solve for a zero-velocity solution configuration.
    */
   public void solve()
   {
      workElevator.updateFramesRecursively();

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
         centerOfMassFrame.update();

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

         workElevator.updateFramesRecursively();
      }

      double totalRobotMass = 0.0; // We don't need this parameter
      quality = solutionQualityCalculator.calculateSolutionQuality(feedbackControllerDataHolder, totalRobotMass, 1.0);
      if (quality > GOOD_QUALITY_MAX)
      {
         LogTools.debug("Bad quality solution: {} Try upping the gains, giving more iteration, or setting a more acheivable goal.", quality);
      }
   }

   public void clearCommands()
   {
      for (RobotSide side : RobotSide.values)
      {
         hands.get(side).setActive(false);
         feet.get(side).setActive(false);
      }
      chest.setActive(false);
      pelvis.setActive(false);
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

   public double getQuality()
   {
      return quality;
   }

   public OneDoFJointBasics[] getSolutionOneDoFJoints()
   {
      return workingOneDoFJoints;
   }

//   public Pose3DReadOnly getPelvisPose()
//   {
//      pelvisFramePose.set(workPelvisSixDoFJoint.getJointPose());
//      pelvisFramePose.setToZero(worldFrame);
//      pelvisFramePose.set(workPelvisSixDoFJoint.getJointPose());
//   }

   public SixDoFJoint getSolutionPelvisSixDoFJoint()
   {
      return workPelvisSixDoFJoint;
   }
}
