package us.ihmc.commonWalkingControlModules.inverseKinematics;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsSolution;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointspaceVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.InverseDynamicsQPBoundCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInput;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInputCalculator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class InverseKinematicsOptimizationControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final InverseKinematicsQPSolver qpSolver;
   private final MotionQPInput motionQPInput;
   private final MotionQPInputCalculator motionQPInputCalculator;
   private final InverseDynamicsQPBoundCalculator boundCalculator;

   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final int numberOfDoFs;

   private final DenseMatrix64F qDotMin, qDotMax;

   public InverseKinematicsOptimizationControlModule(WholeBodyControlCoreToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      JointIndexHandler jointIndexHandler = toolbox.getJointIndexHandler();
      jointsToOptimizeFor = jointIndexHandler.getIndexedJoints();

      ReferenceFrame centerOfMassFrame = toolbox.getCenterOfMassFrame();

      numberOfDoFs = ScrewTools.computeDegreesOfFreedom(jointsToOptimizeFor);
      motionQPInput = new MotionQPInput(numberOfDoFs);

      double controlDT = toolbox.getControlDT();
      GeometricJacobianHolder geometricJacobianHolder = toolbox.getGeometricJacobianHolder();
      TwistCalculator twistCalculator = toolbox.getTwistCalculator();
      motionQPInputCalculator = new MotionQPInputCalculator(centerOfMassFrame, geometricJacobianHolder, twistCalculator, jointIndexHandler, registry);
      boundCalculator = new InverseDynamicsQPBoundCalculator(jointIndexHandler, controlDT, registry);

      qDotMin = new DenseMatrix64F(numberOfDoFs, 1);
      qDotMax = new DenseMatrix64F(numberOfDoFs, 1);

      qpSolver = new InverseKinematicsQPSolver(numberOfDoFs, registry);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      qpSolver.reset();
      motionQPInputCalculator.initialize();
   }

   public InverseKinematicsSolution compute() throws InverseKinematicsOptimizationException
   {
      NoConvergenceException noConvergenceException = null;

      computePrivilegedJointVelocities();
      boundCalculator.computeJointVelocityLimits(qDotMin, qDotMax);
      qpSolver.setMaxJointVelocities(qDotMax);
      qpSolver.setMinJointVelocities(qDotMin);

      try
      {
         qpSolver.solve();
      }
      catch (NoConvergenceException e)
      {
         noConvergenceException = e;
      }

      DenseMatrix64F jointVelocities = qpSolver.getJointVelocities();
      InverseKinematicsSolution inverseKinematicsSolution = new InverseKinematicsSolution(jointsToOptimizeFor, jointVelocities);

      if (noConvergenceException != null)
         throw new InverseKinematicsOptimizationException(noConvergenceException, inverseKinematicsSolution);

      return inverseKinematicsSolution;
   }

   private void computePrivilegedJointVelocities()
   {
      boolean success = motionQPInputCalculator.computePrivilegedJointVelocities(motionQPInput);
      if (success)
         qpSolver.addMotionInput(motionQPInput);
   }

   public void submitInverseKinematicsCommand(InverseKinematicsCommand<?> command)
   {
      switch (command.getCommandType())
      {
      case TASKSPACE:
         submitSpatialVelocityCommand((SpatialVelocityCommand) command);
         return;
      case JOINTSPACE:
         submitJointspaceVelocityCommand((JointspaceVelocityCommand) command);
         return;
      case MOMENTUM:
         submitMomentumCommand((MomentumCommand) command);
         return;
      case PRIVILEGED_CONFIGURATION:
         submitPrivilegedConfigurationCommand((PrivilegedConfigurationCommand) command);
         break;
      case LIMIT_REDUCTION:
         submitJointLimitReductionCommand((JointLimitReductionCommand) command);
         break;
      case COMMAND_LIST:
         submitInverseKinematicsCommandList((InverseKinematicsCommandList) command);
         return;
      default:
         throw new RuntimeException("The command type: " + command.getCommandType() + " is not handled.");
      }
   }

   private void submitSpatialVelocityCommand(SpatialVelocityCommand command)
   {
      boolean success = motionQPInputCalculator.convertSpatialVelocityCommand(command, motionQPInput);
      if (success)
         qpSolver.addMotionInput(motionQPInput);
   }

   private void submitJointspaceVelocityCommand(JointspaceVelocityCommand command)
   {
      boolean success = motionQPInputCalculator.convertJointspaceVelocityCommand(command, motionQPInput);
      if (success)
         qpSolver.addMotionInput(motionQPInput);
   }

   private void submitMomentumCommand(MomentumCommand command)
   {
      boolean success = motionQPInputCalculator.convertMomentumCommand(command, motionQPInput);
      if (success)
         qpSolver.addMotionInput(motionQPInput);
   }

   private void submitInverseKinematicsCommandList(InverseKinematicsCommandList command)
   {
      while (command.getNumberOfCommands() > 0)
         submitInverseKinematicsCommand(command.pollCommand());
   }

   private void submitPrivilegedConfigurationCommand(PrivilegedConfigurationCommand command)
   {
      motionQPInputCalculator.updatePrivilegedConfiguration(command);
   }

   private void submitJointLimitReductionCommand(JointLimitReductionCommand command)
   {
      boundCalculator.submitJointLimitReductionCommand(command);
   }
}
