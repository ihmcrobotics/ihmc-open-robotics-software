package us.ihmc.commonWalkingControlModules.inverseKinematics;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsOptimizationSettingsCommand.ActivationState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsSolution;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointspaceVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.LinearMomentumConvexConstraint2DCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.WholeBodyControllerBoundCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInputCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPVariableSubstitution;
import us.ihmc.convexOptimization.exceptions.NoConvergenceException;
import us.ihmc.convexOptimization.quadraticProgram.ActiveSetQPSolverWithInactiveVariablesInterface;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolverWithInactiveVariables;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.KinematicLoopFunction;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class InverseKinematicsOptimizationControlModule
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final InverseKinematicsQPSolver qpSolver;
   private final QPInputTypeA qpInput;
   private final QPVariableSubstitution qpVariableSubstitution;
   private final MotionQPInputCalculator motionQPInputCalculator;
   private final WholeBodyControllerBoundCalculator boundCalculator;

   private final OneDoFJointBasics[] oneDoFJoints;
   private final JointBasics[] jointsToOptimizeFor;
   private final List<KinematicLoopFunction> kinematicLoopFunctions;
   private final int numberOfDoFs;

   private final Map<OneDoFJointBasics, YoDouble> jointMaximumVelocities = new HashMap<>();
   private final Map<OneDoFJointBasics, YoDouble> jointMinimumVelocities = new HashMap<>();
   private final DMatrixRMaj qDotMinMatrix, qDotMaxMatrix;
   private final JointIndexHandler jointIndexHandler;

   private final YoBoolean computeJointTorques = new YoBoolean("computeJointTorques", registry);
   private final YoBoolean hasNotConvergedInPast = new YoBoolean("hasNotConvergedInPast", registry);
   private final YoInteger hasNotConvergedCounts = new YoInteger("hasNotConvergedCounts", registry);

   private final InverseKinematicsSolution inverseKinematicsSolution;
   private final WholeBodyControlCoreToolbox toolbox;

   public InverseKinematicsOptimizationControlModule(WholeBodyControlCoreToolbox toolbox, YoRegistry parentRegistry)
   {
      this.toolbox = toolbox;
      jointIndexHandler = toolbox.getJointIndexHandler();
      jointsToOptimizeFor = jointIndexHandler.getIndexedJoints();
      oneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      kinematicLoopFunctions = toolbox.getKinematicLoopFunctions();
      inverseKinematicsSolution = new InverseKinematicsSolution(jointsToOptimizeFor);

      numberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(jointsToOptimizeFor);
      qpInput = new QPInputTypeA(numberOfDoFs);
      qpVariableSubstitution = new QPVariableSubstitution();

      motionQPInputCalculator = toolbox.getMotionQPInputCalculator();
      boundCalculator = toolbox.getQPBoundCalculator();

      qDotMinMatrix = new DMatrixRMaj(numberOfDoFs, 1);
      qDotMaxMatrix = new DMatrixRMaj(numberOfDoFs, 1);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJointBasics joint = oneDoFJoints[i];
         jointMaximumVelocities.put(joint, new YoDouble("qd_max_qp_" + joint.getName(), registry));
         jointMinimumVelocities.put(joint, new YoDouble("qd_min_qp_" + joint.getName(), registry));
      }

      ControllerCoreOptimizationSettings optimizationSettings = toolbox.getOptimizationSettings();
      if (optimizationSettings != null)
      {
         computeJointTorques.set(optimizationSettings.areJointTorquesMinimized());
      }

      ActiveSetQPSolverWithInactiveVariablesInterface activeSetQPSolver;
      if (optimizationSettings == null)
         activeSetQPSolver = new SimpleEfficientActiveSetQPSolverWithInactiveVariables();
      else
         activeSetQPSolver = optimizationSettings.getActiveSetQPSolver();

      double controlDT = toolbox.getControlDT();
      qpSolver = new InverseKinematicsQPSolver(activeSetQPSolver, numberOfDoFs, controlDT, registry);

      if (optimizationSettings != null)
      {
         qpSolver.setVelocityRegularizationWeight(optimizationSettings.getJointVelocityWeight());
         qpSolver.setAccelerationRegularizationWeight(optimizationSettings.getJointAccelerationWeight());
      }

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

      computeJointTorqueMinimization();
      computePrivilegedJointVelocities();
      computeJointVelocityLimits();
      qpSolver.setMaxJointVelocities(qDotMaxMatrix);
      qpSolver.setMinJointVelocities(qDotMinMatrix);

      for (int i = 0; i < kinematicLoopFunctions.size(); i++)
      {
         motionQPInputCalculator.convertKinematicLoopFunction(kinematicLoopFunctions.get(i), qpVariableSubstitution);
         qpSolver.addVariableSubstitution(qpVariableSubstitution);
      }

      try
      {
         qpSolver.solve();
      }
      catch (NoConvergenceException e)
      {
         if (!hasNotConvergedInPast.getBooleanValue())
         {
            LogTools.warn("First failure for the optimization " + e.getClass().getSimpleName() + ". Only reporting the first failure, see failure counter: "
                  + hasNotConvergedCounts.getFullNameString());
         }

         hasNotConvergedInPast.set(true);
         hasNotConvergedCounts.increment();

         noConvergenceException = e;
      }

      DMatrixRMaj jointVelocities = qpSolver.getJointVelocities();
      MomentumReadOnly centroidalMomentumSoltuion = motionQPInputCalculator.computeCentroidalMomentumFromSolution(jointVelocities);
      inverseKinematicsSolution.setJointVelocities(jointVelocities);
      if (computeJointTorques.getValue())
         inverseKinematicsSolution.setJointTorques(toolbox.getGravityGradientCalculator().getTauMatrix());
      inverseKinematicsSolution.setCentroidalMomentumSolution(centroidalMomentumSoltuion);

      if (noConvergenceException != null)
         throw new InverseKinematicsOptimizationException(noConvergenceException, inverseKinematicsSolution);

      return inverseKinematicsSolution;
   }

   private void computeJointTorqueMinimization()
   {
      if (!computeJointTorques.getValue())
         return;

      boolean success = motionQPInputCalculator.computeGravityCompensationMinimization(qpInput,
                                                                                       toolbox.getJointTorqueMinimizationWeightCalculator(),
                                                                                       true,
                                                                                       toolbox.getControlDT());
      if (success)
         qpSolver.addMotionInput(qpInput);
   }

   private void computeJointVelocityLimits()
   {
      boundCalculator.computeJointVelocityLimits(qDotMinMatrix, qDotMaxMatrix);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJointBasics joint = oneDoFJoints[i];

         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);
         double qDotMin = qDotMinMatrix.get(jointIndex, 0);
         double qDotMax = qDotMaxMatrix.get(jointIndex, 0);
         jointMinimumVelocities.get(joint).set(qDotMin);
         jointMaximumVelocities.get(joint).set(qDotMax);
      }
   }

   private void computePrivilegedJointVelocities()
   {
      boolean success = motionQPInputCalculator.computePrivilegedJointVelocities(qpInput);
      if (success)
         qpSolver.addMotionInput(qpInput);
   }

   public void submitSpatialVelocityCommand(SpatialVelocityCommand command)
   {
      boolean success = motionQPInputCalculator.convertSpatialVelocityCommand(command, qpInput);
      if (success)
         qpSolver.addMotionInput(qpInput);
   }

   public void submitJointspaceVelocityCommand(JointspaceVelocityCommand command)
   {
      boolean success = motionQPInputCalculator.convertJointspaceVelocityCommand(command, qpInput);
      if (success)
         qpSolver.addMotionInput(qpInput);
   }

   public void submitMomentumCommand(MomentumCommand command)
   {
      boolean success = motionQPInputCalculator.convertMomentumCommand(command, qpInput);
      if (success)
         qpSolver.addMotionInput(qpInput);
   }

   public void submitLinearMomentumConvexConstraint2DCommand(LinearMomentumConvexConstraint2DCommand command)
   {
      boolean success = motionQPInputCalculator.convertLinearMomentumConvexConstraint2DCommand(command, qpInput);
      if (success)
         qpSolver.addMotionInput(qpInput);
   }

   public void submitPrivilegedConfigurationCommand(PrivilegedConfigurationCommand command)
   {
      motionQPInputCalculator.updatePrivilegedConfiguration(command);
   }

   public void submitPrivilegedVelocityCommand(PrivilegedJointSpaceCommand command)
   {
      motionQPInputCalculator.submitPrivilegedVelocities(command);
   }

   public void submitJointLimitReductionCommand(JointLimitReductionCommand command)
   {
      boundCalculator.submitJointLimitReductionCommand(command);
   }

   public void submitJointLimitEnforcementMethodCommand(JointLimitEnforcementMethodCommand command)
   {
      boundCalculator.submitJointLimitEnforcementMethodCommand(command);
   }

   public void submitOptimizationSettingsCommand(InverseKinematicsOptimizationSettingsCommand command)
   {
      if (command.hasJointVelocityWeight())
         qpSolver.setVelocityRegularizationWeight(command.getJointVelocityWeight());
      if (command.hasJointAccelerationWeight())
         qpSolver.setAccelerationRegularizationWeight(command.getJointAccelerationWeight());
      if (command.hashJointVelocityLimitMode())
         boundCalculator.considerJointVelocityLimits(command.getJointVelocityLimitMode() == ActivationState.ENABLED);
      if (command.hasComputeJointTorques())
         computeJointTorques.set(command.getComputeJointTorques() == ActivationState.ENABLED);
   }
}
