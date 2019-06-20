package us.ihmc.commonWalkingControlModules.inverseKinematics;

import java.util.HashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsSolution;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointspaceVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.WholeBodyControllerBoundCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInputCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInput;
import us.ihmc.convexOptimization.quadraticProgram.ActiveSetQPSolver;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class InverseKinematicsOptimizationControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final InverseKinematicsQPSolver qpSolver;
   private final QPInput motionQPInput;
   private final MotionQPInputCalculator motionQPInputCalculator;
   private final WholeBodyControllerBoundCalculator boundCalculator;

   private final OneDoFJointBasics[] oneDoFJoints;
   private final JointBasics[] jointsToOptimizeFor;
   private final int numberOfDoFs;

   private final Map<OneDoFJointBasics, YoDouble> jointMaximumVelocities = new HashMap<>();
   private final Map<OneDoFJointBasics, YoDouble> jointMinimumVelocities = new HashMap<>();
   private final DenseMatrix64F qDotMinMatrix, qDotMaxMatrix;
   private final JointIndexHandler jointIndexHandler;

   private final YoBoolean hasNotConvergedInPast = new YoBoolean("hasNotConvergedInPast", registry);
   private final YoInteger hasNotConvergedCounts = new YoInteger("hasNotConvergedCounts", registry);

   public InverseKinematicsOptimizationControlModule(WholeBodyControlCoreToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      jointIndexHandler = toolbox.getJointIndexHandler();
      jointsToOptimizeFor = jointIndexHandler.getIndexedJoints();
      oneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();

      numberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(jointsToOptimizeFor);
      motionQPInput = new QPInput(numberOfDoFs);

      motionQPInputCalculator = toolbox.getMotionQPInputCalculator();
      boundCalculator = toolbox.getQPBoundCalculator();

      qDotMinMatrix = new DenseMatrix64F(numberOfDoFs, 1);
      qDotMaxMatrix = new DenseMatrix64F(numberOfDoFs, 1);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJointBasics joint = oneDoFJoints[i];
         jointMaximumVelocities.put(joint, new YoDouble("qd_max_qp_" + joint.getName(), registry));
         jointMinimumVelocities.put(joint, new YoDouble("qd_min_qp_" + joint.getName(), registry));
      }

      ControllerCoreOptimizationSettings optimizationSettings = toolbox.getOptimizationSettings();
      ActiveSetQPSolver activeSetQPSolver;
      if (optimizationSettings == null)
         activeSetQPSolver = new SimpleEfficientActiveSetQPSolver();
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

      computePrivilegedJointVelocities();
      computeJointVelocityLimits();
      qpSolver.setMaxJointVelocities(qDotMaxMatrix);
      qpSolver.setMinJointVelocities(qDotMinMatrix);

      try
      {
         qpSolver.solve();
      }
      catch (NoConvergenceException e)
      {
         if (!hasNotConvergedInPast.getBooleanValue())
         {
            e.printStackTrace();
            LogTools.warn("Only showing the stack trace of the first " + e.getClass().getSimpleName() + ". This may be happening more than once.");
         }

         hasNotConvergedInPast.set(true);
         hasNotConvergedCounts.increment();

         noConvergenceException = e;
      }

      DenseMatrix64F jointVelocities = qpSolver.getJointVelocities();
      MomentumReadOnly centroidalMomentumSoltuion = motionQPInputCalculator.computeCentroidalMomentumFromSolution(jointVelocities);
      InverseKinematicsSolution inverseKinematicsSolution = new InverseKinematicsSolution(jointsToOptimizeFor, jointVelocities);
      inverseKinematicsSolution.setCentroidalMomentumSolution(centroidalMomentumSoltuion);

      if (noConvergenceException != null)
         throw new InverseKinematicsOptimizationException(noConvergenceException, inverseKinematicsSolution);

      return inverseKinematicsSolution;
   }

   private void computeJointVelocityLimits()
   {
      boundCalculator.computeJointVelocityLimits(qDotMinMatrix, qDotMaxMatrix);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJointBasics joint = oneDoFJoints[i];

         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);
         double qDDotMin = qDotMinMatrix.get(jointIndex, 0);
         double qDDotMax = qDotMaxMatrix.get(jointIndex, 0);
         jointMinimumVelocities.get(joint).set(qDDotMin);
         jointMaximumVelocities.get(joint).set(qDDotMax);
      }
   }

   private void computePrivilegedJointVelocities()
   {
      boolean success = motionQPInputCalculator.computePrivilegedJointVelocities(motionQPInput);
      if (success)
         qpSolver.addMotionInput(motionQPInput);
   }

   public void submitSpatialVelocityCommand(SpatialVelocityCommand command)
   {
      boolean success = motionQPInputCalculator.convertSpatialVelocityCommand(command, motionQPInput);
      if (success)
         qpSolver.addMotionInput(motionQPInput);
   }

   public void submitJointspaceVelocityCommand(JointspaceVelocityCommand command)
   {
      boolean success = motionQPInputCalculator.convertJointspaceVelocityCommand(command, motionQPInput);
      if (success)
         qpSolver.addMotionInput(motionQPInput);
   }

   public void submitMomentumCommand(MomentumCommand command)
   {
      boolean success = motionQPInputCalculator.convertMomentumCommand(command, motionQPInput);
      if (success)
         qpSolver.addMotionInput(motionQPInput);
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
   }
}
