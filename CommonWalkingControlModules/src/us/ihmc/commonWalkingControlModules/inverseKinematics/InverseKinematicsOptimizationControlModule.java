package us.ihmc.commonWalkingControlModules.inverseKinematics;

import java.util.HashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsSolution;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointspaceVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.InverseDynamicsQPBoundCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInput;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInputCalculator;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class InverseKinematicsOptimizationControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final InverseKinematicsQPSolver qpSolver;
   private final MotionQPInput motionQPInput;
   private final MotionQPInputCalculator motionQPInputCalculator;
   private final InverseDynamicsQPBoundCalculator boundCalculator;

   private final OneDoFJoint[] oneDoFJoints;
   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final int numberOfDoFs;

   private final Map<OneDoFJoint, DoubleYoVariable> jointMaximumVelocities = new HashMap<>();
   private final Map<OneDoFJoint, DoubleYoVariable> jointMinimumVelocities = new HashMap<>();
   private final DenseMatrix64F qDotMinMatrix, qDotMaxMatrix;
   private final JointIndexHandler jointIndexHandler;

   private final BooleanYoVariable hasNotConvergedInPast = new BooleanYoVariable("hasNotConvergedInPast", registry);
   private final IntegerYoVariable hasNotConvergedCounts = new IntegerYoVariable("hasNotConvergedCounts", registry);

   public InverseKinematicsOptimizationControlModule(WholeBodyControlCoreToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      jointIndexHandler = toolbox.getJointIndexHandler();
      jointsToOptimizeFor = jointIndexHandler.getIndexedJoints();
      oneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();

      numberOfDoFs = ScrewTools.computeDegreesOfFreedom(jointsToOptimizeFor);
      motionQPInput = new MotionQPInput(numberOfDoFs);

      motionQPInputCalculator = toolbox.getMotionQPInputCalculator();
      boundCalculator = toolbox.getQPBoundCalculator();

      qDotMinMatrix = new DenseMatrix64F(numberOfDoFs, 1);
      qDotMaxMatrix = new DenseMatrix64F(numberOfDoFs, 1);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         jointMaximumVelocities.put(joint, new DoubleYoVariable("qd_max_qp_" + joint.getName(), registry));
         jointMinimumVelocities.put(joint, new DoubleYoVariable("qd_min_qp_" + joint.getName(), registry));
      }

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
            PrintTools.warn(this, "Only showing the stack trace of the first " + e.getClass().getSimpleName() + ". This may be happening more than once.");
         }

         hasNotConvergedInPast.set(true);
         hasNotConvergedCounts.increment();

         noConvergenceException = e;
      }

      DenseMatrix64F jointVelocities = qpSolver.getJointVelocities();
      InverseKinematicsSolution inverseKinematicsSolution = new InverseKinematicsSolution(jointsToOptimizeFor, jointVelocities);

      if (noConvergenceException != null)
         throw new InverseKinematicsOptimizationException(noConvergenceException, inverseKinematicsSolution);

      return inverseKinematicsSolution;
   }

   private void computeJointVelocityLimits()
   {
      boundCalculator.computeJointVelocityLimits(qDotMinMatrix, qDotMaxMatrix);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];

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

   public void submitJointLimitReductionCommand(JointLimitReductionCommand command)
   {
      boundCalculator.submitJointLimitReductionCommand(command);
   }
}
