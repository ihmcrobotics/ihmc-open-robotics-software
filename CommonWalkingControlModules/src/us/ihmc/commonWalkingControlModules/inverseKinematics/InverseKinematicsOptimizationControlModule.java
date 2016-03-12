package us.ihmc.commonWalkingControlModules.inverseKinematics;

import java.util.LinkedHashMap;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsSolution;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointspaceVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.CentroidalMomentumMatrix;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.Momentum;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class InverseKinematicsOptimizationControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final GeometricJacobianHolder geometricJacobianHolder;
   private final CentroidalMomentumMatrix centroidalMomentumMatrixCalculator;
   private final JointPrivilegedConfigurationHandler privilegedConfigurationHandler;
   private final InverseKinematicsQPSolver qpSolver;
   private final InverseKinematicsMotionQPInput motionQPInput;

   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final int numberOfDoFs;

   private final TIntArrayList indicesIntoCompactBlock = new TIntArrayList();
   private final LinkedHashMap<InverseDynamicsJoint, int[]> columnsForJoints = new LinkedHashMap<InverseDynamicsJoint, int[]>();

   private final DenseMatrix64F qDotMin, qDotMax;
   private final double controlDT;
   private final OneDoFJoint[] oneDoFJoints;

   private final DenseMatrix64F tempTaskJacobian = new DenseMatrix64F(SpatialMotionVector.SIZE, 12);
   private final DenseMatrix64F tempTaskObjective = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
   private final DenseMatrix64F tempTaskWeight = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F tempTaskWeightSubspace = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F tempSelectionMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F partialCentroidalMomtentumMatrix;

   public InverseKinematicsOptimizationControlModule(WholeBodyControlCoreToolbox toolbox, MomentumOptimizationSettings momentumOptimizationSettings,
         YoVariableRegistry parentRegistry)
   {
      controlDT = toolbox.getControlDT();
      jointsToOptimizeFor = momentumOptimizationSettings.getJointsToOptimizeFor();
      oneDoFJoints = ScrewTools.filterJoints(jointsToOptimizeFor, OneDoFJoint.class);

      InverseDynamicsJoint rootJoint = toolbox.getRobotRootJoint();
      ReferenceFrame centerOfMassFrame = toolbox.getCenterOfMassFrame();

      numberOfDoFs = ScrewTools.computeDegreesOfFreedom(jointsToOptimizeFor);
      geometricJacobianHolder = toolbox.getGeometricJacobianHolder();
      centroidalMomentumMatrixCalculator = new CentroidalMomentumMatrix(rootJoint.getPredecessor(), centerOfMassFrame);
      privilegedConfigurationHandler = new JointPrivilegedConfigurationHandler(jointsToOptimizeFor, registry);
      partialCentroidalMomtentumMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, numberOfDoFs);
      motionQPInput = new InverseKinematicsMotionQPInput(numberOfDoFs);

      for (InverseDynamicsJoint joint : jointsToOptimizeFor)
      {
         TIntArrayList listToPackIndices = new TIntArrayList();
         ScrewTools.computeIndexForJoint(jointsToOptimizeFor, listToPackIndices, joint);
         int[] indices = listToPackIndices.toArray();

         columnsForJoints.put(joint, indices);
      }

      qDotMin = new DenseMatrix64F(numberOfDoFs, 1);
      qDotMax = new DenseMatrix64F(numberOfDoFs, 1);

      qpSolver = new InverseKinematicsQPSolver(numberOfDoFs, registry);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      qpSolver.reset();
      centroidalMomentumMatrixCalculator.compute();
   }

   public InverseKinematicsSolution compute() throws InverseKinematicsOptimizationException
   {
      NoConvergenceException noConvergenceException = null;

      computePrivilegedJointVelocities();
      computeJointVelocityLimits();
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

   private void computeJointVelocityLimits()
   {
      CommonOps.fill(qDotMin, Double.NEGATIVE_INFINITY);
      CommonOps.fill(qDotMax, Double.POSITIVE_INFINITY);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         int index = columnsForJoints.get(joint)[0];
         double jointLimitLower = joint.getJointLimitLower();
         if (Double.isFinite(jointLimitLower))
            qDotMin.set(index, 0, (jointLimitLower - joint.getQ()) / controlDT);
         double jointLimitUpper = joint.getJointLimitUpper();
         if (Double.isFinite(jointLimitUpper))
            qDotMax.set(index, 0, (jointLimitUpper - joint.getQ()) / controlDT);
      }
   }

   private void computePrivilegedJointVelocities()
   {
      if (privilegedConfigurationHandler.isEnabled())
      {
         privilegedConfigurationHandler.computePrivilegedJointVelocities();
         OneDoFJoint[] joints = privilegedConfigurationHandler.getJoints();
         DenseMatrix64F privilegedJointVelocities = privilegedConfigurationHandler.getPrivilegedJointVelocities();
         DenseMatrix64F weight = privilegedConfigurationHandler.getWeight();
         DenseMatrix64F selectionMatrix = privilegedConfigurationHandler.getSelectionMatrix();

         int taskSize = privilegedJointVelocities.getNumRows();

         tempSelectionMatrix.reshape(taskSize, numberOfDoFs);
         compactBlockToFullBlock(joints, selectionMatrix, tempSelectionMatrix);

         qpSolver.projectPrivilegedJointVelocitiesInNullspaceOfPreviousTasks(tempSelectionMatrix, privilegedJointVelocities, weight);
      }
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
         submitPrivilegedConfigurationInverseKinematicsCommand((PrivilegedConfigurationCommand) command);
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
      DenseMatrix64F selectionMatrix = command.getSelectionMatrix();
      int taskSize = selectionMatrix.getNumRows();

      if (taskSize == 0)
         return;

      motionQPInput.reshape(taskSize);
      motionQPInput.setIsMotionConstraint(command.isHardConstraint());
      if (!command.isHardConstraint())
      {
         motionQPInput.setUseWeightScalar(true);
         motionQPInput.setWeight(command.getWeight());
      }

      Twist spatialVelocity = command.getSpatialVelocity();
      RigidBody base = command.getBase();
      RigidBody endEffector = command.getEndEffector();
      long jacobianId = geometricJacobianHolder.getOrCreateGeometricJacobian(base, endEffector, spatialVelocity.getExpressedInFrame());
      GeometricJacobian jacobian = geometricJacobianHolder.getJacobian(jacobianId);

      // Compute the task Jacobian: J = S * J
      tempTaskJacobian.reshape(taskSize, jacobian.getNumberOfColumns());
      CommonOps.mult(selectionMatrix, jacobian.getJacobianMatrix(), tempTaskJacobian);
      compactBlockToFullBlock(jacobian.getJointsInOrder(), tempTaskJacobian, motionQPInput.taskJacobian);

      // Compute the task objective: p = S * T
      spatialVelocity.getMatrix(tempTaskObjective, 0);
      CommonOps.mult(selectionMatrix, tempTaskObjective, motionQPInput.taskObjective);

      qpSolver.addMotionInput(motionQPInput);
   }

   private void submitJointspaceVelocityCommand(JointspaceVelocityCommand command)
   {
      int taskSize = ScrewTools.computeDegreesOfFreedom(command.getJoints());

      if (taskSize == 0)
         return;

      motionQPInput.reshape(taskSize);
      motionQPInput.setIsMotionConstraint(command.isHardConstraint());
      if (!command.isHardConstraint())
      {
         motionQPInput.setUseWeightScalar(true);
         motionQPInput.setWeight(command.getWeight());
      }

      motionQPInput.taskJacobian.zero();

      int row = 0;
      for (int jointIndex = 0; jointIndex < command.getNumberOfJoints(); jointIndex++)
      {
         InverseDynamicsJoint joint = command.getJoint(jointIndex);
         int[] columns = columnsForJoints.get(joint);
         if (columns == null)
            return;
         for (int column : columns)
            motionQPInput.taskJacobian.set(row, column, 1.0);

         CommonOps.insert(command.getDesiredVelocity(jointIndex), motionQPInput.taskObjective, row, 0);
         row += joint.getDegreesOfFreedom();
      }

      qpSolver.addMotionInput(motionQPInput);
   }

   private void submitMomentumCommand(MomentumCommand command)
   {
      DenseMatrix64F selectionMatrix = command.getSelectionMatrix();
      int taskSize = selectionMatrix.getNumRows();

      if (taskSize == 0)
         return;

      motionQPInput.reshape(taskSize);
      motionQPInput.setUseWeightScalar(false);
      motionQPInput.setIsMotionConstraint(false);

      // Compute the weight: W = S * W * S^T
      tempTaskWeight.reshape(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
      command.getWeightMatrix(tempTaskWeight);
      tempTaskWeightSubspace.reshape(taskSize, SpatialAccelerationVector.SIZE);
      CommonOps.mult(selectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
      CommonOps.multTransB(tempTaskWeightSubspace, selectionMatrix, motionQPInput.taskWeightMatrix);

      // Compute the task Jacobian: J = S * A
      DenseMatrix64F centroidalMomentumMatrix = getPartialCentroidalMomentumMatrix();
      CommonOps.mult(selectionMatrix, centroidalMomentumMatrix, motionQPInput.taskJacobian);

      DenseMatrix64F momemtum = command.getMomentum();

      // Compute the task objective: p = S * h
      CommonOps.mult(selectionMatrix, momemtum, motionQPInput.taskObjective);

      qpSolver.addMotionInput(motionQPInput);
   }

   private void submitInverseKinematicsCommandList(InverseKinematicsCommandList command)
   {
      for (int i = 0; i < command.getNumberOfCommands(); i++)
         submitInverseKinematicsCommand(command.getCommand(i));
   }

   private void submitPrivilegedConfigurationInverseKinematicsCommand(PrivilegedConfigurationCommand command)
   {
      privilegedConfigurationHandler.submitPrivilegedConfigurationInverseKinematicsCommand(command);
   }

   private DenseMatrix64F getPartialCentroidalMomentumMatrix()
   {
      partialCentroidalMomtentumMatrix.reshape(Momentum.SIZE, numberOfDoFs);
      int startColumn = 0;
      for (InverseDynamicsJoint joint : jointsToOptimizeFor)
      {
         int[] columnsForJoint = columnsForJoints.get(joint);
         MatrixTools.extractColumns(centroidalMomentumMatrixCalculator.getMatrix(), columnsForJoint, partialCentroidalMomtentumMatrix, startColumn);
         startColumn += columnsForJoint.length;
      }

      return partialCentroidalMomtentumMatrix;
   }

   private void compactBlockToFullBlock(InverseDynamicsJoint[] joints, DenseMatrix64F compactMatrix, DenseMatrix64F fullMatrix)
   {
      fullMatrix.zero();

      for (int index = 0; index < joints.length; index++)
      {
         InverseDynamicsJoint joint = joints[index];
         indicesIntoCompactBlock.reset();
         ScrewTools.computeIndexForJoint(joints, indicesIntoCompactBlock, joint);
         int[] indicesIntoFullBlock = columnsForJoints.get(joint);

         if (indicesIntoFullBlock == null) // don't do anything for joints that are not in the list
            return;

         for (int i = 0; i < indicesIntoCompactBlock.size(); i++)
         {
            int compactBlockIndex = indicesIntoCompactBlock.get(i);
            int fullBlockIndex = indicesIntoFullBlock[i];
            CommonOps.extract(compactMatrix, 0, compactMatrix.getNumRows(), compactBlockIndex, compactBlockIndex + 1, fullMatrix, 0, fullBlockIndex);
         }
      }
   }

   public int getOneDoFJointIndex(OneDoFJoint joint)
   {
      int[] jointIndices = columnsForJoints.get(joint);
      if (jointIndices == null)
         return -1;
      else
         return jointIndices[0];
   }

   public int[] getJointIndices(InverseDynamicsJoint joint)
   {
      return columnsForJoints.get(joint);
   }
}
