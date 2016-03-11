package us.ihmc.commonWalkingControlModules.inverseKinematics;

import java.util.LinkedHashMap;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.inverseKinematics.dataObjects.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.inverseKinematics.dataObjects.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.inverseKinematics.dataObjects.InverseKinematicsSolution;
import us.ihmc.commonWalkingControlModules.inverseKinematics.dataObjects.JointspaceVelocityCommand;
import us.ihmc.commonWalkingControlModules.inverseKinematics.dataObjects.MomentumCommand;
import us.ihmc.commonWalkingControlModules.inverseKinematics.dataObjects.PrivilegedConfigurationInverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.inverseKinematics.dataObjects.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.CentroidalMomentumMatrix;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
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

   private final DenseMatrix64F tempTaskJacobian = new DenseMatrix64F(SpatialMotionVector.SIZE, 12);
   private final DenseMatrix64F tempTaskObjective = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
   private final DenseMatrix64F tempTaskWeight = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F tempTaskWeightSubspace = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F tempSelectionMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);

   public InverseKinematicsOptimizationControlModule(WholeBodyControlCoreToolbox toolbox, MomentumOptimizationSettings momentumOptimizationSettings,
         YoVariableRegistry parentRegistry)
   {
      jointsToOptimizeFor = momentumOptimizationSettings.getJointsToOptimizeFor();

      InverseDynamicsJoint rootJoint = toolbox.getRobotRootJoint();
      ReferenceFrame centerOfMassFrame = toolbox.getCenterOfMassFrame();

      numberOfDoFs = ScrewTools.computeDegreesOfFreedom(jointsToOptimizeFor);
      geometricJacobianHolder = toolbox.getGeometricJacobianHolder();
      centroidalMomentumMatrixCalculator = new CentroidalMomentumMatrix(rootJoint.getPredecessor(), centerOfMassFrame);
      privilegedConfigurationHandler = new JointPrivilegedConfigurationHandler(jointsToOptimizeFor, registry);
      motionQPInput = new InverseKinematicsMotionQPInput(numberOfDoFs);

      for (InverseDynamicsJoint joint : jointsToOptimizeFor)
      {
         TIntArrayList listToPackIndices = new TIntArrayList();
         ScrewTools.computeIndexForJoint(jointsToOptimizeFor, listToPackIndices, joint);
         int[] indices = listToPackIndices.toArray();

         columnsForJoints.put(joint, indices);
      }

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
      case PRIVILIEGED_CONFIGURATION:
         submitPrivilegedConfigurationInverseKinematicsCommand((PrivilegedConfigurationInverseKinematicsCommand) command);
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
      motionQPInput.setIsMotionConstraint(!command.getHasWeight());
      if (command.getHasWeight())
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
      motionQPInput.setIsMotionConstraint(!command.getHasWeight());
      if (command.getHasWeight())
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
      DenseMatrix64F centroidalMomentumMatrix = centroidalMomentumMatrixCalculator.getMatrix();
      CommonOps.mult(selectionMatrix, centroidalMomentumMatrix, motionQPInput.taskJacobian);

      DenseMatrix64F momemtum = command.getMomemtum();

      // Compute the task objective: p = S * h
      CommonOps.mult(selectionMatrix, momemtum, motionQPInput.taskObjective);

      qpSolver.addMotionInput(motionQPInput);
   }

   private void submitInverseKinematicsCommandList(InverseKinematicsCommandList command)
   {
      for (int i = 0; i < command.getNumberOfCommands(); i++)
         submitInverseKinematicsCommand(command.getCommand(i));
   }

   private void submitPrivilegedConfigurationInverseKinematicsCommand(PrivilegedConfigurationInverseKinematicsCommand command)
   {
      privilegedConfigurationHandler.submitPrivilegedConfigurationInverseKinematicsCommand(command);

      if (!privilegedConfigurationHandler.isEnabled())
         return;

      privilegedConfigurationHandler.compute();
      OneDoFJoint[] joints = privilegedConfigurationHandler.getJoints();
      DenseMatrix64F privilegedJointVelocities = privilegedConfigurationHandler.getPrivilegedJointVelocities();
      DenseMatrix64F weight = privilegedConfigurationHandler.getWeight();
      DenseMatrix64F selectionMatrix = privilegedConfigurationHandler.getSelectionMatrix();

      int taskSize = privilegedJointVelocities.getNumRows();

      tempSelectionMatrix.reshape(taskSize, numberOfDoFs);
      compactBlockToFullBlock(joints, selectionMatrix, tempSelectionMatrix);

      qpSolver.projectPrivilegedJointVelocitiesInNullspaceOfPreviousTasks(selectionMatrix, privilegedJointVelocities, weight);
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
