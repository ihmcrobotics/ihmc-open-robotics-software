package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.*;
import us.ihmc.commonWalkingControlModules.inverseKinematics.JointPrivilegedConfigurationHandler;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.robotics.linearAlgebra.DampedLeastSquaresNullspaceCalculator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.GeometricJacobianCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;

public class MotionQPInputCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble nullspaceProjectionAlpha;
   private final YoDouble secondaryTaskJointsWeight = new YoDouble("secondaryTaskJointsWeight", registry);

   private final PoseReferenceFrame controlFrame = new PoseReferenceFrame("controlFrame", worldFrame);
   private final GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();

   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final OneDoFJoint[] oneDoFJoints;

   private final DenseMatrix64F convectiveTermMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);

   private final CentroidalMomentumHandler centroidalMomentumHandler;

   private final JointPrivilegedConfigurationHandler privilegedConfigurationHandler;

   private final DenseMatrix64F tempPrimaryTaskJacobian = new DenseMatrix64F(SpatialMotionVector.SIZE, 12);

   private final DenseMatrix64F tempTaskJacobian = new DenseMatrix64F(SpatialMotionVector.SIZE, 12);
   private final DenseMatrix64F tempTaskObjective = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
   private final DenseMatrix64F tempTaskWeight = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F tempTaskWeightSubspace = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);

   private final DenseMatrix64F tempSelectionMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);

   private final FrameVector3D angularMomentum = new FrameVector3D();
   private final FrameVector3D linearMomentum = new FrameVector3D();
   private final ReferenceFrame centerOfMassFrame;

   private final JointIndexHandler jointIndexHandler;

   private final DenseMatrix64F allTaskJacobian;
   private final DampedLeastSquaresNullspaceCalculator nullspaceCalculator;

   private final int numberOfDoFs;

   public MotionQPInputCalculator(ReferenceFrame centerOfMassFrame, CentroidalMomentumHandler centroidalMomentumHandler, JointIndexHandler jointIndexHandler,
                                  JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters, YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      this.jointIndexHandler = jointIndexHandler;
      this.jointsToOptimizeFor = jointIndexHandler.getIndexedJoints();
      this.centroidalMomentumHandler = centroidalMomentumHandler;
      oneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      numberOfDoFs = jointIndexHandler.getNumberOfDoFs();

      if (jointPrivilegedConfigurationParameters != null)
      {
         privilegedConfigurationHandler = new JointPrivilegedConfigurationHandler(oneDoFJoints, jointPrivilegedConfigurationParameters, registry);
         nullspaceProjectionAlpha = new YoDouble("nullspaceProjectionAlpha", registry);
         nullspaceProjectionAlpha.set(jointPrivilegedConfigurationParameters.getNullspaceProjectionAlpha());
         nullspaceCalculator = new DampedLeastSquaresNullspaceCalculator(numberOfDoFs, nullspaceProjectionAlpha.getDoubleValue());
      }
      else
      {
         privilegedConfigurationHandler = null;
         nullspaceProjectionAlpha = null;
         nullspaceCalculator = null;
      }

      allTaskJacobian = new DenseMatrix64F(numberOfDoFs, numberOfDoFs);
      secondaryTaskJointsWeight.set(1.0); // TODO Needs to be rethought, it doesn't seem to be that useful.

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      centroidalMomentumHandler.compute();
      allTaskJacobian.reshape(0, numberOfDoFs);
   }

   public void updatePrivilegedConfiguration(PrivilegedConfigurationCommand command)
   {
      if (privilegedConfigurationHandler == null)
         throw new NullPointerException("JointPrivilegedConfigurationParameters have to be set to enable this feature.");
      privilegedConfigurationHandler.submitPrivilegedConfigurationCommand(command);
   }


   public void submitPrivilegedAccelerations(PrivilegedAccelerationCommand command)
   {
      if (privilegedConfigurationHandler == null)
         throw new NullPointerException("JointPrivilegedConfigurationParameters have to be set to enable this feature.");
      privilegedConfigurationHandler.submitPrivilegedAccelerations(command);
   }

   public void submitPrivilegedVelocities(PrivilegedVelocityCommand command)
   {
      if (privilegedConfigurationHandler == null)
         throw new NullPointerException("JointPrivilegedConfigurationParameters have to be set to enable this feature.");
      privilegedConfigurationHandler.submitPrivilegedVelocities(command);
   }


   public boolean computePrivilegedJointAccelerations(MotionQPInput motionQPInputToPack)
   {
      if (privilegedConfigurationHandler == null || !privilegedConfigurationHandler.isEnabled())
         return false;

      privilegedConfigurationHandler.computePrivilegedJointAccelerations();

      motionQPInputToPack.setConstraintType(ConstraintType.OBJECTIVE);
      motionQPInputToPack.setUseWeightScalar(false);

      nullspaceCalculator.setPseudoInverseAlpha(nullspaceProjectionAlpha.getDoubleValue());

      int taskSize = 0;

      DenseMatrix64F selectionMatrix = privilegedConfigurationHandler.getSelectionMatrix();
      int robotTaskSize = selectionMatrix.getNumRows();

      if (robotTaskSize > 0)
      {
         OneDoFJoint[] joints = privilegedConfigurationHandler.getJoints();
         tempTaskJacobian.reshape(robotTaskSize, numberOfDoFs);
         boolean success = jointIndexHandler.compactBlockToFullBlock(joints, selectionMatrix, tempTaskJacobian);

         if (success)
         {
            motionQPInputToPack.reshape(robotTaskSize);
            nullspaceCalculator.projectOntoNullspace(tempTaskJacobian, allTaskJacobian);
            CommonOps.insert(tempTaskJacobian, motionQPInputToPack.taskJacobian, taskSize, 0);
            CommonOps.insert(privilegedConfigurationHandler.getPrivilegedJointAccelerations(), motionQPInputToPack.taskObjective, taskSize, 0);
            CommonOps.insert(privilegedConfigurationHandler.getWeights(), motionQPInputToPack.taskWeightMatrix, taskSize, taskSize);
         }
      }

      return robotTaskSize > 0;
   }

   public boolean computePrivilegedJointVelocities(MotionQPInput motionQPInputToPack)
   {
      if (privilegedConfigurationHandler == null || !privilegedConfigurationHandler.isEnabled())
         return false;

      privilegedConfigurationHandler.computePrivilegedJointVelocities();

      motionQPInputToPack.setConstraintType(ConstraintType.OBJECTIVE);
      motionQPInputToPack.setUseWeightScalar(false);

      DenseMatrix64F selectionMatrix = privilegedConfigurationHandler.getSelectionMatrix();

      int taskSize = selectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      motionQPInputToPack.reshape(taskSize);

      motionQPInputToPack.setTaskObjective(privilegedConfigurationHandler.getPrivilegedJointVelocities());
      motionQPInputToPack.setTaskWeightMatrix(privilegedConfigurationHandler.getWeights());

      OneDoFJoint[] joints = privilegedConfigurationHandler.getJoints();
      boolean success = jointIndexHandler.compactBlockToFullBlock(joints, selectionMatrix, motionQPInputToPack.taskJacobian);

      if (!success)
         return false;

      nullspaceCalculator.projectOntoNullspace(motionQPInputToPack.taskJacobian, allTaskJacobian);

      return true;
   }

   /**
    * Converts a {@link SpatialAccelerationCommand} into a {@link MotionQPInput}.
    * <p>
    * The idea is to convert the information held in the {@code commandToConvert} such that it ends
    * up being formulated as follows:<br>
    * J<sub>MxN</sub> * vDot<sub>Nx1</sub> = p<sub>Mx1</sub> <br>
    * where J is the M-by-N Jacobian matrix, vDot is the N-by-1 desired joint acceleration vector
    * that the QP solver is solving for, and p is the M-by-1 objective vector. M is called the task
    * size and N is the overall number of degrees of freedom (DoFs) to be controlled.
    * </p>
    * 
    * @return true if the command was successfully converted.
    */
   public boolean convertSpatialAccelerationCommand(SpatialAccelerationCommand commandToConvert, MotionQPInput motionQPInputToPack)
   {
      commandToConvert.getControlFrame(controlFrame);
      // Gets the M-by-6 selection matrix S.
      commandToConvert.getSelectionMatrix(controlFrame, tempSelectionMatrix);
      int taskSize = tempSelectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      motionQPInputToPack.reshape(taskSize);
      motionQPInputToPack.setConstraintType(commandToConvert.isHardConstraint() ? ConstraintType.EQUALITY : ConstraintType.OBJECTIVE);
      // If the task is setup as a hard constraint, there is no need for a weight matrix.
      if (!commandToConvert.isHardConstraint())
      {
         // Compute the M-by-M weight matrix W computed as follows: W = S * W * S^T
         motionQPInputToPack.setUseWeightScalar(false);
         tempTaskWeight.reshape(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
         commandToConvert.getWeightMatrix(controlFrame, tempTaskWeight);
         tempTaskWeightSubspace.reshape(taskSize, SpatialAccelerationVector.SIZE);
         DiagonalMatrixTools.postMult(tempSelectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
         CommonOps.multTransB(tempTaskWeightSubspace, tempSelectionMatrix, motionQPInputToPack.taskWeightMatrix);
      }

      RigidBody base = commandToConvert.getBase();
      RigidBody endEffector = commandToConvert.getEndEffector();

      jacobianCalculator.clear();
      jacobianCalculator.setKinematicChain(base, endEffector);
      jacobianCalculator.setJacobianFrame(controlFrame);
      jacobianCalculator.computeJacobianMatrix();
      jacobianCalculator.computeConvectiveTerm();

      /*
       * @formatter:off
       * Compute the M-by-1 task objective vector p as follows:
       * p = S * ( TDot - JDot * qDot )
       * where TDot is the 6-by-1 end-effector desired acceleration vector and (JDot * qDot) is the 6-by-1
       * convective term vector resulting from the Coriolis and Centrifugal effects.
       * @formatter:on
       */
      jacobianCalculator.getConvectiveTerm(convectiveTermMatrix);
      commandToConvert.getDesiredSpatialAcceleration(tempTaskObjective);
      CommonOps.subtractEquals(tempTaskObjective, convectiveTermMatrix);
      CommonOps.mult(tempSelectionMatrix, tempTaskObjective, motionQPInputToPack.taskObjective);

      // Compute the M-by-N task Jacobian: J = S * J
      // Step 1, let's get the 'small' Jacobian matrix j.
      // It is called small as its number of columns is equal to the number of DoFs to its kinematic chain which is way smaller than the number of robot DoFs.
      jacobianCalculator.getJacobianMatrix(tempSelectionMatrix, tempTaskJacobian);

      // Dealing with the primary base:
      RigidBody primaryBase = commandToConvert.getPrimaryBase();
      List<InverseDynamicsJoint> jointsUsedInTask = jacobianCalculator.getJointsFromBaseToEndEffector();

      // Step 2: The small Jacobian matrix into the full Jacobian matrix. Proper indexing has to be ensured, so it is handled by the jointIndexHandler.
      jointIndexHandler.compactBlockToFullBlockIgnoreUnindexedJoints(jointsUsedInTask, tempTaskJacobian, motionQPInputToPack.taskJacobian);

      if (primaryBase == null)
      { // No primary base provided for this task.
           // Record the resulting Jacobian matrix for the privileged configuration.
         recordTaskJacobian(motionQPInputToPack.taskJacobian);
         // We're done!
      }
      else
      { // A primary base has been provided, two things are happening here:
           // 1- A weight is applied on the joints between the base and the primary base with objective to reduce their involvement for this task.
        // 2- The Jacobian is transformed before being recorded such that the privileged configuration is only applied from the primary base to the end-effector.
         tempPrimaryTaskJacobian.set(motionQPInputToPack.taskJacobian);

         boolean isJointUpstreamOfPrimaryBase = false;
         for (int i = jointsUsedInTask.size() - 1; i >= 0; i--)
         {
            InverseDynamicsJoint joint = jointsUsedInTask.get(i);

            if (joint.getSuccessor() == primaryBase)
               isJointUpstreamOfPrimaryBase = true;

            if (isJointUpstreamOfPrimaryBase)
            { // The current joint is located between the base and the primary base:
                 // Find the column indices corresponding to this joint (it is usually only one index except for the floating joint which has 6).
               int[] jointIndices = jointIndexHandler.getJointIndices(joint);

               for (int dofIndex : jointIndices)
               {
                  double scaleFactor = secondaryTaskJointsWeight.getDoubleValue();
                  if (commandToConvert.scaleSecondaryTaskJointWeight())
                     scaleFactor = commandToConvert.getSecondaryTaskJointWeightScale();

                  // Apply a down-scale on the task Jacobian for the joint's column(s) so it has lower priority in the optimization.
                  MatrixTools.scaleColumn(scaleFactor, dofIndex, motionQPInputToPack.taskJacobian);
                  // Zero out the task Jacobian at the joint's column(s) so it is removed from the nullspace calculation for applying the privileged configuration.
                  MatrixTools.zeroColumn(dofIndex, tempPrimaryTaskJacobian);
               }
            }
         }

         // Record the resulting Jacobian matrix which only zeros before the primary base for the privileged configuration.
         recordTaskJacobian(tempPrimaryTaskJacobian);
         // We're done!
      }

      return true;
   }

   /**
    * Converts a {@link SpatialVelocityCommand} into a {@link MotionQPInput}.
    * <p>
    * The idea is to convert the information held in the {@code commandToConvert} such that it ends
    * up being formulated as follows:<br>
    * J<sub>MxN</sub> * v<sub>Nx1</sub> = p<sub>Mx1</sub> <br>
    * where J is the M-by-N Jacobian matrix, v is the N-by-1 desired joint velocity vector
    * that the QP solver is solving for, and p is the M-by-1 objective vector. M is called the task
    * size and N is the overall number of degrees of freedom (DoFs) to be controlled.
    * </p>
    * 
    * @return true if the command was successfully converted.
    */
   public boolean convertSpatialVelocityCommand(SpatialVelocityCommand commandToConvert, MotionQPInput motionQPInputToPack)
   {
      // Gets the M-by-6 selection matrix S.
      commandToConvert.getControlFrame(controlFrame);
      commandToConvert.getSelectionMatrix(controlFrame, tempSelectionMatrix);
      int taskSize = tempSelectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      ConstraintType constraintType = commandToConvert.getConstraintType();
      motionQPInputToPack.reshape(taskSize);
      motionQPInputToPack.setConstraintType(constraintType);
      // If the task is setup as a hard constraint, there is no need for a weight matrix.
      if (constraintType == ConstraintType.OBJECTIVE)
      {
         // Compute the M-by-M weight matrix W computed as follows: W = S * W * S^T
         motionQPInputToPack.setUseWeightScalar(false);
         tempTaskWeight.reshape(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
         commandToConvert.getWeightMatrix(controlFrame, tempTaskWeight);
         tempTaskWeightSubspace.reshape(taskSize, SpatialAccelerationVector.SIZE);
         DiagonalMatrixTools.postMult(tempSelectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
         CommonOps.multTransB(tempTaskWeightSubspace, tempSelectionMatrix, motionQPInputToPack.taskWeightMatrix);
      }

      RigidBody base = commandToConvert.getBase();
      RigidBody endEffector = commandToConvert.getEndEffector();

      jacobianCalculator.clear();
      jacobianCalculator.setKinematicChain(base, endEffector);
      jacobianCalculator.setJacobianFrame(controlFrame);
      jacobianCalculator.computeJacobianMatrix();

      /*
       * @formatter:off
       * Compute the M-by-1 task objective vector p as follows:
       * p = S * T
       * where T is the 6-by-1 end-effector desired velocity vector.
       * @formatter:on
       */
      commandToConvert.getDesiredSpatialVelocity(tempTaskObjective);
      CommonOps.mult(tempSelectionMatrix, tempTaskObjective, motionQPInputToPack.taskObjective);

      // Compute the M-by-N task Jacobian: J = S * J
      // Step 1, let's get the 'small' Jacobian matrix j.
      // It is called small as its number of columns is equal to the number of DoFs to its kinematic chain which is way smaller than the number of robot DoFs.
      jacobianCalculator.getJacobianMatrix(tempSelectionMatrix, tempTaskJacobian);

      // Dealing with the primary base:
      RigidBody primaryBase = commandToConvert.getPrimaryBase();
      List<InverseDynamicsJoint> jointsUsedInTask = jacobianCalculator.getJointsFromBaseToEndEffector();

      // Step 2: The small Jacobian matrix into the full Jacobian matrix. Proper indexing has to be ensured, so it is handled by the jointIndexHandler.
      jointIndexHandler.compactBlockToFullBlockIgnoreUnindexedJoints(jointsUsedInTask, tempTaskJacobian, motionQPInputToPack.taskJacobian);

      if (primaryBase == null)
      { // No primary base provided for this task.
           // Record the resulting Jacobian matrix for the privileged configuration.
         recordTaskJacobian(motionQPInputToPack.taskJacobian);
         // We're done!
      }
      else
      { // A primary base has been provided, two things are happening here:
           // 1- A weight is applied on the joints between the base and the primary base with objective to reduce their involvement for this task.
        // 2- The Jacobian is transformed before being recorded such that the privileged configuration is only applied from the primary base to the end-effector.
         tempPrimaryTaskJacobian.set(motionQPInputToPack.taskJacobian);

         boolean isJointUpstreamOfPrimaryBase = false;
         for (int i = jointsUsedInTask.size() - 1; i >= 0; i--)
         {
            InverseDynamicsJoint joint = jointsUsedInTask.get(i);

            if (joint.getSuccessor() == primaryBase)
               isJointUpstreamOfPrimaryBase = true;

            if (isJointUpstreamOfPrimaryBase)
            { // The current joint is located between the base and the primary base:
                 // Find the column indices corresponding to this joint (it is usually only one index except for the floating joint which has 6).
               int[] jointIndices = jointIndexHandler.getJointIndices(joint);

               for (int dofIndex : jointIndices)
               {
                  double scaleFactor = secondaryTaskJointsWeight.getDoubleValue();
                  if (commandToConvert.scaleSecondaryTaskJointWeight())
                     scaleFactor = commandToConvert.getSecondaryTaskJointWeightScale();

                  // Apply a down-scale on the task Jacobian for the joint's column(s) so it has lower priority in the optimization.
                  MatrixTools.scaleColumn(scaleFactor, dofIndex, motionQPInputToPack.taskJacobian);
                  // Zero out the task Jacobian at the joint's column(s) so it is removed from the nullspace calculation for applying the privileged configuration.
                  MatrixTools.zeroColumn(dofIndex, tempPrimaryTaskJacobian);
               }
            }
         }

         // Record the resulting Jacobian matrix which only zeros before the primary base for the privileged configuration.
         recordTaskJacobian(tempPrimaryTaskJacobian);
         // We're done!
      }

      return true;
   }

   /**
    * Converts a {@link MomentumRateCommand} into a {@link MotionQPInput}.
    * 
    * @return true if the command was successfully converted.
    */
   public boolean convertMomentumRateCommand(MomentumRateCommand commandToConvert, MotionQPInput motionQPInputToPack)
   {
      commandToConvert.getSelectionMatrix(centerOfMassFrame, tempSelectionMatrix);
      int taskSize = tempSelectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      motionQPInputToPack.reshape(taskSize);
      motionQPInputToPack.setUseWeightScalar(false);
      motionQPInputToPack.setConstraintType(ConstraintType.OBJECTIVE);

      // Compute the weight: W = S * W * S^T
      tempTaskWeight.reshape(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
      commandToConvert.getWeightMatrix(tempTaskWeight);
      tempTaskWeightSubspace.reshape(taskSize, SpatialAccelerationVector.SIZE);
      DiagonalMatrixTools.postMult(tempSelectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
      CommonOps.multTransB(tempTaskWeightSubspace, tempSelectionMatrix, motionQPInputToPack.taskWeightMatrix);

      // Compute the task Jacobian: J = S * A
      DenseMatrix64F centroidalMomentumMatrix = getCentroidalMomentumMatrix();
      CommonOps.mult(tempSelectionMatrix, centroidalMomentumMatrix, motionQPInputToPack.taskJacobian);

      commandToConvert.getMomentumRate(angularMomentum, linearMomentum);
      angularMomentum.changeFrame(centerOfMassFrame);
      linearMomentum.changeFrame(centerOfMassFrame);
      angularMomentum.get(0, tempTaskObjective);
      linearMomentum.get(3, tempTaskObjective);
      DenseMatrix64F convectiveTerm = centroidalMomentumHandler.getCentroidalMomentumConvectiveTerm();

      // Compute the task objective: p = S * ( hDot - ADot qDot )
      CommonOps.subtractEquals(tempTaskObjective, convectiveTerm);
      CommonOps.mult(tempSelectionMatrix, tempTaskObjective, motionQPInputToPack.taskObjective);

      recordTaskJacobian(motionQPInputToPack.taskJacobian);

      return true;
   }

   /**
    * Converts a {@link MomentumCommand} into a {@link MotionQPInput}.
    * 
    * @return true if the command was successfully converted.
    */
   public boolean convertMomentumCommand(MomentumCommand commandToConvert, MotionQPInput motionQPInputToPack)
   {
      commandToConvert.getSelectionMatrix(centerOfMassFrame, tempSelectionMatrix);
      int taskSize = tempSelectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      motionQPInputToPack.reshape(taskSize);
      motionQPInputToPack.setUseWeightScalar(false);
      motionQPInputToPack.setConstraintType(ConstraintType.OBJECTIVE);

      // Compute the weight: W = S * W * S^T
      tempTaskWeight.reshape(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
      commandToConvert.getWeightMatrix(tempTaskWeight);
      tempTaskWeightSubspace.reshape(taskSize, SpatialAccelerationVector.SIZE);
      DiagonalMatrixTools.postMult(tempSelectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
      CommonOps.multTransB(tempTaskWeightSubspace, tempSelectionMatrix, motionQPInputToPack.taskWeightMatrix);

      // Compute the task Jacobian: J = S * A
      DenseMatrix64F centroidalMomentumMatrix = getCentroidalMomentumMatrix();
      CommonOps.mult(tempSelectionMatrix, centroidalMomentumMatrix, motionQPInputToPack.taskJacobian);

      commandToConvert.getMomentumRate(angularMomentum, linearMomentum);
      angularMomentum.changeFrame(centerOfMassFrame);
      linearMomentum.changeFrame(centerOfMassFrame);
      angularMomentum.get(0, tempTaskObjective);
      linearMomentum.get(3, tempTaskObjective);

      // Compute the task objective: p = S * h
      CommonOps.mult(tempSelectionMatrix, tempTaskObjective, motionQPInputToPack.taskObjective);

      recordTaskJacobian(motionQPInputToPack.taskJacobian);

      return true;
   }

   /**
    * Converts a {@link JointspaceAccelerationCommand} into a {@link MotionQPInput}.
    * 
    * @return true if the command was successfully converted.
    */
   public boolean convertJointspaceAccelerationCommand(JointspaceAccelerationCommand commandToConvert, MotionQPInput motionQPInputToPack)
   {
      int taskSize = ScrewTools.computeDegreesOfFreedom(commandToConvert.getJoints());

      if (taskSize == 0)
         return false;

      motionQPInputToPack.reshape(taskSize);
      motionQPInputToPack.setConstraintType(commandToConvert.isHardConstraint() ? ConstraintType.EQUALITY :ConstraintType.OBJECTIVE);
      motionQPInputToPack.taskJacobian.zero();
      motionQPInputToPack.taskWeightMatrix.zero();
      motionQPInputToPack.setUseWeightScalar(false);

      int row = 0;
      for (int jointIndex = 0; jointIndex < commandToConvert.getNumberOfJoints(); jointIndex++)
      {
         InverseDynamicsJoint joint = commandToConvert.getJoint(jointIndex);
         double weight = commandToConvert.getWeight(jointIndex);
         int[] columns = jointIndexHandler.getJointIndices(joint);
         if (columns == null)
            return false;

         CommonOps.insert(commandToConvert.getDesiredAcceleration(jointIndex), motionQPInputToPack.taskObjective, row, 0);
         for (int column : columns)
         {
            motionQPInputToPack.taskJacobian.set(row, column, 1.0);
            motionQPInputToPack.taskWeightMatrix.set(row, row, weight);
            row++;
         }
      }

      recordTaskJacobian(motionQPInputToPack.taskJacobian);
      return true;
   }

   /**
    * Converts a {@link JointspaceVelocityCommand} into a {@link MotionQPInput}.
    * 
    * @return true if the command was successfully converted.
    */
   public boolean convertJointspaceVelocityCommand(JointspaceVelocityCommand commandToConvert, MotionQPInput motionQPInputToPack)
   {
      int taskSize = ScrewTools.computeDegreesOfFreedom(commandToConvert.getJoints());

      if (taskSize == 0)
         return false;

      motionQPInputToPack.reshape(taskSize);
      motionQPInputToPack.setConstraintType(commandToConvert.isHardConstraint() ? ConstraintType.EQUALITY :ConstraintType.OBJECTIVE);
      motionQPInputToPack.taskJacobian.zero();
      motionQPInputToPack.taskWeightMatrix.zero();
      motionQPInputToPack.setUseWeightScalar(false);

      int row = 0;
      for (int jointIndex = 0; jointIndex < commandToConvert.getNumberOfJoints(); jointIndex++)
      {
         InverseDynamicsJoint joint = commandToConvert.getJoint(jointIndex);
         double weight = commandToConvert.getWeight(jointIndex);
         int[] columns = jointIndexHandler.getJointIndices(joint);
         if (columns == null)
            return false;

         CommonOps.insert(commandToConvert.getDesiredVelocity(jointIndex), motionQPInputToPack.taskObjective, row, 0);
         for (int column : columns)
         {
            motionQPInputToPack.taskJacobian.set(row, column, 1.0);
            motionQPInputToPack.taskWeightMatrix.set(row, row, weight);
            row++;
         }
      }

      recordTaskJacobian(motionQPInputToPack.taskJacobian);
      return true;
   }

   private void recordTaskJacobian(DenseMatrix64F taskJacobian)
   {
      int taskSize = taskJacobian.getNumRows();
      allTaskJacobian.reshape(allTaskJacobian.getNumRows() + taskSize, numberOfDoFs, true);
      CommonOps.insert(taskJacobian, allTaskJacobian, allTaskJacobian.getNumRows() - taskSize, 0);
   }

   public DenseMatrix64F getCentroidalMomentumMatrix()
   {
      return centroidalMomentumHandler.getCentroidalMomentumMatrixPart(jointsToOptimizeFor);
   }

   public DenseMatrix64F getCentroidalMomentumConvectiveTerm()
   {
      return centroidalMomentumHandler.getCentroidalMomentumConvectiveTerm();
   }

   public SpatialForceVector computeCentroidalMomentumRateFromSolution(DenseMatrix64F jointAccelerations)
   {
      centroidalMomentumHandler.computeCentroidalMomentumRate(jointsToOptimizeFor, jointAccelerations);
      return centroidalMomentumHandler.getCentroidalMomentumRate();
   }
}
