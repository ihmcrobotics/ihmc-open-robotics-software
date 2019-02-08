package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointspaceVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.inverseKinematics.JointPrivilegedConfigurationHandler;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.CentroidalMomentumRateCalculator;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.linearAlgebra.commonOps.NativeCommonOps;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class MotionQPInputCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble nullspaceProjectionAlpha;
   private final YoDouble secondaryTaskJointsWeight = new YoDouble("secondaryTaskJointsWeight", registry);

   private final PoseReferenceFrame controlFrame = new PoseReferenceFrame("controlFrame", worldFrame);
   private final GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();

   private final OneDoFJointBasics[] oneDoFJoints;

   private final CentroidalMomentumRateCalculator centroidalMomentumRateCalculator;

   private final JointPrivilegedConfigurationHandler privilegedConfigurationHandler;

   private final DenseMatrix64F tempPrimaryTaskJacobian = new DenseMatrix64F(SpatialVector.SIZE, 12);

   private final DenseMatrix64F tempTaskJacobian = new DenseMatrix64F(SpatialVector.SIZE, 12);
   private final DenseMatrix64F projectedTaskJacobian = new DenseMatrix64F(SpatialVector.SIZE, 12);
   private final DenseMatrix64F tempTaskObjective = new DenseMatrix64F(SpatialVector.SIZE, 1);
   private final DenseMatrix64F tempTaskWeight = new DenseMatrix64F(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);
   private final DenseMatrix64F tempTaskWeightSubspace = new DenseMatrix64F(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);

   private final DenseMatrix64F tempSelectionMatrix = new DenseMatrix64F(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);

   private final FrameVector3D angularMomentum = new FrameVector3D();
   private final FrameVector3D linearMomentum = new FrameVector3D();
   private final ReferenceFrame centerOfMassFrame;

   private final JointIndexHandler jointIndexHandler;

   private final DenseMatrix64F allTaskJacobian;

   private final int numberOfDoFs;

   public MotionQPInputCalculator(ReferenceFrame centerOfMassFrame, CentroidalMomentumRateCalculator centroidalMomentumRateCalculator, JointIndexHandler jointIndexHandler,
                                  JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters, YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      this.jointIndexHandler = jointIndexHandler;
      this.centroidalMomentumRateCalculator = centroidalMomentumRateCalculator;
      oneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      numberOfDoFs = jointIndexHandler.getNumberOfDoFs();

      if (jointPrivilegedConfigurationParameters != null)
      {
         privilegedConfigurationHandler = new JointPrivilegedConfigurationHandler(oneDoFJoints, jointPrivilegedConfigurationParameters, registry);
         nullspaceProjectionAlpha = new YoDouble("nullspaceProjectionAlpha", registry);
         nullspaceProjectionAlpha.set(jointPrivilegedConfigurationParameters.getNullspaceProjectionAlpha());
      }
      else
      {
         privilegedConfigurationHandler = null;
         nullspaceProjectionAlpha = null;
      }

      allTaskJacobian = new DenseMatrix64F(numberOfDoFs, numberOfDoFs);
      secondaryTaskJointsWeight.set(1.0); // TODO Needs to be rethought, it doesn't seem to be that useful.

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      centroidalMomentumRateCalculator.reset();
      allTaskJacobian.reshape(0, numberOfDoFs);
   }

   public void updatePrivilegedConfiguration(PrivilegedConfigurationCommand command)
   {
      if (privilegedConfigurationHandler == null)
         throw new NullPointerException("JointPrivilegedConfigurationParameters have to be set to enable this feature.");
      privilegedConfigurationHandler.submitPrivilegedConfigurationCommand(command);
   }


   public void submitPrivilegedAccelerations(PrivilegedJointSpaceCommand command)
   {
      if (privilegedConfigurationHandler == null)
         throw new NullPointerException("JointPrivilegedConfigurationParameters have to be set to enable this feature.");
      privilegedConfigurationHandler.submitPrivilegedAccelerations(command);
   }

   public void submitPrivilegedVelocities(PrivilegedJointSpaceCommand command)
   {
      if (privilegedConfigurationHandler == null)
         throw new NullPointerException("JointPrivilegedConfigurationParameters have to be set to enable this feature.");
      privilegedConfigurationHandler.submitPrivilegedVelocities(command);
   }


   public boolean computePrivilegedJointAccelerations(QPInput motionQPInputToPack)
   {
      if (privilegedConfigurationHandler == null || !privilegedConfigurationHandler.isEnabled())
         return false;

      privilegedConfigurationHandler.computePrivilegedJointAccelerations();

      motionQPInputToPack.setConstraintType(ConstraintType.OBJECTIVE);
      motionQPInputToPack.setUseWeightScalar(false);

      int taskSize = 0;

      DenseMatrix64F selectionMatrix = privilegedConfigurationHandler.getSelectionMatrix();
      int robotTaskSize = selectionMatrix.getNumRows();

      if (robotTaskSize > 0)
      {
         OneDoFJointBasics[] joints = privilegedConfigurationHandler.getJoints();
         tempTaskJacobian.reshape(robotTaskSize, numberOfDoFs);
         boolean success = jointIndexHandler.compactBlockToFullBlock(joints, selectionMatrix, tempTaskJacobian);

         if (success)
         {
            motionQPInputToPack.reshape(robotTaskSize);
            NativeCommonOps.projectOnNullspace(tempTaskJacobian, allTaskJacobian, projectedTaskJacobian, nullspaceProjectionAlpha.getValue());

            CommonOps.insert(projectedTaskJacobian, motionQPInputToPack.taskJacobian, taskSize, 0);
            CommonOps.insert(privilegedConfigurationHandler.getPrivilegedJointAccelerations(), motionQPInputToPack.taskObjective, taskSize, 0);
            CommonOps.insert(privilegedConfigurationHandler.getWeights(), motionQPInputToPack.taskWeightMatrix, taskSize, taskSize);
         }
      }

      return robotTaskSize > 0;
   }

   public boolean computePrivilegedJointVelocities(QPInput motionQPInputToPack)
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

      OneDoFJointBasics[] joints = privilegedConfigurationHandler.getJoints();
      boolean success = jointIndexHandler.compactBlockToFullBlock(joints, selectionMatrix, motionQPInputToPack.taskJacobian);

      if (!success)
         return false;

      NativeCommonOps.projectOnNullspace(motionQPInputToPack.taskJacobian, allTaskJacobian, projectedTaskJacobian, nullspaceProjectionAlpha.getValue());
      motionQPInputToPack.taskJacobian.set(projectedTaskJacobian);

      return true;
   }

   /**
    * Converts a {@link SpatialAccelerationCommand} into a {@link QPInput}.
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
   public boolean convertSpatialAccelerationCommand(SpatialAccelerationCommand commandToConvert, QPInput motionQPInputToPack)
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
         tempTaskWeight.reshape(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);
         tempTaskWeightSubspace.reshape(taskSize, SpatialAcceleration.SIZE);
         commandToConvert.getWeightMatrix(controlFrame, tempTaskWeight);
         CommonOps.mult(tempSelectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
         CommonOps.multTransB(tempTaskWeightSubspace, tempSelectionMatrix, motionQPInputToPack.taskWeightMatrix);
      }

      RigidBodyBasics base = commandToConvert.getBase();
      RigidBodyBasics endEffector = commandToConvert.getEndEffector();

      jacobianCalculator.clear();
      jacobianCalculator.setKinematicChain(base, endEffector);
      jacobianCalculator.setJacobianFrame(controlFrame);
      jacobianCalculator.reset();

      /*
       * @formatter:off
       * Compute the M-by-1 task objective vector p as follows:
       * p = S * ( TDot - JDot * qDot )
       * where TDot is the 6-by-1 end-effector desired acceleration vector and (JDot * qDot) is the 6-by-1
       * convective term vector resulting from the Coriolis and Centrifugal effects.
       * @formatter:on
       */
      commandToConvert.getDesiredSpatialAcceleration(tempTaskObjective);
      CommonOps.subtractEquals(tempTaskObjective, jacobianCalculator.getConvectiveTermMatrix());
      CommonOps.mult(tempSelectionMatrix, tempTaskObjective, motionQPInputToPack.taskObjective);

      // Compute the M-by-N task Jacobian: J = S * J
      // Step 1, let's get the 'small' Jacobian matrix j.
      // It is called small as its number of columns is equal to the number of DoFs to its kinematic chain which is way smaller than the number of robot DoFs.
      tempTaskJacobian.reshape(taskSize, jacobianCalculator.getNumberOfDegreesOfFreedom());
      CommonOps.mult(tempSelectionMatrix, jacobianCalculator.getJacobianMatrix(), tempTaskJacobian);

      // Dealing with the primary base:
      RigidBodyBasics primaryBase = commandToConvert.getPrimaryBase();
      List<JointReadOnly> jointsUsedInTask = jacobianCalculator.getJointsFromBaseToEndEffector();

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
            JointReadOnly joint = jointsUsedInTask.get(i);

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
    * Converts a {@link SpatialVelocityCommand} into a {@link QPInput}.
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
   public boolean convertSpatialVelocityCommand(SpatialVelocityCommand commandToConvert, QPInput motionQPInputToPack)
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
         tempTaskWeight.reshape(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);
         tempTaskWeightSubspace.reshape(taskSize, SpatialAcceleration.SIZE);
         commandToConvert.getWeightMatrix(controlFrame, tempTaskWeight);
         CommonOps.mult(tempSelectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
         CommonOps.multTransB(tempTaskWeightSubspace, tempSelectionMatrix, motionQPInputToPack.taskWeightMatrix);
      }

      RigidBodyBasics base = commandToConvert.getBase();
      RigidBodyBasics endEffector = commandToConvert.getEndEffector();

      jacobianCalculator.clear();
      jacobianCalculator.setKinematicChain(base, endEffector);
      jacobianCalculator.setJacobianFrame(controlFrame);
      jacobianCalculator.reset();

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
      tempTaskJacobian.reshape(taskSize, jacobianCalculator.getNumberOfDegreesOfFreedom());
      CommonOps.mult(tempSelectionMatrix, jacobianCalculator.getJacobianMatrix(), tempTaskJacobian);

      // Dealing with the primary base:
      RigidBodyBasics primaryBase = commandToConvert.getPrimaryBase();
      List<JointReadOnly> jointsUsedInTask = jacobianCalculator.getJointsFromBaseToEndEffector();

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
            JointReadOnly joint = jointsUsedInTask.get(i);

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
    * Converts a {@link MomentumRateCommand} into a {@link QPInput}.
    * 
    * @return true if the command was successfully converted.
    */
   public boolean convertMomentumRateCommand(MomentumRateCommand commandToConvert, QPInput motionQPInputToPack)
   {
      commandToConvert.getSelectionMatrix(centerOfMassFrame, tempSelectionMatrix);
      int taskSize = tempSelectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      motionQPInputToPack.reshape(taskSize);
      motionQPInputToPack.setUseWeightScalar(false);
      motionQPInputToPack.setConstraintType(ConstraintType.OBJECTIVE);

      // Compute the weight: W = S * W * S^T
      tempTaskWeight.reshape(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);
      tempTaskWeightSubspace.reshape(taskSize, SpatialAcceleration.SIZE);
      commandToConvert.getWeightMatrix(tempTaskWeight);
      CommonOps.mult(tempSelectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
      CommonOps.multTransB(tempTaskWeightSubspace, tempSelectionMatrix, motionQPInputToPack.taskWeightMatrix);

      // Compute the task Jacobian: J = S * A
      DenseMatrix64F centroidalMomentumMatrix = getCentroidalMomentumMatrix();
      CommonOps.mult(tempSelectionMatrix, centroidalMomentumMatrix, motionQPInputToPack.taskJacobian);

      commandToConvert.getMomentumRate(angularMomentum, linearMomentum);
      angularMomentum.changeFrame(centerOfMassFrame);
      linearMomentum.changeFrame(centerOfMassFrame);
      angularMomentum.get(0, tempTaskObjective);
      linearMomentum.get(3, tempTaskObjective);
      DenseMatrix64F convectiveTerm = centroidalMomentumRateCalculator.getBiasSpatialForceMatrix();

      // Compute the task objective: p = S * ( hDot - ADot qDot )
      CommonOps.subtractEquals(tempTaskObjective, convectiveTerm);
      CommonOps.mult(tempSelectionMatrix, tempTaskObjective, motionQPInputToPack.taskObjective);

      recordTaskJacobian(motionQPInputToPack.taskJacobian);

      return true;
   }

   /**
    * Converts a {@link MomentumCommand} into a {@link QPInput}.
    * 
    * @return true if the command was successfully converted.
    */
   public boolean convertMomentumCommand(MomentumCommand commandToConvert, QPInput motionQPInputToPack)
   {
      commandToConvert.getSelectionMatrix(centerOfMassFrame, tempSelectionMatrix);
      int taskSize = tempSelectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      motionQPInputToPack.reshape(taskSize);
      motionQPInputToPack.setUseWeightScalar(false);
      motionQPInputToPack.setConstraintType(ConstraintType.OBJECTIVE);

      // Compute the weight: W = S * W * S^T
      tempTaskWeight.reshape(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);
      tempTaskWeightSubspace.reshape(taskSize, SpatialAcceleration.SIZE);
      commandToConvert.getWeightMatrix(tempTaskWeight);
      CommonOps.mult(tempSelectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
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
    * Converts a {@link JointspaceAccelerationCommand} into a {@link QPInput}.
    * 
    * @return true if the command was successfully converted.
    */
   public boolean convertJointspaceAccelerationCommand(JointspaceAccelerationCommand commandToConvert, QPInput motionQPInputToPack)
   {
      int taskSize = MultiBodySystemTools.computeDegreesOfFreedom(commandToConvert.getJoints());

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
         JointBasics joint = commandToConvert.getJoint(jointIndex);
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
    * Converts a {@link JointspaceVelocityCommand} into a {@link QPInput}.
    * 
    * @return true if the command was successfully converted.
    */
   public boolean convertJointspaceVelocityCommand(JointspaceVelocityCommand commandToConvert, QPInput motionQPInputToPack)
   {
      int taskSize = MultiBodySystemTools.computeDegreesOfFreedom(commandToConvert.getJoints());

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
         JointBasics joint = commandToConvert.getJoint(jointIndex);
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
      return centroidalMomentumRateCalculator.getCentroidalMomentumMatrix();
   }

   public DenseMatrix64F getCentroidalMomentumConvectiveTerm()
   {
      return centroidalMomentumRateCalculator.getBiasSpatialForceMatrix();
   }

   private final SpatialForce momentumRate = new SpatialForce();

   public SpatialForceReadOnly computeCentroidalMomentumRateFromSolution(DenseMatrix64F jointAccelerations)
   {
      centroidalMomentumRateCalculator.getMomentumRate(jointAccelerations, momentumRate);
      return momentumRate;
   }

   private final Momentum momentum = new Momentum();

   public MomentumReadOnly computeCentroidalMomentumFromSolution(DenseMatrix64F jointVelocities)
   {
      centroidalMomentumRateCalculator.getMomentum(jointVelocities, momentum);
      return momentum;
   }
}
