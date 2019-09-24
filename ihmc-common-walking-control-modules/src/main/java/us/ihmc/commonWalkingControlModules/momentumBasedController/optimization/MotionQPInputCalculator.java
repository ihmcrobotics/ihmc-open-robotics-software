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
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.LinearMomentumConvexConstraint2DCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.inverseKinematics.InverseKinematicsQPSolver;
import us.ihmc.commonWalkingControlModules.inverseKinematics.JointPrivilegedConfigurationHandler;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
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

   private final CentroidalMomentumCalculator centroidalMomentumCalculator;
   private final CentroidalMomentumRateCalculator centroidalMomentumRateCalculator;

   private final JointPrivilegedConfigurationHandler privilegedConfigurationHandler;

   private final DenseMatrix64F tempPrimaryTaskJacobian = new DenseMatrix64F(SpatialVector.SIZE, 12);

   private final DenseMatrix64F tempTaskJacobian = new DenseMatrix64F(SpatialVector.SIZE, 12);
   private final DenseMatrix64F projectedTaskJacobian = new DenseMatrix64F(SpatialVector.SIZE, 12);
   private final DenseMatrix64F tempTaskObjective = new DenseMatrix64F(SpatialVector.SIZE, 1);
   private final DenseMatrix64F tempTaskWeight = new DenseMatrix64F(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);
   private final DenseMatrix64F tempTaskWeightSubspace = new DenseMatrix64F(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);

   private final DenseMatrix64F lineConstraintSelection = new DenseMatrix64F(1, 2);
   private final DenseMatrix64F lineConstraintJacobian = new DenseMatrix64F(1, 12);

   private final DenseMatrix64F tempSelectionMatrix = new DenseMatrix64F(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);

   private final FrameVector3D angularMomentum = new FrameVector3D();
   private final FrameVector3D linearMomentum = new FrameVector3D();
   private final ReferenceFrame centerOfMassFrame;

   private final JointIndexHandler jointIndexHandler;

   private final DenseMatrix64F allTaskJacobian;

   private final int numberOfDoFs;

   public MotionQPInputCalculator(ReferenceFrame centerOfMassFrame, CentroidalMomentumRateCalculator centroidalMomentumRateCalculator,
                                  JointIndexHandler jointIndexHandler, JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters,
                                  YoVariableRegistry parentRegistry)
   {
      this(centerOfMassFrame, null, centroidalMomentumRateCalculator, jointIndexHandler, jointPrivilegedConfigurationParameters, parentRegistry);
   }

   public MotionQPInputCalculator(ReferenceFrame centerOfMassFrame, CentroidalMomentumCalculator centroidalMomentumCalculator,
                                  JointIndexHandler jointIndexHandler, JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters,
                                  YoVariableRegistry parentRegistry)
   {
      this(centerOfMassFrame, centroidalMomentumCalculator, null, jointIndexHandler, jointPrivilegedConfigurationParameters, parentRegistry);
   }

   private MotionQPInputCalculator(ReferenceFrame centerOfMassFrame, CentroidalMomentumCalculator centroidalMomentumCalculator,
                                   CentroidalMomentumRateCalculator centroidalMomentumRateCalculator, JointIndexHandler jointIndexHandler,
                                   JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters, YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      this.jointIndexHandler = jointIndexHandler;
      this.centroidalMomentumCalculator = centroidalMomentumCalculator;
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
      if (centroidalMomentumRateCalculator != null)
         centroidalMomentumRateCalculator.reset();
      else
         centroidalMomentumCalculator.reset();
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

   public boolean computePrivilegedJointAccelerations(QPInput qpInputToPack)
   {
      if (privilegedConfigurationHandler == null || !privilegedConfigurationHandler.isEnabled())
         return false;

      privilegedConfigurationHandler.computePrivilegedJointAccelerations();

      qpInputToPack.setConstraintType(ConstraintType.OBJECTIVE);
      qpInputToPack.setUseWeightScalar(false);

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
            qpInputToPack.reshape(robotTaskSize);
            NativeCommonOps.projectOnNullspace(tempTaskJacobian, allTaskJacobian, projectedTaskJacobian, nullspaceProjectionAlpha.getValue());

            CommonOps.insert(projectedTaskJacobian, qpInputToPack.taskJacobian, taskSize, 0);
            CommonOps.insert(privilegedConfigurationHandler.getPrivilegedJointAccelerations(), qpInputToPack.taskObjective, taskSize, 0);
            CommonOps.insert(privilegedConfigurationHandler.getWeights(), qpInputToPack.taskWeightMatrix, taskSize, taskSize);
         }
      }

      return robotTaskSize > 0;
   }

   public boolean computePrivilegedJointVelocities(QPInput qpInputToPack)
   {
      if (privilegedConfigurationHandler == null || !privilegedConfigurationHandler.isEnabled())
         return false;

      privilegedConfigurationHandler.computePrivilegedJointVelocities();

      qpInputToPack.setConstraintType(ConstraintType.OBJECTIVE);
      qpInputToPack.setUseWeightScalar(false);

      DenseMatrix64F selectionMatrix = privilegedConfigurationHandler.getSelectionMatrix();

      int taskSize = selectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      qpInputToPack.reshape(taskSize);

      qpInputToPack.setTaskObjective(privilegedConfigurationHandler.getPrivilegedJointVelocities());
      qpInputToPack.setTaskWeightMatrix(privilegedConfigurationHandler.getWeights());

      OneDoFJointBasics[] joints = privilegedConfigurationHandler.getJoints();
      boolean success = jointIndexHandler.compactBlockToFullBlock(joints, selectionMatrix, qpInputToPack.taskJacobian);

      if (!success)
         return false;

      NativeCommonOps.projectOnNullspace(qpInputToPack.taskJacobian, allTaskJacobian, projectedTaskJacobian, nullspaceProjectionAlpha.getValue());
      qpInputToPack.taskJacobian.set(projectedTaskJacobian);

      return true;
   }

   /**
    * Converts a {@link SpatialAccelerationCommand} into a {@link QPInput}.
    * <p>
    * The idea is to convert the information held in the {@code commandToConvert} such that it ends up
    * being formulated as follows:<br>
    * J<sub>MxN</sub> * vDot<sub>Nx1</sub> = p<sub>Mx1</sub> <br>
    * where J is the M-by-N Jacobian matrix, vDot is the N-by-1 desired joint acceleration vector that
    * the QP solver is solving for, and p is the M-by-1 objective vector. M is called the task size and
    * N is the overall number of degrees of freedom (DoFs) to be controlled.
    * </p>
    * 
    * @return true if the command was successfully converted.
    */
   public boolean convertSpatialAccelerationCommand(SpatialAccelerationCommand commandToConvert, QPInput qpInputToPack)
   {
      commandToConvert.getControlFrame(controlFrame);
      // Gets the M-by-6 selection matrix S.
      commandToConvert.getSelectionMatrix(controlFrame, tempSelectionMatrix);
      int taskSize = tempSelectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      qpInputToPack.reshape(taskSize);
      qpInputToPack.setConstraintType(commandToConvert.isHardConstraint() ? ConstraintType.EQUALITY : ConstraintType.OBJECTIVE);
      // If the task is setup as a hard constraint, there is no need for a weight matrix.
      if (!commandToConvert.isHardConstraint())
      {
         // Compute the M-by-M weight matrix W computed as follows: W = S * W * S^T
         qpInputToPack.setUseWeightScalar(false);
         tempTaskWeight.reshape(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);
         tempTaskWeightSubspace.reshape(taskSize, SpatialAcceleration.SIZE);
         commandToConvert.getWeightMatrix(controlFrame, tempTaskWeight);
         CommonOps.mult(tempSelectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
         CommonOps.multTransB(tempTaskWeightSubspace, tempSelectionMatrix, qpInputToPack.taskWeightMatrix);
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
      CommonOps.mult(tempSelectionMatrix, tempTaskObjective, qpInputToPack.taskObjective);

      // Compute the M-by-N task Jacobian: J = S * J
      // Step 1, let's get the 'small' Jacobian matrix j.
      // It is called small as its number of columns is equal to the number of DoFs to its kinematic chain which is way smaller than the number of robot DoFs.
      tempTaskJacobian.reshape(taskSize, jacobianCalculator.getNumberOfDegreesOfFreedom());
      CommonOps.mult(tempSelectionMatrix, jacobianCalculator.getJacobianMatrix(), tempTaskJacobian);

      // Dealing with the primary base:
      RigidBodyBasics primaryBase = commandToConvert.getPrimaryBase();
      List<JointReadOnly> jointsUsedInTask = jacobianCalculator.getJointsFromBaseToEndEffector();

      // Step 2: The small Jacobian matrix into the full Jacobian matrix. Proper indexing has to be ensured, so it is handled by the jointIndexHandler.
      jointIndexHandler.compactBlockToFullBlockIgnoreUnindexedJoints(jointsUsedInTask, tempTaskJacobian, qpInputToPack.taskJacobian);

      if (primaryBase == null)
      { // No primary base provided for this task.
        // Record the resulting Jacobian matrix for the privileged configuration.
         recordTaskJacobian(qpInputToPack.taskJacobian);
         // We're done!
      }
      else
      { // A primary base has been provided, two things are happening here:
        // 1- A weight is applied on the joints between the base and the primary base with objective to reduce their involvement for this task.
        // 2- The Jacobian is transformed before being recorded such that the privileged configuration is only applied from the primary base to the end-effector.
         tempPrimaryTaskJacobian.set(qpInputToPack.taskJacobian);

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
                  MatrixTools.scaleColumn(scaleFactor, dofIndex, qpInputToPack.taskJacobian);
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
    * The idea is to convert the information held in the {@code commandToConvert} such that it ends up
    * being formulated as follows:<br>
    * J<sub>MxN</sub> * v<sub>Nx1</sub> = p<sub>Mx1</sub> <br>
    * where J is the M-by-N Jacobian matrix, v is the N-by-1 desired joint velocity vector that the QP
    * solver is solving for, and p is the M-by-1 objective vector. M is called the task size and N is
    * the overall number of degrees of freedom (DoFs) to be controlled.
    * </p>
    * 
    * @return true if the command was successfully converted.
    */
   public boolean convertSpatialVelocityCommand(SpatialVelocityCommand commandToConvert, QPInput qpInputToPack)
   {
      // Gets the M-by-6 selection matrix S.
      commandToConvert.getControlFrame(controlFrame);
      commandToConvert.getSelectionMatrix(controlFrame, tempSelectionMatrix);
      int taskSize = tempSelectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      ConstraintType constraintType = commandToConvert.getConstraintType();
      qpInputToPack.reshape(taskSize);
      qpInputToPack.setConstraintType(constraintType);
      // If the task is setup as a hard constraint, there is no need for a weight matrix.
      if (constraintType == ConstraintType.OBJECTIVE)
      {
         // Compute the M-by-M weight matrix W computed as follows: W = S * W * S^T
         qpInputToPack.setUseWeightScalar(false);
         tempTaskWeight.reshape(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);
         tempTaskWeightSubspace.reshape(taskSize, SpatialAcceleration.SIZE);
         commandToConvert.getWeightMatrix(controlFrame, tempTaskWeight);
         CommonOps.mult(tempSelectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
         CommonOps.multTransB(tempTaskWeightSubspace, tempSelectionMatrix, qpInputToPack.taskWeightMatrix);
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
      CommonOps.mult(tempSelectionMatrix, tempTaskObjective, qpInputToPack.taskObjective);

      // Compute the M-by-N task Jacobian: J = S * J
      // Step 1, let's get the 'small' Jacobian matrix j.
      // It is called small as its number of columns is equal to the number of DoFs to its kinematic chain which is way smaller than the number of robot DoFs.
      tempTaskJacobian.reshape(taskSize, jacobianCalculator.getNumberOfDegreesOfFreedom());
      CommonOps.mult(tempSelectionMatrix, jacobianCalculator.getJacobianMatrix(), tempTaskJacobian);

      // Dealing with the primary base:
      RigidBodyBasics primaryBase = commandToConvert.getPrimaryBase();
      List<JointReadOnly> jointsUsedInTask = jacobianCalculator.getJointsFromBaseToEndEffector();

      // Step 2: The small Jacobian matrix into the full Jacobian matrix. Proper indexing has to be ensured, so it is handled by the jointIndexHandler.
      jointIndexHandler.compactBlockToFullBlockIgnoreUnindexedJoints(jointsUsedInTask, tempTaskJacobian, qpInputToPack.taskJacobian);

      /*
       * The following is relevant only for objective inputs: i- privileged cannot affect QP inputs that
       * are constraints, ii- constraints have no weight.
       */
      if (constraintType == ConstraintType.OBJECTIVE)
      {
         if (primaryBase == null)
         { // No primary base provided for this task.
           // Record the resulting Jacobian matrix for the privileged configuration.
            recordTaskJacobian(qpInputToPack.taskJacobian);
            // We're done!
         }
         else
         { // A primary base has been provided, two things are happening here:
           // 1- A weight is applied on the joints between the base and the primary base with objective to reduce their involvement for this task.
           // 2- The Jacobian is transformed before being recorded such that the privileged configuration is only applied from the primary base to the end-effector.
            tempPrimaryTaskJacobian.set(qpInputToPack.taskJacobian);

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
                     MatrixTools.scaleColumn(scaleFactor, dofIndex, qpInputToPack.taskJacobian);
                     // Zero out the task Jacobian at the joint's column(s) so it is removed from the nullspace calculation for applying the privileged configuration.
                     MatrixTools.zeroColumn(dofIndex, tempPrimaryTaskJacobian);
                  }
               }
            }

            // Record the resulting Jacobian matrix which only zeros before the primary base for the privileged configuration.
            recordTaskJacobian(tempPrimaryTaskJacobian);
            // We're done!
         }
      }

      return true;
   }

   /**
    * Converts a {@link MomentumRateCommand} into a {@link QPInput}.
    * 
    * @return true if the command was successfully converted.
    */
   public boolean convertMomentumRateCommand(MomentumRateCommand commandToConvert, QPInput qpInputToPack)
   {
      commandToConvert.getSelectionMatrix(centerOfMassFrame, tempSelectionMatrix);
      int taskSize = tempSelectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      qpInputToPack.reshape(taskSize);
      qpInputToPack.setUseWeightScalar(false);
      qpInputToPack.setConstraintType(ConstraintType.OBJECTIVE);

      // Compute the weight: W = S * W * S^T
      tempTaskWeight.reshape(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);
      tempTaskWeightSubspace.reshape(taskSize, SpatialAcceleration.SIZE);
      commandToConvert.getWeightMatrix(tempTaskWeight);
      CommonOps.mult(tempSelectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
      CommonOps.multTransB(tempTaskWeightSubspace, tempSelectionMatrix, qpInputToPack.taskWeightMatrix);

      // Compute the task Jacobian: J = S * A
      DenseMatrix64F centroidalMomentumMatrix = getCentroidalMomentumMatrix();
      CommonOps.mult(tempSelectionMatrix, centroidalMomentumMatrix, qpInputToPack.taskJacobian);

      commandToConvert.getMomentumRate(angularMomentum, linearMomentum);
      angularMomentum.changeFrame(centerOfMassFrame);
      linearMomentum.changeFrame(centerOfMassFrame);
      angularMomentum.get(0, tempTaskObjective);
      linearMomentum.get(3, tempTaskObjective);
      DenseMatrix64F convectiveTerm = centroidalMomentumRateCalculator.getBiasSpatialForceMatrix();

      // Compute the task objective: p = S * ( hDot - ADot qDot )
      CommonOps.subtractEquals(tempTaskObjective, convectiveTerm);
      CommonOps.mult(tempSelectionMatrix, tempTaskObjective, qpInputToPack.taskObjective);

      recordTaskJacobian(qpInputToPack.taskJacobian);

      return true;
   }

   /**
    * Converts a {@link MomentumCommand} into a {@link QPInput}.
    * 
    * @return true if the command was successfully converted.
    */
   public boolean convertMomentumCommand(MomentumCommand commandToConvert, QPInput qpInputToPack)
   {
      commandToConvert.getSelectionMatrix(centerOfMassFrame, tempSelectionMatrix);
      int taskSize = tempSelectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      ConstraintType constraintType = ConstraintType.OBJECTIVE;
      qpInputToPack.reshape(taskSize);
      qpInputToPack.setConstraintType(constraintType);

      if (constraintType == ConstraintType.OBJECTIVE)
      {
         // Compute the weight: W = S * W * S^T
         qpInputToPack.setUseWeightScalar(false);
         tempTaskWeight.reshape(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);
         tempTaskWeightSubspace.reshape(taskSize, SpatialAcceleration.SIZE);
         commandToConvert.getWeightMatrix(tempTaskWeight);
         CommonOps.mult(tempSelectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
         CommonOps.multTransB(tempTaskWeightSubspace, tempSelectionMatrix, qpInputToPack.taskWeightMatrix);
      }

      // Compute the task Jacobian: J = S * A
      DenseMatrix64F centroidalMomentumMatrix = getCentroidalMomentumMatrix();
      CommonOps.mult(tempSelectionMatrix, centroidalMomentumMatrix, qpInputToPack.taskJacobian);

      commandToConvert.getMomentumRate(angularMomentum, linearMomentum);
      angularMomentum.changeFrame(centerOfMassFrame);
      linearMomentum.changeFrame(centerOfMassFrame);
      angularMomentum.get(0, tempTaskObjective);
      linearMomentum.get(3, tempTaskObjective);

      // Compute the task objective: p = S * h
      CommonOps.mult(tempSelectionMatrix, tempTaskObjective, qpInputToPack.taskObjective);

      recordTaskJacobian(qpInputToPack.taskJacobian);

      return true;
   }

   /**
    * Converts a {@link LinearMomentumConvexConstraint2DCommand} into a {@link QPInput} intended to be
    * consumed by {@link InverseKinematicsQPSolver}.
    * <p>
    * The resulting output is an {@link ConstraintType#LEQ_INEQUALITY} constraining the x and y
    * components of the linear momentum to remain on the right side of a set of 2D lines defined by
    * each pair of consecutive vertices given by
    * {@link LinearMomentumConvexConstraint2DCommand#getLinearMomentumConstraintVertices()}.
    * </p>
    * 
    * @param command       the command to be converted. Not modified.
    * @param qpInputToPack the result of the conversion. Modified.
    * @return {@code true} if the command was successfully converted, {@code false} otherwise.
    */
   public boolean convertLinearMomentumConvexConstraint2DCommand(LinearMomentumConvexConstraint2DCommand command, QPInput qpInputToPack)
   {
      List<Vector2D> vertices = command.getLinearMomentumConstraintVertices();

      if (vertices.isEmpty())
         return false;

      tempSelectionMatrix.reshape(2, 6);
      tempSelectionMatrix.zero();
      tempSelectionMatrix.set(0, 3, 1.0);
      tempSelectionMatrix.set(1, 4, 1.0);

      // Compute the task Jacobian: J = S * A
      tempTaskJacobian.reshape(2, numberOfDoFs);
      DenseMatrix64F centroidalMomentumMatrix = getCentroidalMomentumMatrix();
      CommonOps.mult(tempSelectionMatrix, centroidalMomentumMatrix, tempTaskJacobian);

      if (vertices.size() < 2)
      {
         return false;
      }
      else if (vertices.size() == 2)
      {
         qpInputToPack.reshape(1);
         qpInputToPack.setConstraintType(ConstraintType.LEQ_INEQUALITY);
         setupLineConstraint(0, vertices.get(0), vertices.get(1), tempTaskJacobian, qpInputToPack);
      }
      else
      {
         qpInputToPack.reshape(vertices.size());
         qpInputToPack.setConstraintType(ConstraintType.LEQ_INEQUALITY);

         Vector2D v0 = vertices.get(vertices.size() - 1);

         for (int i = 0; i < vertices.size(); i++)
         {
            Vector2D v1 = vertices.get(i);
            setupLineConstraint(i, v0, v1, tempTaskJacobian, qpInputToPack);
            v0 = v1;
         }
      }

      return true;
   }

   /**
    * Sets up the {@link QPInput#taskJacobian} and {@link QPInput#taskObjective} to formulate a
    * constraint with respect to 2D line going through {@code firstPointOnLine} and
    * {@code secondPointOnLine}.
    * <p>
    * This method can be used for three applications:
    * <ul>
    * <li>If {@link ConstraintType#LEQ_INEQUALITY}: constrains the momentum to remain of the right side
    * of the line.
    * <li>If {@link ConstraintType#GEQ_INEQUALITY}: constrains the momentum to remain of the left side
    * of the line.
    * <li>If {@link ConstraintType#EQUALITY}: constrains the momentum to remain on the line.
    * </ul>
    * It is up to the caller of this method to set the type of constraint of the given
    * {@code qoInputToPack}.
    * </p>
    * 
    * @param constraintIndex                  the index in {@code qpInputToPack} where to put the new
    *                                         constraint.
    * @param firstPointOnLine                 the first point the line is going through. Not modified.
    * @param secondPointOnLine                the second point the line is going through. Not modified.
    * @param centroidalMomemtumMatrixLinearXY the x and y components of the linear part of the
    *                                         centroidal momentum matrix. Not modified.
    * @param qpInputToPack                    the result is stored in
    *                                         {@code qpInputToPack.taskJacobian} and
    *                                         {@code qpInputToPack.taskObjective}. Modified.
    */
   private void setupLineConstraint(int constraintIndex, Tuple2DReadOnly firstPointOnLine, Tuple2DReadOnly secondPointOnLine,
                                    DenseMatrix64F centroidalMomemtumMatrixLinearXY, QPInput qpInputToPack)
   {
      double directionX = secondPointOnLine.getX() - firstPointOnLine.getX();
      double directionY = secondPointOnLine.getY() - firstPointOnLine.getY();
      double norm = Math.sqrt(EuclidCoreTools.normSquared(directionX, directionY));
      directionX /= norm;
      directionY /= norm;

      lineConstraintSelection.set(0, -directionY);
      lineConstraintSelection.set(1, directionX);
      lineConstraintJacobian.reshape(1, numberOfDoFs);
      CommonOps.mult(lineConstraintSelection, centroidalMomemtumMatrixLinearXY, lineConstraintJacobian);
      CommonOps.insert(lineConstraintJacobian, qpInputToPack.taskJacobian, constraintIndex, 0);

      qpInputToPack.taskObjective.set(constraintIndex, directionX * firstPointOnLine.getY() - directionY * firstPointOnLine.getX());
   }

   /**
    * Converts a {@link JointspaceAccelerationCommand} into a {@link QPInput}.
    * 
    * @return true if the command was successfully converted.
    */
   public boolean convertJointspaceAccelerationCommand(JointspaceAccelerationCommand commandToConvert, QPInput qpInputToPack)
   {
      int taskSize = MultiBodySystemTools.computeDegreesOfFreedom(commandToConvert.getJoints());

      if (taskSize == 0)
         return false;

      qpInputToPack.reshape(taskSize);
      qpInputToPack.setConstraintType(commandToConvert.isHardConstraint() ? ConstraintType.EQUALITY : ConstraintType.OBJECTIVE);
      qpInputToPack.taskJacobian.zero();
      qpInputToPack.taskWeightMatrix.zero();
      qpInputToPack.setUseWeightScalar(false);

      int row = 0;
      for (int jointIndex = 0; jointIndex < commandToConvert.getNumberOfJoints(); jointIndex++)
      {
         JointBasics joint = commandToConvert.getJoint(jointIndex);
         double weight = commandToConvert.getWeight(jointIndex);
         int[] columns = jointIndexHandler.getJointIndices(joint);
         if (columns == null)
            return false;

         CommonOps.insert(commandToConvert.getDesiredAcceleration(jointIndex), qpInputToPack.taskObjective, row, 0);
         for (int column : columns)
         {
            qpInputToPack.taskJacobian.set(row, column, 1.0);
            qpInputToPack.taskWeightMatrix.set(row, row, weight);
            row++;
         }
      }

      recordTaskJacobian(qpInputToPack.taskJacobian);
      return true;
   }

   /**
    * Converts a {@link JointspaceVelocityCommand} into a {@link QPInput}.
    * 
    * @return true if the command was successfully converted.
    */
   public boolean convertJointspaceVelocityCommand(JointspaceVelocityCommand commandToConvert, QPInput qpInputToPack)
   {
      int taskSize = MultiBodySystemTools.computeDegreesOfFreedom(commandToConvert.getJoints());

      if (taskSize == 0)
         return false;

      qpInputToPack.reshape(taskSize);
      qpInputToPack.setConstraintType(commandToConvert.isHardConstraint() ? ConstraintType.EQUALITY : ConstraintType.OBJECTIVE);
      qpInputToPack.taskJacobian.zero();
      qpInputToPack.taskWeightMatrix.zero();
      qpInputToPack.setUseWeightScalar(false);

      int row = 0;
      for (int jointIndex = 0; jointIndex < commandToConvert.getNumberOfJoints(); jointIndex++)
      {
         JointBasics joint = commandToConvert.getJoint(jointIndex);
         double weight = commandToConvert.getWeight(jointIndex);
         int[] columns = jointIndexHandler.getJointIndices(joint);
         if (columns == null)
            return false;

         CommonOps.insert(commandToConvert.getDesiredVelocity(jointIndex), qpInputToPack.taskObjective, row, 0);
         for (int column : columns)
         {
            qpInputToPack.taskJacobian.set(row, column, 1.0);
            qpInputToPack.taskWeightMatrix.set(row, row, weight);
            row++;
         }
      }

      recordTaskJacobian(qpInputToPack.taskJacobian);
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
      if (centroidalMomentumCalculator != null)
         return centroidalMomentumCalculator.getCentroidalMomentumMatrix();
      else
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
      if (centroidalMomentumCalculator != null)
         centroidalMomentumCalculator.getMomentum(jointVelocities, momentum);
      else
         centroidalMomentumRateCalculator.getMomentum(jointVelocities, momentum);
      return momentum;
   }
}
