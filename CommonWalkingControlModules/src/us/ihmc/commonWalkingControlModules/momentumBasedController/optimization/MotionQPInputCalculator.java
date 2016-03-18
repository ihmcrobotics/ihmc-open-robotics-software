package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointspaceVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.inverseKinematics.JointPrivilegedConfigurationHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.ConvectiveTermCalculator;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.PointJacobian;
import us.ihmc.robotics.screwTheory.PointJacobianConvectiveTermCalculator;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class MotionQPInputCalculator
{
   private final GeometricJacobianHolder geometricJacobianHolder;

   private final PointJacobian pointJacobian = new PointJacobian();
   private final PointJacobianConvectiveTermCalculator pointJacobianConvectiveTermCalculator;

   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final OneDoFJoint[] oneDoFJoints;

   private final FrameVector pPointVelocity = new FrameVector();
   private final DenseMatrix64F tempPPointMatrixVelocity = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F convectiveTermMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);

   private final SpatialAccelerationVector convectiveTerm = new SpatialAccelerationVector();
   private final ConvectiveTermCalculator convectiveTermCalculator = new ConvectiveTermCalculator();

   private final CentroidalMomentumHandler centroidalMomentumHandler;

   private final JointPrivilegedConfigurationHandler privilegedConfigurationHandler;

   private final DenseMatrix64F tempTaskJacobian = new DenseMatrix64F(SpatialMotionVector.SIZE, 12);
   private final DenseMatrix64F tempTaskObjective = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
   private final DenseMatrix64F tempTaskWeight = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F tempTaskWeightSubspace = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);

   private final double controlDT;

   private final JointIndexHandler jointIndexHandler;

   public MotionQPInputCalculator(ReferenceFrame centerOfMassFrame, GeometricJacobianHolder geometricJacobianHolder, TwistCalculator twistCalculator,
         JointIndexHandler jointIndexHandler, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.geometricJacobianHolder = geometricJacobianHolder;
      this.jointIndexHandler = jointIndexHandler;
      this.jointsToOptimizeFor = jointIndexHandler.getIndexedJoints();
      this.controlDT = controlDT;
      oneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      pointJacobianConvectiveTermCalculator = new PointJacobianConvectiveTermCalculator(twistCalculator);
      centroidalMomentumHandler = new CentroidalMomentumHandler(twistCalculator.getRootBody(), centerOfMassFrame, parentRegistry);
      privilegedConfigurationHandler = new JointPrivilegedConfigurationHandler(oneDoFJoints, parentRegistry);
   }

   public void update()
   {
      centroidalMomentumHandler.compute();
   }

   public void updatePrivilegedConfiguration(PrivilegedConfigurationCommand command)
   {
      privilegedConfigurationHandler.submitPrivilegedConfigurationCommand(command);
   }

   public boolean computePrivilegedJointAccelerations(PrivilegedMotionQPInput privilegedMotionQPInputToPack)
   {
      if (!privilegedConfigurationHandler.isEnabled())
         return false;

      privilegedConfigurationHandler.computePrivilegedJointAccelerations();

      DenseMatrix64F selectionMatrix = privilegedConfigurationHandler.getSelectionMatrix();

      int taskSize = selectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      privilegedMotionQPInputToPack.reshape(taskSize);

      privilegedMotionQPInputToPack.setPrivilegedJointspaceMotion(privilegedConfigurationHandler.getPrivilegedJointAccelerations());
      privilegedMotionQPInputToPack.setWeight(privilegedConfigurationHandler.getWeight());

      OneDoFJoint[] joints = privilegedConfigurationHandler.getJoints();
      boolean success = jointIndexHandler.compactBlockToFullBlock(joints, selectionMatrix, privilegedMotionQPInputToPack.selectionMatrix);

      if (!success)
         return false;

      return true;
   }

   public boolean computePrivilegedJointVelocities(PrivilegedMotionQPInput privilegedMotionQPInputToPack)
   {
      if (!privilegedConfigurationHandler.isEnabled())
         return false;

      privilegedConfigurationHandler.computePrivilegedJointVelocities();

      DenseMatrix64F selectionMatrix = privilegedConfigurationHandler.getSelectionMatrix();

      int taskSize = selectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      privilegedMotionQPInputToPack.reshape(taskSize);

      privilegedMotionQPInputToPack.setPrivilegedJointspaceMotion(privilegedConfigurationHandler.getPrivilegedJointVelocities());
      privilegedMotionQPInputToPack.setWeight(privilegedConfigurationHandler.getWeight());

      OneDoFJoint[] joints = privilegedConfigurationHandler.getJoints();
      boolean success = jointIndexHandler.compactBlockToFullBlock(joints, selectionMatrix, privilegedMotionQPInputToPack.selectionMatrix);

      if (!success)
         return false;

      return true;
   }

   /**
    * Converts a {@link PointAccelerationCommand} into a {@link MotionQPInput}.
    * @return true if the command was successfully converted.
    */
   public boolean convertPointAccelerationCommand(PointAccelerationCommand commandToConvert, MotionQPInput motionQPInputToPack)
   {
      DenseMatrix64F selectionMatrix = commandToConvert.getSelectionMatrix();
      int taskSize = selectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      motionQPInputToPack.reshape(taskSize);
      motionQPInputToPack.setIsMotionConstraint(!commandToConvert.getHasWeight());
      if (commandToConvert.getHasWeight())
      {
         motionQPInputToPack.setUseWeightScalar(true);
         motionQPInputToPack.setWeight(commandToConvert.getWeight());
      }

      RigidBody base = commandToConvert.getBase();
      RigidBody endEffector = commandToConvert.getEndEffector();
      long jacobianId = geometricJacobianHolder.getOrCreateGeometricJacobian(base, endEffector, base.getBodyFixedFrame());
      GeometricJacobian jacobian = geometricJacobianHolder.getJacobian(jacobianId);

      FramePoint bodyFixedPoint = commandToConvert.getBodyFixedPointToControl();
      FrameVector desiredAccelerationWithRespectToBase = commandToConvert.getDesiredAcceleration();

      pointJacobian.set(jacobian, bodyFixedPoint);
      pointJacobian.compute();

      desiredAccelerationWithRespectToBase.changeFrame(jacobian.getBaseFrame());

      DenseMatrix64F pointJacobianMatrix = pointJacobian.getJacobianMatrix();

      tempTaskJacobian.reshape(selectionMatrix.getNumRows(), pointJacobianMatrix.getNumCols());
      CommonOps.mult(selectionMatrix, pointJacobianMatrix, tempTaskJacobian);
      boolean success = jointIndexHandler.compactBlockToFullBlock(jacobian.getJointsInOrder(), tempTaskJacobian, motionQPInputToPack.taskJacobian);

      if (!success)
         return false;

      pointJacobianConvectiveTermCalculator.compute(pointJacobian, pPointVelocity);
      pPointVelocity.scale(-1.0);
      pPointVelocity.add(desiredAccelerationWithRespectToBase);
      MatrixTools.setDenseMatrixFromTuple3d(tempPPointMatrixVelocity, pPointVelocity.getVector(), 0, 0);
      CommonOps.mult(selectionMatrix, tempPPointMatrixVelocity, motionQPInputToPack.taskObjective);

      return true;
   }

   /**
    * Converts a {@link SpatialAccelerationCommand} into a {@link MotionQPInput}.
    * @return true if the command was successfully converted.
    */
   public boolean convertSpatialAccelerationCommand(SpatialAccelerationCommand commandToConvert, MotionQPInput motionQPInputToPack)
   {
      DenseMatrix64F selectionMatrix = commandToConvert.getSelectionMatrix();
      int taskSize = selectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      motionQPInputToPack.reshape(taskSize);
      motionQPInputToPack.setIsMotionConstraint(!commandToConvert.getHasWeight());
      if (commandToConvert.getHasWeight())
      {
         motionQPInputToPack.setUseWeightScalar(true);
         motionQPInputToPack.setWeight(commandToConvert.getWeight());
      }

      SpatialAccelerationVector spatialAcceleration = commandToConvert.getSpatialAcceleration();
      RigidBody base = commandToConvert.getBase();
      RigidBody endEffector = commandToConvert.getEndEffector();
      long jacobianId = geometricJacobianHolder.getOrCreateGeometricJacobian(base, endEffector, spatialAcceleration.getExpressedInFrame());
      GeometricJacobian jacobian = geometricJacobianHolder.getJacobian(jacobianId);

      // Compute the task Jacobian: J = S * J
      tempTaskJacobian.reshape(taskSize, jacobian.getNumberOfColumns());
      CommonOps.mult(selectionMatrix, jacobian.getJacobianMatrix(), tempTaskJacobian);
      boolean success = jointIndexHandler.compactBlockToFullBlock(jacobian.getJointsInOrder(), tempTaskJacobian, motionQPInputToPack.taskJacobian);

      if (!success)
         return false;

      // Compute the task objective: p = S * ( TDot - JDot qDot )
      convectiveTermCalculator.computeJacobianDerivativeTerm(jacobian, convectiveTerm);
      convectiveTerm.getMatrix(convectiveTermMatrix, 0);
      spatialAcceleration.getMatrix(tempTaskObjective, 0);
      CommonOps.subtractEquals(tempTaskObjective, convectiveTermMatrix);
      CommonOps.mult(selectionMatrix, tempTaskObjective, motionQPInputToPack.taskObjective);

      return true;
   }

   /**
    * Converts a {@link SpatialVelocityCommand} into a {@link MotionQPInput}.
    * @return true if the command was successfully converted.
    */
   public boolean convertSpatialVelocityCommand(SpatialVelocityCommand commandToConvert, MotionQPInput motionQPInputToPack)
   {
      DenseMatrix64F selectionMatrix = commandToConvert.getSelectionMatrix();
      int taskSize = selectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      motionQPInputToPack.reshape(taskSize);
      motionQPInputToPack.setIsMotionConstraint(commandToConvert.isHardConstraint());
      if (!commandToConvert.isHardConstraint())
      {
         motionQPInputToPack.setUseWeightScalar(true);
         motionQPInputToPack.setWeight(commandToConvert.getWeight());
      }

      Twist spatialVelocity = commandToConvert.getSpatialVelocity();
      RigidBody base = commandToConvert.getBase();
      RigidBody endEffector = commandToConvert.getEndEffector();
      long jacobianId = geometricJacobianHolder.getOrCreateGeometricJacobian(base, endEffector, spatialVelocity.getExpressedInFrame());
      GeometricJacobian jacobian = geometricJacobianHolder.getJacobian(jacobianId);

      // Compute the task Jacobian: J = S * J
      tempTaskJacobian.reshape(taskSize, jacobian.getNumberOfColumns());
      CommonOps.mult(selectionMatrix, jacobian.getJacobianMatrix(), tempTaskJacobian);
      boolean success = jointIndexHandler.compactBlockToFullBlock(jacobian.getJointsInOrder(), tempTaskJacobian, motionQPInputToPack.taskJacobian);

      if (!success)
         return false;

      // Compute the task objective: p = S * T
      spatialVelocity.getMatrix(tempTaskObjective, 0);
      CommonOps.mult(selectionMatrix, tempTaskObjective, motionQPInputToPack.taskObjective);

      return true;
   }

   /**
    * Converts a {@link MomentumRateCommand} into a {@link MotionQPInput}.
    * @return true if the command was successfully converted.
    */
   public boolean convertMomentumRateCommand(MomentumRateCommand commandToConvert, MotionQPInput motionQPInputToPack)
   {
      DenseMatrix64F selectionMatrix = commandToConvert.getSelectionMatrix();
      int taskSize = selectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      motionQPInputToPack.reshape(taskSize);
      motionQPInputToPack.setUseWeightScalar(false);
      motionQPInputToPack.setIsMotionConstraint(false);

      // Compute the weight: W = S * W * S^T
      tempTaskWeight.reshape(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
      commandToConvert.getWeightMatrix(tempTaskWeight);
      tempTaskWeightSubspace.reshape(taskSize, SpatialAccelerationVector.SIZE);
      CommonOps.mult(selectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
      CommonOps.multTransB(tempTaskWeightSubspace, selectionMatrix, motionQPInputToPack.taskWeightMatrix);

      // Compute the task Jacobian: J = S * A
      DenseMatrix64F centroidalMomentumMatrix = getCentroidalMomentumMatrix();
      CommonOps.mult(selectionMatrix, centroidalMomentumMatrix, motionQPInputToPack.taskJacobian);

      DenseMatrix64F momemtumRate = commandToConvert.getMomentumRate();
      DenseMatrix64F convectiveTerm = centroidalMomentumHandler.getCentroidalMomentumConvectiveTerm();

      // Compute the task objective: p = S * ( hDot - ADot qDot )
      CommonOps.subtract(momemtumRate, convectiveTerm, tempTaskObjective);
      CommonOps.mult(selectionMatrix, tempTaskObjective, motionQPInputToPack.taskObjective);

      return true;
   }

   /**
    * Converts a {@link MomentumCommand} into a {@link MotionQPInput}.
    * @return true if the command was successfully converted.
    */
   public boolean convertMomentumCommand(MomentumCommand commandToConvert, MotionQPInput motionQPInputToPack)
   {
      DenseMatrix64F selectionMatrix = commandToConvert.getSelectionMatrix();
      int taskSize = selectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      motionQPInputToPack.reshape(taskSize);
      motionQPInputToPack.setUseWeightScalar(false);
      motionQPInputToPack.setIsMotionConstraint(false);

      // Compute the weight: W = S * W * S^T
      tempTaskWeight.reshape(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
      commandToConvert.getWeightMatrix(tempTaskWeight);
      tempTaskWeightSubspace.reshape(taskSize, SpatialAccelerationVector.SIZE);
      CommonOps.mult(selectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
      CommonOps.multTransB(tempTaskWeightSubspace, selectionMatrix, motionQPInputToPack.taskWeightMatrix);

      // Compute the task Jacobian: J = S * A
      DenseMatrix64F centroidalMomentumMatrix = getCentroidalMomentumMatrix();
      CommonOps.mult(selectionMatrix, centroidalMomentumMatrix, motionQPInputToPack.taskJacobian);

      DenseMatrix64F momemtum = commandToConvert.getMomentum();

      // Compute the task objective: p = S * h
      CommonOps.mult(selectionMatrix, momemtum, motionQPInputToPack.taskObjective);

      return true;
   }

   /**
    * Converts a {@link JointspaceAccelerationCommand} into a {@link MotionQPInput}.
    * @return true if the command was successfully converted.
    */
   public boolean convertJointspaceAccelerationCommand(JointspaceAccelerationCommand commandToConvert, MotionQPInput motionQPInputToPack)
   {
      int taskSize = ScrewTools.computeDegreesOfFreedom(commandToConvert.getJoints());

      if (taskSize == 0)
         return false;

      motionQPInputToPack.reshape(taskSize);
      motionQPInputToPack.setIsMotionConstraint(!commandToConvert.getHasWeight());
      if (commandToConvert.getHasWeight())
      {
         motionQPInputToPack.setUseWeightScalar(true);
         motionQPInputToPack.setWeight(commandToConvert.getWeight());
      }

      motionQPInputToPack.taskJacobian.zero();

      int row = 0;
      for (int jointIndex = 0; jointIndex < commandToConvert.getNumberOfJoints(); jointIndex++)
      {
         InverseDynamicsJoint joint = commandToConvert.getJoint(jointIndex);
         int[] columns = jointIndexHandler.getJointIndices(joint);
         if (columns == null)
            return false;
         for (int column : columns)
            motionQPInputToPack.taskJacobian.set(row, column, 1.0);

         CommonOps.insert(commandToConvert.getDesiredAcceleration(jointIndex), motionQPInputToPack.taskObjective, row, 0);
         row += joint.getDegreesOfFreedom();
      }

      return true;
   }

   /**
    * Converts a {@link JointspaceVelocityCommand} into a {@link MotionQPInput}.
    * @return true if the command was successfully converted.
    */
   public boolean convertJointspaceVelocityCommand(JointspaceVelocityCommand commandToConvert, MotionQPInput motionQPInputToPack)
   {
      int taskSize = ScrewTools.computeDegreesOfFreedom(commandToConvert.getJoints());

      if (taskSize == 0)
         return false;

      motionQPInputToPack.reshape(taskSize);
      motionQPInputToPack.setIsMotionConstraint(commandToConvert.isHardConstraint());
      if (!commandToConvert.isHardConstraint())
      {
         motionQPInputToPack.setUseWeightScalar(true);
         motionQPInputToPack.setWeight(commandToConvert.getWeight());
      }

      motionQPInputToPack.taskJacobian.zero();

      int row = 0;
      for (int jointIndex = 0; jointIndex < commandToConvert.getNumberOfJoints(); jointIndex++)
      {
         InverseDynamicsJoint joint = commandToConvert.getJoint(jointIndex);
         int[] columns = jointIndexHandler.getJointIndices(joint);
         if (columns == null)
            return false;
         for (int column : columns)
            motionQPInputToPack.taskJacobian.set(row, column, 1.0);

         CommonOps.insert(commandToConvert.getDesiredVelocity(jointIndex), motionQPInputToPack.taskObjective, row, 0);
         row += joint.getDegreesOfFreedom();
      }

      return true;
   }

   public void computeJointAccelerationLimits(DenseMatrix64F qDDotMinToPack, DenseMatrix64F qDDotMaxToPack)
   {
      CommonOps.fill(qDDotMinToPack, Double.NEGATIVE_INFINITY);
      CommonOps.fill(qDDotMaxToPack, Double.POSITIVE_INFINITY);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         int index = jointIndexHandler.getOneDoFJointIndex(joint);
         double jointLimitLower = joint.getJointLimitLower();
         double jointLimitUpper = joint.getJointLimitUpper();

         double qDDotMin = Double.NEGATIVE_INFINITY;
         double qDDotMax = Double.POSITIVE_INFINITY;

         if (!Double.isInfinite(jointLimitLower))
         {
            double qDotMin = (jointLimitLower - joint.getQ()) / controlDT;
            qDDotMin = (qDotMin - joint.getQd()) / controlDT;
            qDDotMin = MathTools.clipToMinMax(qDDotMin, -100.0, 0.0);
            qDDotMinToPack.set(index, 0, qDDotMin);
         }
         if (!Double.isInfinite(jointLimitUpper))
         {
            double qDotMax = (jointLimitUpper - joint.getQ()) / controlDT;
            qDDotMax = (qDotMax - joint.getQd()) / controlDT;
            qDDotMax = MathTools.clipToMinMax(qDDotMax, -0.0, 100.0);
            qDDotMaxToPack.set(index, 0, qDDotMax);
         }
      }
   }

   public void computeJointVelocityLimits(DenseMatrix64F qDotMinToPack, DenseMatrix64F qDotMaxToPack)
   {
      CommonOps.fill(qDotMinToPack, Double.NEGATIVE_INFINITY);
      CommonOps.fill(qDotMaxToPack, Double.POSITIVE_INFINITY);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         int index = jointIndexHandler.getOneDoFJointIndex(joint);
         double jointLimitLower = joint.getJointLimitLower();
         if (!Double.isInfinite(jointLimitLower))
            qDotMinToPack.set(index, 0, (jointLimitLower - joint.getQ()) / controlDT);
         double jointLimitUpper = joint.getJointLimitUpper();
         if (!Double.isInfinite(jointLimitUpper))
            qDotMaxToPack.set(index, 0, (jointLimitUpper - joint.getQ()) / controlDT);
      }
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
