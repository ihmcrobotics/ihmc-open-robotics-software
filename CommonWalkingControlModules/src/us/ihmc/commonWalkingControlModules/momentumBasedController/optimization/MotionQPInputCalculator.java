package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
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
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
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
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable nullspaceProjectionAlpha = new DoubleYoVariable("nullspaceProjectionAlpha", registry);
   private final DoubleYoVariable secondaryTaskJointsWeight = new DoubleYoVariable("secondaryTaskJointsWeight", registry);

   private final GeometricJacobianHolder geometricJacobianHolder;

   private final PointJacobian pointJacobian = new PointJacobian();
   private final PointJacobianConvectiveTermCalculator pointJacobianConvectiveTermCalculator;

   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final OneDoFJoint[] oneDoFJoints;

   private final FramePoint tempBodyFixedPoint = new FramePoint();
   private final FrameVector pPointVelocity = new FrameVector();
   private final DenseMatrix64F tempPPointMatrixVelocity = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F convectiveTermMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);

   private final SpatialAccelerationVector convectiveTerm = new SpatialAccelerationVector();
   private final ConvectiveTermCalculator convectiveTermCalculator = new ConvectiveTermCalculator();

   private final CentroidalMomentumHandler centroidalMomentumHandler;

   private final JointPrivilegedConfigurationHandler privilegedConfigurationHandler;

   private final DenseMatrix64F tempPrimaryTaskJacobian = new DenseMatrix64F(SpatialMotionVector.SIZE, 12);
   private final DenseMatrix64F tempFullPrimaryTaskJacobian = new DenseMatrix64F(SpatialMotionVector.SIZE, 12);

   private final DenseMatrix64F tempTaskJacobian = new DenseMatrix64F(SpatialMotionVector.SIZE, 12);
   private final DenseMatrix64F tempTaskObjective = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
   private final DenseMatrix64F tempTaskAlphaTaskPriority = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
   private final DenseMatrix64F tempTaskWeight = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F tempTaskWeightSubspace = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);

   private final JointIndexHandler jointIndexHandler;

   private final DenseMatrix64F allTaskJacobian;
   private final DampedLeastSquaresNullspaceCalculator nullspaceCalculator;

   private final int numberOfDoFs;

   public MotionQPInputCalculator(ReferenceFrame centerOfMassFrame, GeometricJacobianHolder geometricJacobianHolder, TwistCalculator twistCalculator,
         JointIndexHandler jointIndexHandler, JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters, YoVariableRegistry parentRegistry)
   {
      this.geometricJacobianHolder = geometricJacobianHolder;
      this.jointIndexHandler = jointIndexHandler;
      this.jointsToOptimizeFor = jointIndexHandler.getIndexedJoints();
      oneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      pointJacobianConvectiveTermCalculator = new PointJacobianConvectiveTermCalculator(twistCalculator);
      centroidalMomentumHandler = new CentroidalMomentumHandler(twistCalculator.getRootBody(), centerOfMassFrame, registry);

      if (jointPrivilegedConfigurationParameters != null)
         privilegedConfigurationHandler = new JointPrivilegedConfigurationHandler(oneDoFJoints, jointPrivilegedConfigurationParameters, registry);
      else
         privilegedConfigurationHandler = null;

      numberOfDoFs = jointIndexHandler.getNumberOfDoFs();
      allTaskJacobian = new DenseMatrix64F(numberOfDoFs, numberOfDoFs);
      secondaryTaskJointsWeight.set(1.0); // TODO Needs to be rethought, it doesn't seem to be that useful.
      nullspaceProjectionAlpha.set(0.005);
      nullspaceCalculator = new DampedLeastSquaresNullspaceCalculator(numberOfDoFs, nullspaceProjectionAlpha.getDoubleValue());

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

   public boolean computePrivilegedJointAccelerations(MotionQPInput motionQPInputToPack)
   {
      if (privilegedConfigurationHandler == null || !privilegedConfigurationHandler.isEnabled())
         return false;

      privilegedConfigurationHandler.computePrivilegedJointAccelerations();

      motionQPInputToPack.setIsMotionConstraint(false);
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

      motionQPInputToPack.setIsMotionConstraint(false);
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
         // Compute the weight: W = S * W * S^T
         motionQPInputToPack.setUseWeightScalar(false);
         tempTaskWeight.reshape(3, 3);
         commandToConvert.getWeightMatrix(tempTaskWeight);
         tempTaskWeightSubspace.reshape(taskSize, 3);
         CommonOps.mult(selectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
         CommonOps.multTransB(tempTaskWeightSubspace, selectionMatrix, motionQPInputToPack.taskWeightMatrix);
      }

      RigidBody base = commandToConvert.getBase();
      RigidBody endEffector = commandToConvert.getEndEffector();
      long jacobianId = geometricJacobianHolder.getOrCreateGeometricJacobian(base, endEffector, base.getBodyFixedFrame());
      GeometricJacobian jacobian = geometricJacobianHolder.getJacobian(jacobianId);

      commandToConvert.getBodyFixedPointIncludingFrame(tempBodyFixedPoint);
      FrameVector desiredAccelerationWithRespectToBase = commandToConvert.getDesiredAcceleration();

      pointJacobian.set(jacobian, tempBodyFixedPoint);
      pointJacobian.compute();

      desiredAccelerationWithRespectToBase.changeFrame(jacobian.getBaseFrame());

      DenseMatrix64F pointJacobianMatrix = pointJacobian.getJacobianMatrix();

      boolean success = true;
      tempTaskJacobian.reshape(selectionMatrix.getNumRows(), pointJacobianMatrix.getNumCols());
      CommonOps.mult(selectionMatrix, pointJacobianMatrix, tempTaskJacobian);

      RigidBody primaryBase = commandToConvert.getPrimaryBase();
      InverseDynamicsJoint[] jointsUsedInTask = jacobian.getJointsInOrder();
      if (primaryBase != null)
      {
         tempPrimaryTaskJacobian.reshape(taskSize, tempTaskJacobian.getNumCols());
         tempPrimaryTaskJacobian.set(tempTaskJacobian);

         boolean isJointUpstreamOfPrimaryBase = false;

         for (int i = jointsUsedInTask.length - 1; i >= 0; i--)
         {
            InverseDynamicsJoint joint = jointsUsedInTask[i];

            if (joint.getSuccessor() == primaryBase)
               isJointUpstreamOfPrimaryBase = true;

            if (isJointUpstreamOfPrimaryBase)
            {
               tempJointIndices.reset();
               ScrewTools.computeIndexForJoint(jointsUsedInTask, tempJointIndices, joint);
               for (int upstreamJointIndex = 0; upstreamJointIndex < tempJointIndices.size(); upstreamJointIndex++)
               {
                  MatrixTools.scaleColumn(secondaryTaskJointsWeight.getDoubleValue(), tempJointIndices.get(upstreamJointIndex), tempTaskJacobian);
                  MatrixTools.zeroColumn(tempJointIndices.get(upstreamJointIndex), tempPrimaryTaskJacobian);
               }
            }
         }

         tempFullPrimaryTaskJacobian.reshape(taskSize, numberOfDoFs);
         success = jointIndexHandler.compactBlockToFullBlock(jointsUsedInTask, tempPrimaryTaskJacobian, tempFullPrimaryTaskJacobian);
      }

      success = success && jointIndexHandler.compactBlockToFullBlock(jointsUsedInTask, tempTaskJacobian, motionQPInputToPack.taskJacobian);

      if (!success)
         return false;

      if (commandToConvert.getPrimaryBase() != null)
         recordTaskJacobian(tempFullPrimaryTaskJacobian);
      else
         recordTaskJacobian(motionQPInputToPack.taskJacobian);

      // Compute the task objective: p = S * ( TDot - JDot qDot )
      pointJacobianConvectiveTermCalculator.compute(pointJacobian, pPointVelocity);
      pPointVelocity.scale(-1.0);
      pPointVelocity.add(desiredAccelerationWithRespectToBase);
      pPointVelocity.getVector().get(tempPPointMatrixVelocity);
      CommonOps.mult(selectionMatrix, tempPPointMatrixVelocity, motionQPInputToPack.taskObjective);

      return true;
   }

   private final TIntArrayList tempJointIndices = new TIntArrayList();

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
         // Compute the weight: W = S * W * S^T
         motionQPInputToPack.setUseWeightScalar(false);
         tempTaskWeight.reshape(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
         commandToConvert.getWeightMatrix(tempTaskWeight);
         tempTaskWeightSubspace.reshape(taskSize, SpatialAccelerationVector.SIZE);
         CommonOps.mult(selectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
         CommonOps.multTransB(tempTaskWeightSubspace, selectionMatrix, motionQPInputToPack.taskWeightMatrix);
      }

      SpatialAccelerationVector spatialAcceleration = commandToConvert.getSpatialAcceleration();
      RigidBody base = commandToConvert.getBase();
      RigidBody endEffector = commandToConvert.getEndEffector();
      long jacobianId = geometricJacobianHolder.getOrCreateGeometricJacobian(base, endEffector, spatialAcceleration.getExpressedInFrame());
      GeometricJacobian jacobian = geometricJacobianHolder.getJacobian(jacobianId);

      // Compute the task Jacobian: J = S * J
      tempTaskJacobian.reshape(taskSize, jacobian.getNumberOfColumns());
      CommonOps.mult(selectionMatrix, jacobian.getJacobianMatrix(), tempTaskJacobian);

      RigidBody primaryBase = commandToConvert.getPrimaryBase();
      InverseDynamicsJoint[] jointsUsedInTask = jacobian.getJointsInOrder();
      if (primaryBase != null)
      {
         tempPrimaryTaskJacobian.reshape(taskSize, jointsUsedInTask.length);
         tempPrimaryTaskJacobian.set(tempTaskJacobian);

         boolean isJointUpstreamOfPrimaryBase = false;
         for (int i = jointsUsedInTask.length - 1; i >= 0; i--)
         {
            InverseDynamicsJoint joint = jointsUsedInTask[i];

            if (joint.getSuccessor() == primaryBase)
               isJointUpstreamOfPrimaryBase = true;

            if (isJointUpstreamOfPrimaryBase)
            {
               tempJointIndices.reset();
               ScrewTools.computeIndexForJoint(jointsUsedInTask, tempJointIndices, joint);
               for (int upstreamJointIndex = 0; upstreamJointIndex < tempJointIndices.size(); upstreamJointIndex++)
               {
                  MatrixTools.scaleColumn(secondaryTaskJointsWeight.getDoubleValue(), tempJointIndices.get(upstreamJointIndex), tempTaskJacobian);
                  MatrixTools.zeroColumn(tempJointIndices.get(upstreamJointIndex), tempPrimaryTaskJacobian);
               }
            }
         }

         tempFullPrimaryTaskJacobian.reshape(taskSize, numberOfDoFs);
         jointIndexHandler.compactBlockToFullBlockIgnoreUnindexedJoints(jointsUsedInTask, tempPrimaryTaskJacobian, tempFullPrimaryTaskJacobian);
      }

      jointIndexHandler.compactBlockToFullBlockIgnoreUnindexedJoints(jointsUsedInTask, tempTaskJacobian, motionQPInputToPack.taskJacobian);

      // Compute the task objective: p = S * ( TDot - JDot qDot )
      convectiveTermCalculator.computeJacobianDerivativeTerm(jacobian, convectiveTerm);
      convectiveTerm.getMatrix(convectiveTermMatrix, 0);
      spatialAcceleration.getMatrix(tempTaskObjective, 0);
      CommonOps.subtractEquals(tempTaskObjective, convectiveTermMatrix);
      CommonOps.mult(selectionMatrix, tempTaskObjective, motionQPInputToPack.taskObjective);

      if (primaryBase != null)
         recordTaskJacobian(tempFullPrimaryTaskJacobian);
      else
         recordTaskJacobian(motionQPInputToPack.taskJacobian);

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
      motionQPInputToPack.setIsMotionConstraint(!commandToConvert.getHasWeight());
      if (commandToConvert.getHasWeight())
      {
         // Compute the weight: W = S * W * S^T
         motionQPInputToPack.setUseWeightScalar(false);
         tempTaskWeight.reshape(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
         commandToConvert.getWeightMatrix(tempTaskWeight);
         tempTaskWeightSubspace.reshape(taskSize, SpatialAccelerationVector.SIZE);
         CommonOps.mult(selectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
         CommonOps.multTransB(tempTaskWeightSubspace, selectionMatrix, motionQPInputToPack.taskWeightMatrix);
      }

      Twist spatialVelocity = commandToConvert.getSpatialVelocity();
      RigidBody base = commandToConvert.getBase();
      RigidBody endEffector = commandToConvert.getEndEffector();
      long jacobianId = geometricJacobianHolder.getOrCreateGeometricJacobian(base, endEffector, spatialVelocity.getExpressedInFrame());
      GeometricJacobian jacobian = geometricJacobianHolder.getJacobian(jacobianId);

      // Compute the task Jacobian: J = S * J
      tempTaskJacobian.reshape(taskSize, jacobian.getNumberOfColumns());
      CommonOps.mult(selectionMatrix, jacobian.getJacobianMatrix(), tempTaskJacobian);

      RigidBody primaryBase = commandToConvert.getPrimaryBase();
      InverseDynamicsJoint[] jointsUsedInTask = jacobian.getJointsInOrder();
      if (primaryBase != null)
      {
         tempPrimaryTaskJacobian.reshape(taskSize, jointsUsedInTask.length);
         tempPrimaryTaskJacobian.set(tempTaskJacobian);

         boolean isJointUpstreamOfPrimaryBase = false;
         for (int i = jointsUsedInTask.length - 1; i >= 0; i--)
         {
            InverseDynamicsJoint joint = jointsUsedInTask[i];

            if (joint.getSuccessor() == primaryBase)
               isJointUpstreamOfPrimaryBase = true;

            if (isJointUpstreamOfPrimaryBase)
            {
               tempJointIndices.reset();
               ScrewTools.computeIndexForJoint(jointsUsedInTask, tempJointIndices, joint);
               for (int upstreamJointIndex = 0; upstreamJointIndex < tempJointIndices.size(); upstreamJointIndex++)
               {
                  MatrixTools.scaleColumn(secondaryTaskJointsWeight.getDoubleValue(), tempJointIndices.get(upstreamJointIndex), tempTaskJacobian);
                  MatrixTools.zeroColumn(tempJointIndices.get(upstreamJointIndex), tempPrimaryTaskJacobian);
               }
            }
         }

         tempFullPrimaryTaskJacobian.reshape(taskSize, numberOfDoFs);
         jointIndexHandler.compactBlockToFullBlockIgnoreUnindexedJoints(jointsUsedInTask, tempPrimaryTaskJacobian, tempFullPrimaryTaskJacobian);
      }

      jointIndexHandler.compactBlockToFullBlockIgnoreUnindexedJoints(jointsUsedInTask, tempTaskJacobian, motionQPInputToPack.taskJacobian);

      // Compute the task objective: p = S * T
      spatialVelocity.getMatrix(tempTaskObjective, 0);
      CommonOps.mult(selectionMatrix, tempTaskObjective, motionQPInputToPack.taskObjective);

      if (primaryBase != null)
         recordTaskJacobian(tempFullPrimaryTaskJacobian);
      else
         recordTaskJacobian(motionQPInputToPack.taskJacobian);

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

      tempTaskAlphaTaskPriority.reshape(taskSize, 1);
      CommonOps.mult(selectionMatrix, commandToConvert.getAlphaTaskPriorityVector(), tempTaskAlphaTaskPriority);

      for (int i = taskSize - 1; i >= 0; i--)
      {
         double alpha = tempTaskAlphaTaskPriority.get(i, 0);
         MatrixTools.scaleRow(alpha, i, motionQPInputToPack.taskJacobian);
         MatrixTools.scaleRow(alpha, i, motionQPInputToPack.taskObjective);
      }

      recordTaskJacobian(motionQPInputToPack.taskJacobian);

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

      recordTaskJacobian(motionQPInputToPack.taskJacobian);

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
      motionQPInputToPack.setIsMotionConstraint(commandToConvert.isHardConstraint());
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
    * @return true if the command was successfully converted.
    */
   public boolean convertJointspaceVelocityCommand(JointspaceVelocityCommand commandToConvert, MotionQPInput motionQPInputToPack)
   {
      int taskSize = ScrewTools.computeDegreesOfFreedom(commandToConvert.getJoints());

      if (taskSize == 0)
         return false;

      motionQPInputToPack.reshape(taskSize);
      motionQPInputToPack.setIsMotionConstraint(commandToConvert.isHardConstraint());
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
      allTaskJacobian.reshape(allTaskJacobian.getNumRows() + taskSize, numberOfDoFs);
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
