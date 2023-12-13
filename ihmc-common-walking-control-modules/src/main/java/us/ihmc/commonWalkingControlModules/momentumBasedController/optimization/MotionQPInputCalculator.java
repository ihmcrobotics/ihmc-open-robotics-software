package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.MatrixDimensionException;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.*;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.*;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointTorqueCommand;
import us.ihmc.commonWalkingControlModules.inverseKinematics.InverseKinematicsQPSolver;
import us.ihmc.commonWalkingControlModules.inverseKinematics.JointPrivilegedConfigurationHandler;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.matrixlib.NativeNullspaceProjector;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.mecano.algorithms.CentroidalMomentumRateCalculator;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.algorithms.MultiBodyGravityGradientCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.*;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.List;

public class MotionQPInputCalculator
{
   private static final double minWeightForTaskInclusion = 1e-12;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble nullspaceProjectionAlpha;
   private final YoDouble secondaryTaskJointsWeight = new YoDouble("secondaryTaskJointsWeight", registry);

   private final PoseReferenceFrame controlFrame = new PoseReferenceFrame("controlFrame", worldFrame);
   private final GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();

   private final CentroidalMomentumCalculator centroidalMomentumCalculator;
   private final CentroidalMomentumRateCalculator centroidalMomentumRateCalculator;
   private final MultiBodyGravityGradientCalculator gravityGradientCalculator;

   private final JointPrivilegedConfigurationHandler privilegedConfigurationHandler;

   private final DMatrixRMaj tempPrimaryTaskJacobian = new DMatrixRMaj(SpatialVector.SIZE, 12);
   private final NativeMatrix nativeTempPrimaryTaskJacobian = new NativeMatrix(SpatialVector.SIZE, 12);

   private final DMatrixRMaj tempTaskJacobian = new DMatrixRMaj(SpatialVector.SIZE, 12);
   private final NativeMatrix tempTaskJacobianNative = new NativeMatrix(SpatialVector.SIZE, 12);
   private final NativeMatrix tempTaskVelocityJacobianNative = new NativeMatrix(SpatialVector.SIZE, 12);
   private final NativeMatrix projectedTaskJacobian = new NativeMatrix(SpatialVector.SIZE, 12);
   private final DMatrixRMaj tempTaskObjective = new DMatrixRMaj(SpatialVector.SIZE, 1);
   private final NativeMatrix nativeTempTaskObjective = new NativeMatrix(SpatialVector.SIZE, 1);
   private final DMatrixRMaj tempTaskWeight = new DMatrixRMaj(SpatialVector.SIZE, SpatialAcceleration.SIZE);
   private final NativeMatrix nativeTempTaskWeight = new NativeMatrix(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);
   private final DMatrixRMaj tempTaskWeightSubspace = new DMatrixRMaj(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);
   private final NativeMatrix nativeTempTaskWeightSubspace = new NativeMatrix(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);
   private final NativeMatrix nativeCentroidalMomentumMatrix = new NativeMatrix(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);
   private final NativeMatrix nativeBiasSpatialForceMatrix = new NativeMatrix(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);

   private final NativeMatrix nativeTempJacobian = new NativeMatrix(0, 0);
   private final NativeMatrix nativeTempTaskJacobian = new NativeMatrix(0, 0);
   private final NativeMatrix nativeTempJacobianRate = new NativeMatrix(0, 0);
   private final NativeMatrix nativeTempConvectiveTerm = new NativeMatrix(0, 0);

   private final DMatrixRMaj lineConstraintSelection = new DMatrixRMaj(1, 2);
   private final DMatrixRMaj lineConstraintJacobian = new DMatrixRMaj(1, 12);

   private final DMatrixRMaj tempSelectionMatrix = new DMatrixRMaj(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);
   private final NativeMatrix nativeTempSelectionMatrix = new NativeMatrix(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);

   private final FrameVector3D angularMomentum = new FrameVector3D();
   private final FrameVector3D linearMomentum = new FrameVector3D();
   private final ReferenceFrame centerOfMassFrame;

   private final JointIndexHandler jointIndexHandler;

   // TODO get rid of the not native all task jacobian
   private final DMatrixRMaj allTaskJacobian;
   private final NativeMatrix nativeAllTaskJacobian;

   private final int numberOfDoFs;

   private final NativeNullspaceProjector accelerationNativeNullspaceProjector;
   private final NativeNullspaceProjector velocityNativeNullspaceProjector;

   private final YoInteger sizeOfAllTasksJacobian = new YoInteger("sizeOfAllTasksJacobian", registry);

   public MotionQPInputCalculator(ReferenceFrame centerOfMassFrame,
                                  CentroidalMomentumRateCalculator centroidalMomentumRateCalculator,
                                  MultiBodyGravityGradientCalculator gravityGradientCalculator,
                                  JointIndexHandler jointIndexHandler,
                                  JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters,
                                  YoRegistry parentRegistry)
   {
      this(centerOfMassFrame,
           null,
           centroidalMomentumRateCalculator,
           gravityGradientCalculator,
           jointIndexHandler,
           jointPrivilegedConfigurationParameters,
           parentRegistry);
   }

   public MotionQPInputCalculator(ReferenceFrame centerOfMassFrame,
                                  CentroidalMomentumCalculator centroidalMomentumCalculator,
                                  MultiBodyGravityGradientCalculator gravityGradientCalculator,
                                  JointIndexHandler jointIndexHandler,
                                  JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters,
                                  YoRegistry parentRegistry)
   {
      this(centerOfMassFrame,
           centroidalMomentumCalculator,
           null,
           gravityGradientCalculator,
           jointIndexHandler,
           jointPrivilegedConfigurationParameters,
           parentRegistry);
   }

   private MotionQPInputCalculator(ReferenceFrame centerOfMassFrame,
                                   CentroidalMomentumCalculator centroidalMomentumCalculator,
                                   CentroidalMomentumRateCalculator centroidalMomentumRateCalculator,
                                   MultiBodyGravityGradientCalculator gravityGradientCalculator,
                                   JointIndexHandler jointIndexHandler,
                                   JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters,
                                   YoRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      this.gravityGradientCalculator = gravityGradientCalculator;
      this.jointIndexHandler = jointIndexHandler;
      this.centroidalMomentumCalculator = centroidalMomentumCalculator;
      this.centroidalMomentumRateCalculator = centroidalMomentumRateCalculator;

      OneDoFJointBasics[] oneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
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

      allTaskJacobian = new DMatrixRMaj(numberOfDoFs, numberOfDoFs);
      nativeAllTaskJacobian = new NativeMatrix(numberOfDoFs, numberOfDoFs);
      secondaryTaskJointsWeight.set(1.0); // TODO Needs to be rethought, it doesn't seem to be that useful.

      accelerationNativeNullspaceProjector = new NativeNullspaceProjector(numberOfDoFs);
      velocityNativeNullspaceProjector = new NativeNullspaceProjector(numberOfDoFs);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      if (centroidalMomentumRateCalculator != null)
         centroidalMomentumRateCalculator.reset();
      else
         centroidalMomentumCalculator.reset();
      if (gravityGradientCalculator != null)
         gravityGradientCalculator.reset();
      allTaskJacobian.reshape(0, numberOfDoFs);
      nativeAllTaskJacobian.reshape(0, numberOfDoFs);
      sizeOfAllTasksJacobian.set(0);
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

   public boolean computePrivilegedJointAccelerations(NativeQPInputTypeA qpInputToPack)
   {
      if (privilegedConfigurationHandler == null || !privilegedConfigurationHandler.isEnabled())
         return false;

      privilegedConfigurationHandler.computePrivilegedJointAccelerations();

      qpInputToPack.setConstraintType(ConstraintType.OBJECTIVE);
      qpInputToPack.setUseWeightScalar(false);

      int taskSize = 0;

      DMatrixRMaj selectionMatrix = privilegedConfigurationHandler.getSelectionMatrix();
      int robotTaskSize = selectionMatrix.getNumRows();

      if (robotTaskSize > 0)
      {
         OneDoFJointBasics[] joints = privilegedConfigurationHandler.getJoints();
         tempTaskJacobianNative.reshape(robotTaskSize, numberOfDoFs);
         boolean success = jointIndexHandler.compactBlockToFullBlock(joints, selectionMatrix, tempTaskJacobianNative);

         if (success)
         {
            qpInputToPack.reshape(robotTaskSize);
            mergeAllTaskJacobians();
            accelerationNativeNullspaceProjector.project(tempTaskJacobianNative,
                                                         nativeAllTaskJacobian,
                                                         projectedTaskJacobian,
                                                         nullspaceProjectionAlpha.getValue());

            qpInputToPack.taskJacobian.insert(projectedTaskJacobian, taskSize, 0);
            qpInputToPack.taskObjective.insert(privilegedConfigurationHandler.getPrivilegedJointAccelerations(), taskSize, 0);
            qpInputToPack.taskWeightMatrix.insert(privilegedConfigurationHandler.getWeights(), taskSize, taskSize);
         }
      }

      return robotTaskSize > 0;
   }

   public boolean computePrivilegedJointVelocities(QPInputTypeA qpInputToPack)
   {
      if (privilegedConfigurationHandler == null || !privilegedConfigurationHandler.isEnabled())
         return false;

      privilegedConfigurationHandler.computePrivilegedJointVelocities();

      qpInputToPack.setConstraintType(ConstraintType.OBJECTIVE);
      qpInputToPack.setUseWeightScalar(false);

      DMatrixRMaj selectionMatrix = privilegedConfigurationHandler.getSelectionMatrix();

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

      tempTaskVelocityJacobianNative.set(qpInputToPack.taskJacobian);

      mergeAllTaskJacobians();
      velocityNativeNullspaceProjector.project(tempTaskVelocityJacobianNative,
                                               nativeAllTaskJacobian,
                                               projectedTaskJacobian,
                                               nullspaceProjectionAlpha.getValue());

      projectedTaskJacobian.get(qpInputToPack.taskJacobian);

      return true;
   }

   public boolean computeGravityCompensationMinimization(QPInputTypeA qpInputToPack, double weight, boolean projectIntoNullspace, double dt)
   {
      if (weight <= 0.0)
         return false;

      qpInputToPack.reshape(numberOfDoFs);
      CommonOps_DDRM.scale(dt, gravityGradientCalculator.getTauGradientMatrix(), qpInputToPack.taskJacobian);
      qpInputToPack.taskObjective.set(gravityGradientCalculator.getTauMatrix());
      CommonOps_DDRM.changeSign(qpInputToPack.taskObjective);
      qpInputToPack.setUseWeightScalar(true);
      qpInputToPack.setWeight(weight);

      if (projectIntoNullspace)
      {
         tempTaskVelocityJacobianNative.set(qpInputToPack.taskJacobian);
         mergeAllTaskJacobians();
         velocityNativeNullspaceProjector.project(tempTaskVelocityJacobianNative,
                                                  nativeAllTaskJacobian,
                                                  projectedTaskJacobian,
                                                  nullspaceProjectionAlpha.getValue());
         projectedTaskJacobian.get(qpInputToPack.taskJacobian);
      }
      else
      {
         recordTaskJacobian(qpInputToPack.taskJacobian);
      }
      return true;
   }

   public boolean computeGravityCompensationMinimization(QPInputTypeA qpInputToPack,
                                                         JointTorqueMinimizationWeightCalculator weightCalculator,
                                                         boolean projectIntoNullspace,
                                                         double dt)
   {
      if (weightCalculator.isWeightZero())
         return false;

      qpInputToPack.reshape(numberOfDoFs);
      CommonOps_DDRM.scale(dt, gravityGradientCalculator.getTauGradientMatrix(), qpInputToPack.taskJacobian);
      qpInputToPack.taskObjective.set(gravityGradientCalculator.getTauMatrix());
      CommonOps_DDRM.changeSign(qpInputToPack.taskObjective);
      qpInputToPack.taskWeightMatrix.zero();
      weightCalculator.computeWeightMatrix(gravityGradientCalculator.getTauMatrix(), qpInputToPack.taskWeightMatrix);

      if (projectIntoNullspace)
      {
         tempTaskVelocityJacobianNative.set(qpInputToPack.taskJacobian);
         mergeAllTaskJacobians();
         velocityNativeNullspaceProjector.project(tempTaskVelocityJacobianNative,
                                                  nativeAllTaskJacobian,
                                                  projectedTaskJacobian,
                                                  nullspaceProjectionAlpha.getValue());
         projectedTaskJacobian.get(qpInputToPack.taskJacobian);
      }
      else
      {
         recordTaskJacobian(qpInputToPack.taskJacobian);
      }
      return true;
   }

   /**
    * Configures the appropriate variable substitution to perform in the QP for satisfying the physical
    * constraint for a kinematic loop.
    * <p>
    * The {@code qpVariableSubstitutionToPack} can then be add to the QP to register an additional
    * substitution perform.
    * </p>
    * 
    * @param function                     the explicit function representing a kinematic loop in the
    *                                     multi-body system.
    * @param qpVariableSubstitutionToPack the variable substitution to be configured. Modified.
    * @see InverseDynamicsQPSolver#addAccelerationSubstitution(QPVariableSubstitution)
    * @see InverseKinematicsQPSolver#addVariableSubstitution(QPVariableSubstitution)
    */
   public void convertKinematicLoopFunction(KinematicLoopFunction function, QPVariableSubstitutionInterface<?> qpVariableSubstitutionToPack)
   {
      DMatrixRMaj loopJacobian = function.getLoopJacobian();
      DMatrixRMaj loopConvectiveTerm = function.getLoopConvectiveTerm();
      List<? extends OneDoFJointReadOnly> loopJoints = function.getLoopJoints();
      int[] actuatedJointIndices = function.getActuatedJointIndices();

      if (loopJacobian.getNumRows() != loopConvectiveTerm.getNumRows())
         throw new IllegalArgumentException("Inconsistent dimensions: number of rows in Jacobian: " + loopJacobian.getNumRows() + ", number of joints: "
                                            + loopJoints.size());

      if (loopJacobian.getNumRows() != loopConvectiveTerm.getNumRows())
         throw new MatrixDimensionException("Inconsistent dimensions: loopJacobian.numRows=" + loopJacobian.getNumRows() + ", loopConvectiveTerm.numRows="
                                            + loopConvectiveTerm.getNumRows());

      if (actuatedJointIndices.length != loopJacobian.getNumCols())
         throw new IllegalArgumentException("Inconsistent dimensions: number of actuated joints: " + actuatedJointIndices.length
                                            + ", number of columns in Jacobian: " + loopJacobian.getNumCols());

      qpVariableSubstitutionToPack.reshape(loopJoints.size(), loopJacobian.getNumRows());

      qpVariableSubstitutionToPack.getTransformation().set(loopJacobian);
      qpVariableSubstitutionToPack.getBias().set(loopConvectiveTerm);

      for (int i = 0; i < loopJoints.size(); i++)
      {
         qpVariableSubstitutionToPack.getVariableIndices()[i] = jointIndexHandler.getOneDoFJointIndex(loopJoints.get(i));
      }

      for (int i = 0; i < actuatedJointIndices.length; i++)
      {
         qpVariableSubstitutionToPack.getActiveIndices().add(actuatedJointIndices[i]);
      }
   }

   public boolean convertQPObjectiveCommand(QPObjectiveCommand commandToConvert, NativeQPInputTypeA qpInputToPack)
   {
      //      LogTools.info("-------------------------WBCC------------------------------------");
      //      LogTools.info("-------------------------WBCC------------------------------------");
      //      LogTools.info(EuclidCoreIOTools.getArrayString(", ", jointIndexHandler.getIndexedJoints(), j -> j.getName()));
      //      LogTools.info("-------------------------WBCC------------------------------------");
      //      LogTools.info("-------------------------WBCC------------------------------------");
      //      LogTools.info("-------------------------WBCC------------------------------------");
      DMatrixRMaj jacobian = commandToConvert.getJacobian();
      DMatrixRMaj objective = commandToConvert.getObjective();
      DMatrixRMaj selectionMatrix = commandToConvert.getSelectionMatrix();
      DMatrixRMaj weightMatrix = commandToConvert.getWeightMatrix();

      int taskSize = selectionMatrix.getNumRows();
      // TODO check the weights to determine the task size as well.

      if (taskSize == 0)
         return false;

      if (jacobian.getNumCols() != numberOfDoFs)
      {
         LogTools.error("Jacobian is not of the right size: {}, expected: {}", jacobian.getNumCols(), numberOfDoFs);
         return false;
      }

      qpInputToPack.reshape(taskSize);
      qpInputToPack.setConstraintType(ConstraintType.OBJECTIVE);

      // Copy into native
      nativeTempSelectionMatrix.set(selectionMatrix);
      nativeTempTaskWeight.set(weightMatrix);
      nativeTempTaskObjective.set(objective);
      nativeTempJacobian.set(jacobian);

      // W = S * W * S^T
      qpInputToPack.setUseWeightScalar(false);
      nativeTempTaskWeightSubspace.mult(nativeTempSelectionMatrix, nativeTempTaskWeight);
      qpInputToPack.taskWeightMatrix.multTransB(nativeTempTaskWeightSubspace, nativeTempSelectionMatrix);

      // b = S * b
      qpInputToPack.taskObjective.mult(nativeTempSelectionMatrix, nativeTempTaskObjective);

      // J = S * J
      qpInputToPack.taskJacobian.reshape(taskSize, jacobianCalculator.getNumberOfDegreesOfFreedom());
      qpInputToPack.taskJacobian.mult(nativeTempSelectionMatrix, nativeTempJacobian);

      if (commandToConvert.isNullspaceProjected())
      {
         mergeAllTaskJacobians();
         accelerationNativeNullspaceProjector.project(qpInputToPack.taskJacobian,
                                                      nativeAllTaskJacobian,
                                                      qpInputToPack.taskJacobian,
                                                      nullspaceProjectionAlpha.getValue());
      }
      else
      {
         recordTaskJacobian(qpInputToPack.taskJacobian);
      }

      return true;
   }

   /**
    * Converts a {@link SpatialAccelerationCommand} into a {@link NativeQPInputTypeA}.
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
   public boolean convertSpatialAccelerationCommand(SpatialAccelerationCommand commandToConvert, NativeQPInputTypeA qpInputToPack)
   {
      commandToConvert.getControlFrame(controlFrame);
      // Gets the M-by-6 selection matrix S.
      commandToConvert.getSelectionMatrix(controlFrame, tempSelectionMatrix);
      nativeTempSelectionMatrix.set(tempSelectionMatrix);
      int taskSize = tempSelectionMatrix.getNumRows();
      // TODO check the weights to determine the task size as well.

      // TODO probably need to resize the weight matrix
      // TODO stop just doing things in java land and pushing to native!
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

         commandToConvert.getWeightMatrix(controlFrame, tempTaskWeight);
         nativeTempTaskWeight.set(tempTaskWeight);

         nativeTempTaskWeightSubspace.mult(nativeTempSelectionMatrix, nativeTempTaskWeight);
         qpInputToPack.taskWeightMatrix.multTransB(nativeTempTaskWeightSubspace, nativeTempSelectionMatrix);
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
      nativeTempTaskObjective.set(tempTaskObjective);
      nativeTempConvectiveTerm.set(jacobianCalculator.getConvectiveTermMatrix());
      nativeTempTaskObjective.addEquals(-1.0, nativeTempConvectiveTerm);
      qpInputToPack.taskObjective.mult(nativeTempSelectionMatrix, nativeTempTaskObjective);

      // Compute the M-by-N task Jacobian: J = S * J
      // Step 1, let's get the 'small' Jacobian matrix j.
      // It is called small as its number of columns is equal to the number of DoFs to its kinematic chain which is way smaller than the number of robot DoFs.
      nativeTempJacobian.set(jacobianCalculator.getJacobianMatrix());
      nativeTempTaskJacobian.mult(nativeTempSelectionMatrix, nativeTempJacobian);

      // Dealing with the primary base:
      RigidBodyBasics primaryBase = commandToConvert.getPrimaryBase();
      List<JointReadOnly> jointsUsedInTask = jacobianCalculator.getJointsFromBaseToEndEffector();

      // Step 2: The small Jacobian matrix into the full Jacobian matrix. Proper indexing has to be ensured, so it is handled by the jointIndexHandler.
      jointIndexHandler.compactBlockToFullBlockIgnoreUnindexedJoints(jointsUsedInTask, nativeTempTaskJacobian, qpInputToPack.taskJacobian);

      if (primaryBase == null)
      { // No primary base provided for this task.
        // Record the resulting Jacobian matrix for the privileged configuration.
         tempTaskJacobian.set(qpInputToPack.taskJacobian);
         recordTaskJacobian(tempTaskJacobian);
         // We're done!
      }
      else
      { // A primary base has been provided, two things are happening here:
        // 1- A weight is applied on the joints between the base and the primary base with objective to reduce their involvement for this task.
        // 2- The Jacobian is transformed before being recorded such that the privileged configuration is only applied from the primary base to the end-effector.
         nativeTempPrimaryTaskJacobian.set(qpInputToPack.taskJacobian);

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
                  qpInputToPack.taskJacobian.scaleColumn(dofIndex, scaleFactor);
                  // Zero out the task Jacobian at the joint's column(s) so it is removed from the nullspace calculation for applying the privileged configuration.
                  nativeTempPrimaryTaskJacobian.zeroColumn(dofIndex);
               }
            }
         }

         // Record the resulting Jacobian matrix which only zeros before the primary base for the privileged configuration.
         recordTaskJacobian(nativeTempPrimaryTaskJacobian);
         // We're done!
      }

      return true;
   }

   /**
    * Converts a {@link SpatialVelocityCommand} into a {@link QPInputTypeA}.
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
   public boolean convertSpatialVelocityCommand(SpatialVelocityCommand commandToConvert, QPInputTypeA qpInputToPack)
   {
      // Gets the M-by-6 selection matrix S.
      commandToConvert.getControlFrame(controlFrame);
      commandToConvert.getSelectionMatrix(controlFrame, tempSelectionMatrix);
      int taskSize = tempSelectionMatrix.getNumRows();
      // TODO check the weights to determine the task size as well.

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
         CommonOps_DDRM.mult(tempSelectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
         CommonOps_DDRM.multTransB(tempTaskWeightSubspace, tempSelectionMatrix, qpInputToPack.taskWeightMatrix);
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
      CommonOps_DDRM.mult(tempSelectionMatrix, tempTaskObjective, qpInputToPack.taskObjective);

      // Compute the M-by-N task Jacobian: J = S * J
      // Step 1, let's get the 'small' Jacobian matrix j.
      // It is called small as its number of columns is equal to the number of DoFs to its kinematic chain which is way smaller than the number of robot DoFs.
      tempTaskJacobian.reshape(taskSize, jacobianCalculator.getNumberOfDegreesOfFreedom());
      CommonOps_DDRM.mult(tempSelectionMatrix, jacobianCalculator.getJacobianMatrix(), tempTaskJacobian);

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
    * Converts a {@link MomentumRateCommand} into a {@link NativeQPInputTypeA}.
    * 
    * @return true if the command was successfully converted.
    */
   public boolean convertMomentumRateCommand(MomentumRateCommand commandToConvert, NativeQPInputTypeA qpInputToPack)
   {
      commandToConvert.getSelectionMatrix(centerOfMassFrame, tempSelectionMatrix);
      int taskSize = tempSelectionMatrix.getNumRows();
      // TODO check the weights to determine the task size as well.

      if (taskSize == 0)
         return false;

      qpInputToPack.reshape(taskSize);
      qpInputToPack.setUseWeightScalar(false);
      qpInputToPack.setConstraintType(ConstraintType.OBJECTIVE);

      // Compute the weight: W = S * W * S^T
      tempTaskWeight.reshape(SpatialAcceleration.SIZE, SpatialAcceleration.SIZE);
      commandToConvert.getWeightMatrix(tempTaskWeight);

      nativeTempSelectionMatrix.set(tempSelectionMatrix);
      nativeTempTaskWeight.set(tempTaskWeight);

      nativeTempTaskWeightSubspace.mult(nativeTempSelectionMatrix, nativeTempTaskWeight);
      qpInputToPack.taskWeightMatrix.multTransB(nativeTempTaskWeightSubspace, nativeTempSelectionMatrix);

      // Compute the task Jacobian: J = S * A
      // FIXME figure out how to not have to set this every time
      nativeCentroidalMomentumMatrix.set(getCentroidalMomentumMatrix());

      if (commandToConvert.isConsiderAllJoints())
      {
         qpInputToPack.taskJacobian.mult(nativeTempSelectionMatrix, nativeCentroidalMomentumMatrix);
      }
      else
      {
         nativeTempTaskJacobian.mult(nativeTempSelectionMatrix, nativeCentroidalMomentumMatrix);
         qpInputToPack.taskJacobian.zero();
         List<JointReadOnly> jointSelection = commandToConvert.getJointSelection();

         for (int i = 0; i < jointSelection.size(); i++)
         {
            int[] jointIndices = jointIndexHandler.getJointIndices(jointSelection.get(i));
            int jointFirstIndex = jointIndices[0];
            int jointLastIndex = jointIndices[jointIndices.length - 1];
            qpInputToPack.taskJacobian.insert(nativeTempTaskJacobian, 0, taskSize, jointFirstIndex, jointLastIndex + 1, 0, jointFirstIndex);
         }
      }

      commandToConvert.getMomentumRate(angularMomentum, linearMomentum);
      angularMomentum.changeFrame(centerOfMassFrame);
      linearMomentum.changeFrame(centerOfMassFrame);

      // TODO there's possibly a better time to push this into native land
      if (commandToConvert.isConsiderAllJoints())
      {
         angularMomentum.get(0, tempTaskObjective);
         linearMomentum.get(3, tempTaskObjective);
         DMatrixRMaj convectiveTerm = centroidalMomentumRateCalculator.getBiasSpatialForceMatrix();

         // Compute the task objective: p = S * ( hDot - ADot qDot )
         CommonOps_DDRM.subtractEquals(tempTaskObjective, convectiveTerm);
      }
      else
      {
         centroidalMomentumRateCalculator.getBiasSpatialForceMatrix(commandToConvert.getJointSelection(), tempTaskObjective);

         // Compute the task objective: p = S * ( hDot - ADot qDot )
         for (Axis3D axis : Axis3D.values)
         {
            tempTaskObjective.set(0 + axis.ordinal(), angularMomentum.getElement(axis) - tempTaskObjective.get(0 + axis.ordinal()));
            tempTaskObjective.set(3 + axis.ordinal(), linearMomentum.getElement(axis) - tempTaskObjective.get(3 + axis.ordinal()));
         }
      }
      nativeTempTaskObjective.set(tempTaskObjective);
      qpInputToPack.taskObjective.mult(nativeTempSelectionMatrix, nativeTempTaskObjective);

      recordTaskJacobian(qpInputToPack.taskJacobian);

      return true;
   }

   private final QPInputTypeB tempBInput = new QPInputTypeB(6);

   /**
    * Converts a {@link LinearMomentumRateCostCommand} into a {@link NativeQPInputTypeB}.
    *
    * @return true if the command was successfully converted.
    */
   public boolean convertLinearMomentumRateCostCommand(LinearMomentumRateCostCommand commandToConvert, NativeQPInputTypeB qpInputToPack)
   {
      if (convertLinearMomentumRateCostCommand(commandToConvert, tempBInput))
      {
         qpInputToPack.getTaskWeightMatrix().set(tempBInput.getTaskWeightMatrix());
         qpInputToPack.getDirectCostHessian().set(tempBInput.getDirectCostHessian());
         qpInputToPack.getDirectCostGradient().set(tempBInput.getDirectCostGradient());
         qpInputToPack.getTaskJacobian().set(tempBInput.getTaskJacobian());
         qpInputToPack.getTaskConvectiveTerm().set(tempBInput.getTaskConvectiveTerm());

         return true;
      }
      else
      {
         return false;
      }
   }

   /**
    * Converts a {@link LinearMomentumRateCostCommand} into a {@link NativeQPInputTypeB}.
    *
    * @return true if the command was successfully converted.
    */
   public boolean convertLinearMomentumRateCostCommand(LinearMomentumRateCostCommand commandToConvert, QPInputTypeB qpInputToPack)
   {
      tempSelectionMatrix.zero();
      commandToConvert.getSelectionMatrix(centerOfMassFrame, tempSelectionMatrix);
      int taskSize = tempSelectionMatrix.getNumRows();
      // TODO check the weights to determine the task size as well.

      if (taskSize == 0)
         return false;

      qpInputToPack.reshape(taskSize);
      qpInputToPack.setUseWeightScalar(false);

      // Compute the weight: W = S * W * S^T
      tempTaskWeight.reshape(Wrench.SIZE, Wrench.SIZE);
      tempTaskWeightSubspace.reshape(taskSize, 3);
      commandToConvert.getWeightMatrix(tempTaskWeight);
      CommonOps_DDRM.mult(tempSelectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
      CommonOps_DDRM.multTransB(tempTaskWeightSubspace, tempSelectionMatrix, qpInputToPack.taskWeightMatrix);

      // Compute the hessian: H = S * H * S^T
      tempTaskWeight.set(commandToConvert.getMomentumRateHessian());
      CommonOps_DDRM.mult(tempSelectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
      CommonOps_DDRM.multTransB(tempTaskWeightSubspace, tempSelectionMatrix, qpInputToPack.directCostHessian);

      // Compute the task Jacobian: J = S * A
      DMatrixRMaj centroidalMomentumMatrix = getCentroidalMomentumMatrix();
      CommonOps_DDRM.mult(tempSelectionMatrix, centroidalMomentumMatrix, qpInputToPack.taskJacobian);

      // Compute the gradient: g = S * g
      CommonOps_DDRM.multTransB(tempSelectionMatrix, commandToConvert.getMomentumRateGradient(), qpInputToPack.directCostGradient);

      // Compute the task convective term: p = S * ADot qDot
      DMatrixRMaj convectiveTerm = getCentroidalMomentumConvectiveTerm();
      CommonOps_DDRM.mult(tempSelectionMatrix, convectiveTerm, qpInputToPack.taskConvectiveTerm);

      recordTaskJacobian(qpInputToPack.taskJacobian);

      return true;
   }

   /**
    * Converts a {@link MomentumCommand} into a {@link QPInputTypeA}.
    * 
    * @return true if the command was successfully converted.
    */
   public boolean convertMomentumCommand(MomentumCommand commandToConvert, QPInputTypeA qpInputToPack)
   {
      commandToConvert.getSelectionMatrix(centerOfMassFrame, tempSelectionMatrix);
      int taskSize = tempSelectionMatrix.getNumRows();
      // TODO check the weights to determine the task size as well.

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
         CommonOps_DDRM.mult(tempSelectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
         CommonOps_DDRM.multTransB(tempTaskWeightSubspace, tempSelectionMatrix, qpInputToPack.taskWeightMatrix);
      }

      // Compute the task Jacobian: J = S * A
      DMatrixRMaj centroidalMomentumMatrix = getCentroidalMomentumMatrix();
      CommonOps_DDRM.mult(tempSelectionMatrix, centroidalMomentumMatrix, qpInputToPack.taskJacobian);

      commandToConvert.getMomentumRate(angularMomentum, linearMomentum);
      angularMomentum.changeFrame(centerOfMassFrame);
      linearMomentum.changeFrame(centerOfMassFrame);
      angularMomentum.get(0, tempTaskObjective);
      linearMomentum.get(3, tempTaskObjective);

      // Compute the task objective: p = S * h
      CommonOps_DDRM.mult(tempSelectionMatrix, tempTaskObjective, qpInputToPack.taskObjective);

      recordTaskJacobian(qpInputToPack.taskJacobian);

      return true;
   }

   /**
    * Converts a {@link LinearMomentumConvexConstraint2DCommand} into a {@link QPInputTypeA} intended
    * to be consumed by {@link InverseKinematicsQPSolver}.
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
   public boolean convertLinearMomentumConvexConstraint2DCommand(LinearMomentumConvexConstraint2DCommand command, QPInputTypeA qpInputToPack)
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
      DMatrixRMaj centroidalMomentumMatrix = getCentroidalMomentumMatrix();
      CommonOps_DDRM.mult(tempSelectionMatrix, centroidalMomentumMatrix, tempTaskJacobian);

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
    * Sets up the {@link QPInputTypeA#taskJacobian} and {@link QPInputTypeA#taskObjective} to formulate
    * a constraint with respect to 2D line going through {@code firstPointOnLine} and
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
   private void setupLineConstraint(int constraintIndex,
                                    Tuple2DReadOnly firstPointOnLine,
                                    Tuple2DReadOnly secondPointOnLine,
                                    DMatrixRMaj centroidalMomemtumMatrixLinearXY,
                                    QPInputTypeA qpInputToPack)
   {
      double directionX = secondPointOnLine.getX() - firstPointOnLine.getX();
      double directionY = secondPointOnLine.getY() - firstPointOnLine.getY();
      double norm = Math.sqrt(EuclidCoreTools.normSquared(directionX, directionY));
      directionX /= norm;
      directionY /= norm;

      lineConstraintSelection.set(0, -directionY);
      lineConstraintSelection.set(1, directionX);
      lineConstraintJacobian.reshape(1, numberOfDoFs);
      CommonOps_DDRM.mult(lineConstraintSelection, centroidalMomemtumMatrixLinearXY, lineConstraintJacobian);
      CommonOps_DDRM.insert(lineConstraintJacobian, qpInputToPack.taskJacobian, constraintIndex, 0);

      qpInputToPack.taskObjective.set(constraintIndex, directionX * firstPointOnLine.getY() - directionY * firstPointOnLine.getX());
   }

   /**
    * Converts a {@link JointspaceAccelerationCommand} into a {@link QPInputTypeA}.
    *
    * @return true if the command was successfully converted.
    */
   public boolean convertJointspaceAccelerationCommand(JointspaceAccelerationCommand commandToConvert, NativeQPInputTypeA qpInputToPack)
   {
      int taskSize = 0;

      for (int jointIndex = 0; jointIndex < commandToConvert.getNumberOfJoints(); jointIndex++)
      {
         if (commandToConvert.getWeight(jointIndex) > minWeightForTaskInclusion)
            taskSize++;
      }

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
         if (commandToConvert.getWeight(jointIndex) <= minWeightForTaskInclusion)
            continue;

         JointBasics joint = commandToConvert.getJoint(jointIndex);
         double weight = commandToConvert.getWeight(jointIndex);
         int[] columns = jointIndexHandler.getJointIndices(joint);
         if (columns == null)
            return false;

         qpInputToPack.taskObjective.insert(commandToConvert.getDesiredAcceleration(jointIndex), row, 0);
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
    * Converts a {@link JointspaceVelocityCommand} into a {@link QPInputTypeA}.
    *
    * @return true if the command was successfully converted.
    */
   public boolean convertJointspaceVelocityCommand(JointspaceVelocityCommand commandToConvert, QPInputTypeA qpInputToPack)
   {
      int taskSize = MultiBodySystemTools.computeDegreesOfFreedom(commandToConvert.getJoints());
      // TODO check the weights to determine the task size as well.

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

         CommonOps_DDRM.insert(commandToConvert.getDesiredVelocity(jointIndex), qpInputToPack.taskObjective, row, 0);
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
    * Sets up an objective for matching a desired set of joint torques. <br>
    * Given a desired torque vector tau_d, this task is: <br>
    * [M (-J^T beta)] [q_dd^T rho^T]^T = tau_d - (Cq_d + G) <br>
    *
    * @param jointTorqueCommand                 desired joint torque command to convert
    * @param hasFloatingBase                    whether the robot has a floating base, used for
    *                                           indexing the mass matrix
    * @param qpInputToPack                      the result of the conversion.
    * @param bodyMassMatrix,                    M in the above equation
    * @param bodyContactForceJacobianTranspose, J^T beta in the above equation
    * @param bodyGravityCoriolisMatrix,         {Cq_d + G} in the above equation
    */
   public boolean convertJointTorqueCommand(JointTorqueCommand jointTorqueCommand,
                                            boolean hasFloatingBase,
                                            NativeQPInputTypeA qpInputToPack,
                                            DMatrixRMaj bodyMassMatrix,
                                            DMatrixRMaj bodyContactForceJacobianTranspose,
                                            DMatrixRMaj bodyGravityCoriolisMatrix)
   {
      int taskSize = MultiBodySystemTools.computeDegreesOfFreedom(jointTorqueCommand.getJoints());

      if (taskSize == 0)
         return false;

      qpInputToPack.reshape(taskSize);
      qpInputToPack.setUseWeightScalar(false);
      if (jointTorqueCommand.getConstraintType() == ConstraintType.LEQ_INEQUALITY || jointTorqueCommand.getConstraintType() == ConstraintType.GEQ_INEQUALITY)
         qpInputToPack.setConstraintType(jointTorqueCommand.getConstraintType());
      else
         qpInputToPack.setConstraintType(jointTorqueCommand.isHardConstraint() ? ConstraintType.EQUALITY : ConstraintType.OBJECTIVE);
      qpInputToPack.taskJacobian.zero();
      qpInputToPack.taskObjective.zero();
      qpInputToPack.taskWeightMatrix.zero();

      int row = 0;
      for (int jointIndex = 0; jointIndex < jointTorqueCommand.getNumberOfJoints(); jointIndex++)
      {
         JointBasics joint = jointTorqueCommand.getJoints().get(jointIndex);
         DMatrixRMaj desiredTorque = jointTorqueCommand.getDesiredTorque(jointIndex);
         int[] jointIndices = jointIndexHandler.getJointIndices(joint);

         for (int dof = 0; dof < joint.getDegreesOfFreedom(); dof++)
         {
            int bodyDofIndex = jointIndices[dof] - (hasFloatingBase ? 6 : 0);
            double weight = jointTorqueCommand.getWeight(jointIndex);

            // Pack the corresponding row of body mass matrix M
            qpInputToPack.taskJacobian.insert(bodyMassMatrix, bodyDofIndex, bodyDofIndex + 1, 0, numberOfDoFs, row, 0);

            // Pack the corresponding row of contact jacobian {J^T beta}
            int rhoSize = bodyContactForceJacobianTranspose.getNumCols();
            if (rhoSize > 0)
               qpInputToPack.taskJacobian.insert(bodyContactForceJacobianTranspose, bodyDofIndex, bodyDofIndex + 1, 0, rhoSize, row, numberOfDoFs);

            // Pack the corresponding row of the objective, {tau_d - Cq_d - G}
            qpInputToPack.taskObjective.set(row, 0, desiredTorque.get(dof, 0) - bodyGravityCoriolisMatrix.get(bodyDofIndex, 0));
            qpInputToPack.taskWeightMatrix.set(row, row, weight);
            row++;
         }
      }

      return true;
   }

   private void recordTaskJacobian(NativeMatrix taskJacobian)
   {
      int taskSize = taskJacobian.getNumRows();
      int oldSize = nativeAllTaskJacobian.getNumRows();
      nativeTempTaskJacobian.set(nativeAllTaskJacobian);
      nativeAllTaskJacobian.reshape(oldSize + taskSize, numberOfDoFs);
      nativeAllTaskJacobian.insert(nativeTempTaskJacobian, 0, 0);
      nativeAllTaskJacobian.insert(taskJacobian, oldSize, 0);
   }

   private void recordTaskJacobian(DMatrixRMaj taskJacobian)
   {
      int taskSize = taskJacobian.getNumRows();
      allTaskJacobian.reshape(allTaskJacobian.getNumRows() + taskSize, numberOfDoFs, true);
      CommonOps_DDRM.insert(taskJacobian, allTaskJacobian, allTaskJacobian.getNumRows() - taskSize, 0);
   }

   private void mergeAllTaskJacobians()
   {
      int nativeSize = nativeAllTaskJacobian.getNumRows();
      int ejmlSize = allTaskJacobian.getNumRows();
      if (ejmlSize > 0)
      {
         nativeTempTaskJacobian.set(nativeAllTaskJacobian);
         nativeAllTaskJacobian.reshape(nativeSize + ejmlSize, numberOfDoFs);
         nativeAllTaskJacobian.insert(nativeTempTaskJacobian, 0, 0);
         nativeAllTaskJacobian.insert(allTaskJacobian, nativeSize, 0);
      }

      if (nativeSize + ejmlSize > sizeOfAllTasksJacobian.getIntegerValue())
         sizeOfAllTasksJacobian.set(nativeSize + ejmlSize);
   }

   public DMatrixRMaj getCentroidalMomentumMatrix()
   {
      if (centroidalMomentumCalculator != null)
         return centroidalMomentumCalculator.getCentroidalMomentumMatrix();
      else
         return centroidalMomentumRateCalculator.getCentroidalMomentumMatrix();
   }

   public DMatrixRMaj getCentroidalMomentumConvectiveTerm()
   {
      return centroidalMomentumRateCalculator.getBiasSpatialForceMatrix();
   }

   private final SpatialForce momentumRate = new SpatialForce();

   public SpatialForceReadOnly computeCentroidalMomentumRateFromSolution(DMatrixRMaj jointAccelerations)
   {
      centroidalMomentumRateCalculator.getMomentumRate(jointAccelerations, momentumRate);
      return momentumRate;
   }

   private final Momentum momentum = new Momentum();

   public MomentumReadOnly computeCentroidalMomentumFromSolution(DMatrixRMaj jointVelocities)
   {
      if (centroidalMomentumCalculator != null)
         centroidalMomentumCalculator.getMomentum(jointVelocities, momentum);
      else
         centroidalMomentumRateCalculator.getMomentum(jointVelocities, momentum);
      return momentum;
   }
}
