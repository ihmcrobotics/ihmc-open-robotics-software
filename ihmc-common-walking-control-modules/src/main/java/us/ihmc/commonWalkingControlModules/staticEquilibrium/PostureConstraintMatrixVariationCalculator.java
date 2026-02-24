package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.inverseKinematics.RobotJointVelocityAccelerationIntegrator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoRegistry;

public class PostureConstraintMatrixVariationCalculator
{
   private final FullHumanoidRobotModel fullRobotModel;
   private final JointBasics[] controlledJoints;
   private final WholeBodyContactState wholeBodyContactState;
   private final StabilityMarginOptimizationModule stabilityMarginOptimizationModule;

   private final RobotJointVelocityAccelerationIntegrator integrator;
   private final double integrationDT;
   private final DMatrixRMaj initialJointConfiguration = new DMatrixRMaj(0);
   private final DMatrixRMaj initialJointVelocity = new DMatrixRMaj(0);

   private final DMatrixRMaj actuationConstraintNominal = new DMatrixRMaj(0);
   private final DMatrixRMaj actuationConstraintVariation = new DMatrixRMaj(0);
   private final DMatrixRMaj constraintMatrixVariation = new DMatrixRMaj(0);
   private final DMatrixRMaj solverConstraintVariation = new DMatrixRMaj(0);

   private final ExecutionTimer constraintVarCalculatorFramesTimer;
   private final ExecutionTimer constraintVarCalculatorContactUpdateTimer;
   private final ExecutionTimer constraintVarCalculatorSolverMathTimer;

   private final SideDependentList<GeometricJacobianCalculator> jacobianCalculator = new SideDependentList<>(s -> new GeometricJacobianCalculator());

   public PostureConstraintMatrixVariationCalculator(FullHumanoidRobotModel fullRobotModel,
                                                     WholeBodyContactState wholeBodyContactState,
                                                     StabilityMarginOptimizationModule stabilityMarginOptimizationModule,
                                                     double integrationDT,
                                                     YoRegistry registry)
   {
      this.fullRobotModel = fullRobotModel;
      this.controlledJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel);
      this.wholeBodyContactState = wholeBodyContactState;
      this.stabilityMarginOptimizationModule = stabilityMarginOptimizationModule;
      this.integrator = new RobotJointVelocityAccelerationIntegrator(() -> integrationDT);
      this.integrationDT = integrationDT;

      for (RobotSide robotSide : RobotSide.values)
      {
         jacobianCalculator.get(robotSide).setKinematicChain(fullRobotModel.getChest(), fullRobotModel.getHand(robotSide));
      }

      int velocityDofs = MultiBodySystemTools.computeDegreesOfFreedom(controlledJoints);
      initialJointConfiguration.reshape(velocityDofs + 1, 1); // Floating base is 6 dofs but backed by a quaternion so there is an extra dof
      initialJointVelocity.reshape(velocityDofs, 1);

      constraintVarCalculatorFramesTimer = new ExecutionTimer("constraintVarCalculatorFramesTimer", registry);
      constraintVarCalculatorContactUpdateTimer = new ExecutionTimer("constraintVarCalculatorContactUpdateTimer", registry);
      constraintVarCalculatorSolverMathTimer = new ExecutionTimer("constraintVarCalculatorSolverMathTimer", registry);
   }

   public void initializeFiniteDifference()
   {
      actuationConstraintNominal.set(wholeBodyContactState.getActuationConstraintMatrix());
      MultiBodySystemTools.extractJointsState(controlledJoints, JointStateType.CONFIGURATION, initialJointConfiguration);
   }

   public DMatrixRMaj computeFiniteDifference(DMatrixRMaj qd)
   {
      /* Integrate one preview tick and copy to robot model */
      integrator.integrateJointVelocities(controlledJoints, qd);
      MultiBodySystemTools.insertJointsState(controlledJoints, JointStateType.CONFIGURATION, integrator.getJointConfigurations());

      constraintVarCalculatorFramesTimer.startMeasurement();
      fullRobotModel.updateFrames();
      constraintVarCalculatorFramesTimer.stopMeasurement();

      /* Finite difference constraint matrix */
      constraintVarCalculatorContactUpdateTimer.startMeasurement();
      wholeBodyContactState.updateActuationConstraintMatrix();
      constraintVarCalculatorContactUpdateTimer.stopMeasurement();

      constraintVarCalculatorSolverMathTimer.startMeasurement();
      CommonOps_DDRM.subtract(wholeBodyContactState.getActuationConstraintMatrix(), actuationConstraintNominal, actuationConstraintVariation);
      CommonOps_DDRM.scale(1.0 / integrationDT, actuationConstraintVariation);

      DMatrixRMaj solverToNominalTransformation = stabilityMarginOptimizationModule.getSolverToNominalTransformation();
      DMatrixRMaj nominalConstraintMatrix = stabilityMarginOptimizationModule.getNominalConstraintMatrix();

      int numInequalityDynamicsConstraints = stabilityMarginOptimizationModule.getNumDynamicsConstraints();
      int numEqualityDynamicsConstraints = 2 * numInequalityDynamicsConstraints;

      constraintMatrixVariation.reshape(nominalConstraintMatrix.getNumRows(), nominalConstraintMatrix.getNumCols());
      MatrixTools.setMatrixBlock(constraintMatrixVariation, numEqualityDynamicsConstraints, 0, actuationConstraintVariation, 0, 0,
                                 actuationConstraintVariation.getNumRows(), actuationConstraintVariation.getNumCols(), 1.0);

      /* Transform constraint matrix variation to rho-space */
      CommonOps_DDRM.mult(constraintMatrixVariation, solverToNominalTransformation, solverConstraintVariation);

      /* Soft reset to initial configuration (don't call update frames by default) */
      MultiBodySystemTools.insertJointsState(controlledJoints, JointStateType.CONFIGURATION, initialJointConfiguration);

      constraintVarCalculatorSolverMathTimer.stopMeasurement();
      return solverConstraintVariation;
   }

   public void resetToInitialJointState()
   {
      /* Reset joint configuration to initial */
      MultiBodySystemTools.insertJointsState(controlledJoints, JointStateType.CONFIGURATION, initialJointConfiguration);

      fullRobotModel.updateFrames();
      wholeBodyContactState.getActuationConstraintMatrix().set(actuationConstraintNominal);
   }

   public DMatrixRMaj computeJacobianRate(DMatrixRMaj qd)
   {
      /* Integrate one preview tick and copy to robot model */
      MultiBodySystemTools.insertJointsState(controlledJoints, JointStateType.VELOCITY, qd);

      DMatrixRMaj solverToNominalTransformation = stabilityMarginOptimizationModule.getSolverToNominalTransformation();
      DMatrixRMaj nominalConstraintMatrix = stabilityMarginOptimizationModule.getNominalConstraintMatrix();

      int numInequalityDynamicsConstraints = stabilityMarginOptimizationModule.getNumDynamicsConstraints();
      int numEqualityDynamicsConstraints = 2 * numInequalityDynamicsConstraints;

      constraintMatrixVariation.reshape(nominalConstraintMatrix.getNumRows(), nominalConstraintMatrix.getNumCols());
      MatrixTools.setMatrixBlock(constraintMatrixVariation, numEqualityDynamicsConstraints, 0, actuationConstraintVariation, 0, 0,
                                 actuationConstraintVariation.getNumRows(), actuationConstraintVariation.getNumCols(), 1.0);

      /* Transform constraint matrix variation to rho-space */
      CommonOps_DDRM.mult(constraintMatrixVariation, solverToNominalTransformation, solverConstraintVariation);

      /* Reset joint configuration to initial */
      MultiBodySystemTools.insertJointsState(controlledJoints, JointStateType.CONFIGURATION, initialJointConfiguration);
      constraintVarCalculatorSolverMathTimer.stopMeasurement();

      return solverConstraintVariation;
   }

   public DMatrixRMaj getSolverConstraintVariation()
   {
      return solverConstraintVariation;
   }

   public DMatrixRMaj getInitialJointConfiguration()
   {
      return initialJointConfiguration;
   }

   public DMatrixRMaj getActuationConstraintNominal()
   {
      return actuationConstraintNominal;
   }
}
