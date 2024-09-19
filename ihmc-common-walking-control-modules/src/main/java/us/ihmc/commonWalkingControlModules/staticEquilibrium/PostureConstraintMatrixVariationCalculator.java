package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.inverseKinematics.RobotJointVelocityAccelerationIntegrator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.tools.EuclidCoreTools;
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
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class PostureConstraintMatrixVariationCalculator
{
   private final FullHumanoidRobotModel fullRobotModel;
   private final JointBasics[] controlledJoints;
   private final WholeBodyContactState wholeBodyContactState;
   private final CenterOfMassStabilityMarginOptimizationModule stabilityMarginOptimizationModule;

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
   private final ExecutionTimer constraintVarCalculatorSolverUpdateTimer;
   private final ExecutionTimer constraintVarCalculatorSolverMathTimer;

   private final SideDependentList<GeometricJacobianCalculator> jacobianCalculator = new SideDependentList<>(s -> new GeometricJacobianCalculator());

   public PostureConstraintMatrixVariationCalculator(FullHumanoidRobotModel fullRobotModel,
                                                     WholeBodyContactState wholeBodyContactState,
                                                     CenterOfMassStabilityMarginOptimizationModule stabilityMarginOptimizationModule,
                                                     double integrationDT,
                                                     YoRegistry registry)
   {
      this.fullRobotModel = fullRobotModel;
      this.controlledJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel);
      this.wholeBodyContactState = wholeBodyContactState;
      this.stabilityMarginOptimizationModule = stabilityMarginOptimizationModule;
      this.integrator = new RobotJointVelocityAccelerationIntegrator(integrationDT);
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
      constraintVarCalculatorSolverUpdateTimer = new ExecutionTimer("constraintVarCalculatorSolverUpdateTimer", registry);
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
      //      wholeBodyContactState.update();
      wholeBodyContactState.updateActuationConstraintMatrix(false);
      constraintVarCalculatorContactUpdateTimer.stopMeasurement();

      constraintVarCalculatorSolverUpdateTimer.startMeasurement();
//      stabilityMarginOptimizationModule.updateContactState(wholeBodyContactState);
      constraintVarCalculatorSolverUpdateTimer.stopMeasurement();

      constraintVarCalculatorSolverMathTimer.startMeasurement();
      CommonOps_DDRM.subtract(wholeBodyContactState.getActuationConstraintMatrix(), actuationConstraintNominal, actuationConstraintVariation);
      CommonOps_DDRM.scale(1.0 / integrationDT, actuationConstraintVariation);

      int numCoMVariables = 2;
      int numEquilibriumConstraints = 12;
      constraintMatrixVariation.reshape(actuationConstraintVariation.getNumRows() + numEquilibriumConstraints,
                                        actuationConstraintVariation.getNumCols() + numCoMVariables);
      MatrixTools.setMatrixBlock(constraintMatrixVariation, numEquilibriumConstraints, 0, actuationConstraintVariation, 0, 0,
                                 actuationConstraintVariation.getNumRows(), actuationConstraintVariation.getNumCols(), 1.0);

      /* Transform constraint matrix variation to rho-space */
      CommonOps_DDRM.mult(constraintMatrixVariation, stabilityMarginOptimizationModule.getRhoToForceTransformationMatrix(), solverConstraintVariation);

      constraintVarCalculatorSolverMathTimer.stopMeasurement();
      return solverConstraintVariation;
   }

   public void resetToInitialJointState()
   {
      /* Reset joint configuration to initial */
      MultiBodySystemTools.insertJointsState(controlledJoints, JointStateType.CONFIGURATION, initialJointConfiguration);

      fullRobotModel.updateFrames();
      wholeBodyContactState.updateActuationConstraintMatrix(false);
   }

   public DMatrixRMaj computeJacobianRate(DMatrixRMaj qd)
   {
      /* Integrate one preview tick and copy to robot model */
      MultiBodySystemTools.insertJointsState(controlledJoints, JointStateType.VELOCITY, qd);

      int numCoMVariables = 2;
      int numEquilibriumConstraints = 12;
      constraintMatrixVariation.reshape(actuationConstraintVariation.getNumRows() + numEquilibriumConstraints,
                                        actuationConstraintVariation.getNumCols() + numCoMVariables);
      MatrixTools.setMatrixBlock(constraintMatrixVariation, numEquilibriumConstraints, 0, actuationConstraintVariation, 0, 0,
                                 actuationConstraintVariation.getNumRows(), actuationConstraintVariation.getNumCols(), 1.0);

      /* Transform constraint matrix variation to rho-space */
      CommonOps_DDRM.mult(constraintMatrixVariation, stabilityMarginOptimizationModule.getRhoToForceTransformationMatrix(), solverConstraintVariation);

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
