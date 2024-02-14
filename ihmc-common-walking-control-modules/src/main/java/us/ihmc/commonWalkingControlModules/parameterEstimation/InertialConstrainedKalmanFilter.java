package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.LinearSolverSafe;
import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import us.ihmc.commonWalkingControlModules.configurations.InertialEstimationParameters;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator.SpatialInertiaBasisOption;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Set;

public class InertialConstrainedKalmanFilter extends InertialKalmanFilter
{
   private final DMatrixRMaj inversePredictedCovariance;
   private final DMatrixRMaj inverseMeasurementCovariance;

   private final LinearSolverSafe<DMatrixRMaj> predictedCovarianceSolver;
   private final LinearSolverSafe<DMatrixRMaj> measurementCovarianceSolver;

   private final DMatrixRMaj costMatrix;
   private final DMatrixRMaj costMatrixContainer;
   private final DMatrixRMaj costVector;
   private final DMatrixRMaj costVectorContainer;
   private double costScalar;
   private final DMatrixRMaj costScalarContainer;

   private final SimpleEfficientActiveSetQPSolver qpSolver;

   private final DMatrixRMaj delta;

   private final DMatrixRMaj updatedCovarianceContainer1;
   private final DMatrixRMaj updatedCovarianceContainer2;
   private final DMatrixRMaj updatedCovarianceComponentInvertedContainer;
   private final LinearSolverSafe<DMatrixRMaj> updatedCovarianceSolver;


   Set<SpatialInertiaBasisOption>[] basisSets;

   private final YoDouble minimumMassMultiplier;
   private final YoDouble minimumDiagonalInertiaMultiplier;

   private final DMatrixRMaj urdfValues;
   private final SpatialInertiaBasisOption[] parameterOptions;

   private final DMatrixRMaj constraintMatrix;
   private final DMatrixRMaj constraintVector;

   public InertialConstrainedKalmanFilter(FullRobotModel model,
                                          Set<SpatialInertiaBasisOption>[] basisSets,
                                          InertialEstimationParameters parameters,
                                          DMatrixRMaj initialParametersForEstimate,
                                          DMatrixRMaj initialParameterCovariance,
                                          DMatrixRMaj processCovariance,
                                          DMatrixRMaj measurementCovariance,
                                          double postProcessingAlpha,
                                          YoRegistry parentRegistry)
   {
      super(model,
            basisSets,
            parameters,
            initialParametersForEstimate,
            initialParameterCovariance,
            processCovariance,
            measurementCovariance,
            postProcessingAlpha,
            parentRegistry);

      int processSize = processCovariance.getNumRows();
      int measurementSize = measurementCovariance.getNumRows();
      costMatrix = new DMatrixRMaj(processSize, processSize);
      costMatrixContainer = new DMatrixRMaj(processSize, processSize);
      costVector = new DMatrixRMaj(processSize, 1);
      costVectorContainer = new DMatrixRMaj(processSize, 1);
      costScalar = 0.0;
      costScalarContainer = new DMatrixRMaj(1, 1);

      predictedCovarianceSolver = new LinearSolverSafe<>(LinearSolverFactory_DDRM.symmPosDef(processSize));
      measurementCovarianceSolver = new LinearSolverSafe<>(LinearSolverFactory_DDRM.symmPosDef(measurementSize));

      inversePredictedCovariance = new DMatrixRMaj(processSize, processSize);
      inverseMeasurementCovariance = new DMatrixRMaj(measurementSize, measurementSize);

      delta = new DMatrixRMaj(processSize, 1);

      updatedCovarianceContainer1 = new DMatrixRMaj(processSize, processSize);
      updatedCovarianceContainer2 = new DMatrixRMaj(processSize, processSize);
      updatedCovarianceComponentInvertedContainer = new DMatrixRMaj(measurementSize, measurementSize);
      // even though the final result of the larger calculation is processSize, this inner inversion is measurementSize
      updatedCovarianceSolver = new LinearSolverSafe<>(LinearSolverFactory_DDRM.symmPosDef(measurementSize));

      qpSolver = new SimpleEfficientActiveSetQPSolver();
      qpSolver.setUseWarmStart(true);
      qpSolver.setMaxNumberOfIterations(parameters.getMaxNumberOfIterationsForQP());

      this.basisSets = basisSets;

      minimumMassMultiplier = new YoDouble("minimumMassMultiplier", registry);
      minimumMassMultiplier.set(parameters.getMinimumMassMultiplier());
      minimumDiagonalInertiaMultiplier = new YoDouble("minimumDiagonalInertiaMultiplier", registry);
      minimumDiagonalInertiaMultiplier.set(parameters.getMinimumDiagonalInertiaMultiplier());

      urdfValues = new DMatrixRMaj(processSize, 1);
      urdfValues.set(parameters.getURDFParameters(basisSets));
      parameterOptions = new SpatialInertiaBasisOption[processSize];
      int i = 0;
      for (Set<SpatialInertiaBasisOption> basisSet : basisSets)
      {
         for (SpatialInertiaBasisOption option : SpatialInertiaBasisOption.values)
         {
            if (basisSet.contains(option))
            {
               parameterOptions[i] = option;
               i++;
            }
         }
      }

      constraintMatrix = new DMatrixRMaj(processSize, processSize);
      constraintVector = new DMatrixRMaj(processSize, 1);
   }

   @Override
   public DMatrixRMaj calculateEstimate(DMatrix wholeSystemTorques)
   {
      filterTorques(wholeSystemTorques);
      predictionStep();
      updateStep(doubleFilteredWholeSystemTorques);
      cleanupStep();

      return updatedState;
   }

   @Override
   public void updateStep(DMatrix actual)
   {
      calculateMeasurementResidual(actual);

      measurementJacobian.set(linearizeMeasurementModel(predictedState));  // NOTE: measurement model linearization occurs at x_(k|k-1)

      calculateResidualCovarianceAndInverse();

      // Invert the predicted covariance and measurement covariances for use in the cost function of a QP
      predictedCovarianceSolver.setA(predictedCovariance);
      predictedCovarianceSolver.invert(inversePredictedCovariance);
      measurementCovarianceSolver.setA(measurementCovariance);
      measurementCovarianceSolver.invert(inverseMeasurementCovariance);

      // Build cost matrix
      costMatrix.zero();
      costMatrixContainer.zero();
      CommonOps_DDRM.multTransA(measurementJacobian, inverseMeasurementCovariance, costMatrixContainer);
      CommonOps_DDRM.mult(costMatrixContainer, measurementJacobian, costMatrix);
      CommonOps_DDRM.addEquals(costMatrix, inversePredictedCovariance);
      CommonOps_DDRM.scale(2.0, costMatrix); // TODO: might not need the 2

      // Build cost vector
      costVector.zero();
      costVector.zero();
      CommonOps_DDRM.multTransA(measurementJacobian, inverseMeasurementCovariance, costVectorContainer);
      CommonOps_DDRM.mult(costVectorContainer, measurementResidual, costVector);
      CommonOps_DDRM.scale(-2.0, costVector); // TODO: might not need the 2

      // Build cost scalar
      costScalarContainer.zero();
      CommonOps_DDRM.multTransA(measurementResidual, inverseMeasurementCovariance, costScalarContainer);
      costScalar = CommonOps_DDRM.dot(costScalarContainer, measurementResidual);

      // Solve QP
      qpSolver.setQuadraticCostFunction(costMatrix, costVector, costScalar);
      populateConstraints();
      qpSolver.solve(delta);

      // The QP only gives us the delta of the solution, we need to add it to the predicted state to get the updated state
      CommonOps_DDRM.add(predictedState, delta, updatedState);

      calculateUpdatedCovariance();
   }

   @Override
   protected final void calculateUpdatedCovariance()
   {
      // Following covariance update in Varin paper
      CommonOps_DDRM.mult(measurementJacobian, predictedCovariance, updatedCovarianceContainer1);
      CommonOps_DDRM.multTransB(updatedCovarianceContainer1, measurementJacobian, updatedCovarianceContainer2);
      CommonOps_DDRM.addEquals(updatedCovarianceContainer2, measurementCovariance);
      updatedCovarianceSolver.setA(updatedCovarianceContainer2);
      updatedCovarianceSolver.invert(updatedCovarianceComponentInvertedContainer);
      // Terms on the right
      CommonOps_DDRM.mult(updatedCovarianceComponentInvertedContainer, measurementJacobian, updatedCovarianceContainer1);
      CommonOps_DDRM.mult(updatedCovarianceContainer1, predictedCovariance, updatedCovarianceContainer2);
      // Terms on the left
      CommonOps_DDRM.multTransA(measurementJacobian, updatedCovarianceContainer2, updatedCovarianceContainer1);
      CommonOps_DDRM.mult(predictedCovariance, updatedCovarianceContainer1, updatedCovarianceContainer2);
      CommonOps_DDRM.scale(-1.0, updatedCovarianceContainer2);
      CommonOps_DDRM.add(updatedCovarianceContainer2, predictedCovariance, updatedCovariance);
   }


   private void populateConstraints()
   {
      constraintVector.set(urdfValues);
      for (int i = 0; i < constraintVector.getNumElements(); ++i)
      {
         switch(parameterOptions[i])
         {
            case M ->
            {
               constraintMatrix.set(i, i, 1.0);
               constraintVector.set(i, 0, constraintVector.get(i, 0) * minimumMassMultiplier.getValue());
            }
            case I_XX, I_YY, I_ZZ ->
            {
               constraintMatrix.set(i, i, 1.0);
               constraintVector.set(i, 0, constraintVector.get(i, 0) * minimumDiagonalInertiaMultiplier.getValue());
            }
            case MCOM_X, MCOM_Y, MCOM_Z, I_XY, I_XZ, I_YZ ->
            {
               // For now, we do nothing to constrain these parameters, so constraint matrix and constraint vector terms are both zero
               constraintMatrix.set(i, i, 0.0);
               constraintVector.set(i, 0);
            }
         }
      }

      // Because the solution of the constrained QP is the parameter deltas from the predicted state, we need to subtract the predicted state
      CommonOps_DDRM.subtractEquals(constraintVector, predictedState);

      // We've so far set up things for Cx > d, we need to scale both sides by -1 to flip the inequality
      CommonOps_DDRM.scale(-1.0, constraintMatrix);
      CommonOps_DDRM.scale(-1.0, constraintVector);

      qpSolver.setLinearInequalityConstraints(constraintMatrix, constraintVector);
   }
}
