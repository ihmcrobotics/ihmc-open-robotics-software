package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.LinearSolverSafe;
import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator.SpatialInertiaBasisOption;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.yoVariables.registry.YoRegistry;

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


   public InertialConstrainedKalmanFilter(FullRobotModel model,
                                          Set<SpatialInertiaBasisOption>[] basisSets,
                                          DMatrixRMaj initialParametersForEstimate,
                                          DMatrixRMaj initialParameterCovariance,
                                          DMatrixRMaj processCovariance,
                                          DMatrixRMaj measurementCovariance,
                                          double postProcessingAlpha,
                                          YoRegistry parentRegistry)
   {
      super(model,
            basisSets,
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
      qpSolver.setMaxNumberOfIterations(25);
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
      qpSolver.clear();  // TODO: see if you can remove this
      qpSolver.setQuadraticCostFunction(costMatrix, costVector, costScalar);
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
}
