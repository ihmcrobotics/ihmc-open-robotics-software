package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.parameterEstimation.ExtendedKalmanFilter;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.math.filters.AlphaFilteredYoMatrix;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Set;

public class InertialKalmanFilter extends ExtendedKalmanFilter
{
   private static final int WRENCH_DIMENSION = 6;
private static final boolean POST_PROCESS = true;

   private final DMatrixRMaj identity;

   private final DMatrixRMaj regressorForEstimates;
   private final DMatrixRMaj regressorForNominal;

   private final DMatrixRMaj parametersForNominal;

   private final SideDependentList<DMatrixRMaj> contactJacobians = new SideDependentList<>();
   private final SideDependentList<DMatrixRMaj> contactWrenches = new SideDependentList<>();

   /** This is used as a container to build up a measurement from different contributions, see {@link #measurementModel(DMatrixRMaj)}. */
   private final DMatrixRMaj measurement;

   private AlphaFilteredYoMatrix filteredWholeSystemTorques = null;
   private AlphaFilteredYoMatrix doubleFilteredWholeSystemTorques = null;
   private AlphaFilteredYoMatrix filteredMeasurement = null;
   private AlphaFilteredYoMatrix doubleFilteredMeasurement = null;



   public InertialKalmanFilter(FullRobotModel model, Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets,
                               DMatrixRMaj initialParametersForEstimate, DMatrixRMaj initialParameterCovariance,
                               DMatrixRMaj processCovariance, DMatrixRMaj measurementCovariance,
                               double postProcessingAlpha, YoRegistry parentRegistry)
   {
      super(initialParametersForEstimate, initialParameterCovariance, processCovariance, measurementCovariance);


      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(model.getRootJoint().subtreeArray());
      int[] partitionSizes = RegressorTools.sizePartitions(basisSets);

      identity = CommonOps_DDRM.identity(partitionSizes[0]);

      regressorForEstimates = new DMatrixRMaj(nDoFs, partitionSizes[0]);
      regressorForNominal = new DMatrixRMaj(nDoFs, partitionSizes[1]);

      parametersForNominal = new DMatrixRMaj(partitionSizes[1], 1);

      for (RobotSide side : RobotSide.values)
      {
         contactJacobians.put(side, new DMatrixRMaj(WRENCH_DIMENSION, nDoFs));
         contactWrenches.put(side, new DMatrixRMaj(WRENCH_DIMENSION, 1));
      }

      measurement = new DMatrixRMaj(nDoFs, 1);

      if (POST_PROCESS)
      {
         YoRegistry registry = new YoRegistry(getClass().getSimpleName());
         parentRegistry.addChild(registry);
         filteredWholeSystemTorques = new AlphaFilteredYoMatrix("filteredWholeSystemTorques", postProcessingAlpha, nDoFs, 1, registry);
         doubleFilteredWholeSystemTorques = new AlphaFilteredYoMatrix("doubleFilteredWholeSystemTorques", postProcessingAlpha, nDoFs, 1, registry);
         filteredMeasurement = new AlphaFilteredYoMatrix("filteredMeasurement", postProcessingAlpha, nDoFs, 1, registry);
         doubleFilteredMeasurement = new AlphaFilteredYoMatrix("doubleFilteredMeasurement", postProcessingAlpha, nDoFs, 1, registry);
      }
   }

   /** For inertial parameters, the process Jacobian is the identity matrix. */
   @Override
   protected DMatrixRMaj linearizeProcessModel(DMatrixRMaj previousParametersToEstimate)
   {
      return identity;
   }

   /** For inertial parameters, the measurement Jacobian is the regressor corresponding to the parameters one is estimating. */
   @Override
   protected DMatrixRMaj linearizeMeasurementModel(DMatrixRMaj predictedParametersToEstimate)
   {
      return regressorForEstimates;
   }

   /** For inertial parameters, the process model is the identity mapping -- we assume that the parameters are constant. */
   @Override
   protected DMatrixRMaj processModel(DMatrixRMaj parametersToEstimate)
   {
      return parametersToEstimate;
   }

   /** For inertial parameters, the measurement model is the sum of:
    * <li> the regressor contribution from the parameters that are being estimated
    * <li> the regressor contribution from the nominal parameters
    * <li> the contribution from the contact wrenches mapped through the contact Jacobians
    */
   @Override
   protected DMatrixRMaj measurementModel(DMatrixRMaj parametersForEstimate)
   {
      CommonOps_DDRM.mult(regressorForEstimates, parametersForEstimate, measurement);
      CommonOps_DDRM.multAdd(regressorForNominal, parametersForNominal, measurement);
      for (RobotSide side : RobotSide.values)
      {
         // NOTE: the minus for the contact wrench contribution
         CommonOps_DDRM.multAddTransA(-1.0, contactJacobians.get(side), contactWrenches.get(side), measurement);
      }

      if (POST_PROCESS)
      {
         filter(measurement, filteredMeasurement);
         filter(filteredMeasurement, doubleFilteredMeasurement);
         measurement.set(doubleFilteredMeasurement);
      }

      return measurement;
   }

   @Override
   public DMatrixRMaj calculateEstimate(DMatrix wholeSystemTorques)
   {
      if (POST_PROCESS)
      {
         filter(wholeSystemTorques, filteredWholeSystemTorques);
         filter(filteredWholeSystemTorques, doubleFilteredWholeSystemTorques);
         return super.calculateEstimate(doubleFilteredWholeSystemTorques);
      }
      else
      {
         return super.calculateEstimate(wholeSystemTorques);
      }
   }

   public void filter(DMatrix matrixToFilter, AlphaFilteredYoMatrix filterContainer)
   {
      filterContainer.setAndSolve(matrixToFilter);
   }

   public void setRegressorForEstimates(DMatrixRMaj regressorForEstimates)
   {
      this.regressorForEstimates.set(regressorForEstimates);
   }

   public void setRegressorForNominal(DMatrixRMaj regressorForNominal)
   {
      this.regressorForNominal.set(regressorForNominal);
   }

   public void setParametersForNominal(DMatrixRMaj parametersForNominal)
   {
      this.parametersForNominal.set(parametersForNominal);
   }

   public void setContactJacobians(SideDependentList<DMatrixRMaj> contactJacobians)
   {
      for (RobotSide side : RobotSide.values)
      {
         this.contactJacobians.get(side).set(contactJacobians.get(side));
      }
   }

   public void setContactWrenches(SideDependentList<DMatrixRMaj> contactWrenches)
   {
      for (RobotSide side : RobotSide.values)
      {
         this.contactWrenches.get(side).set(contactWrenches.get(side));
      }
   }

   public DMatrixRMaj getProcessCovariance()
   {
      return processCovariance;
   }

   public DMatrixRMaj getMeasurementCovariance()
   {
      return measurementCovariance;
   }
}
