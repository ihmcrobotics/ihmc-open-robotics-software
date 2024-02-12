package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.parameterEstimation.ExtendedKalmanFilter;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.math.filters.AlphaFilteredYoMatrix;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class InertialKalmanFilter extends ExtendedKalmanFilter
{
   private static final int WRENCH_DIMENSION = 6;

   private final DMatrixRMaj identity;

   private final DMatrixRMaj torqueFromNominal;
   private final DMatrixRMaj regressorForEstimates;

   private final SideDependentList<DMatrixRMaj> contactJacobians = new SideDependentList<>();
   private final SideDependentList<DMatrixRMaj> contactWrenches = new SideDependentList<>();

   /** This is used as a container to build up a measurement from different contributions, see {@link #measurementModel(DMatrixRMaj)}. */
   private final DMatrixRMaj measurement;

   private final AlphaFilteredYoMatrix filteredWholeSystemTorques;
   private final AlphaFilteredYoMatrix doubleFilteredWholeSystemTorques;
   private final AlphaFilteredYoMatrix filteredMeasurement;
   private final AlphaFilteredYoMatrix doubleFilteredMeasurement;

   private final DMatrixRMaj bias;


   public InertialKalmanFilter(FullRobotModel model, Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets,
                               DMatrixRMaj initialParametersForEstimate, DMatrixRMaj initialParameterCovariance,
                               DMatrixRMaj processCovariance, DMatrixRMaj measurementCovariance,
                               double postProcessingAlpha, YoRegistry parentRegistry)
   {
      super(initialParametersForEstimate, initialParameterCovariance, processCovariance, measurementCovariance);


      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(model.getRootJoint().subtreeArray());
      int[] partitionSizes = RegressorTools.sizePartitions(basisSets);

      identity = CommonOps_DDRM.identity(partitionSizes[0]);

      torqueFromNominal = new DMatrixRMaj(nDoFs, 1);

      regressorForEstimates = new DMatrixRMaj(nDoFs, partitionSizes[0]);

      for (RobotSide side : RobotSide.values)
      {
         contactJacobians.put(side, new DMatrixRMaj(WRENCH_DIMENSION, nDoFs));
         contactWrenches.put(side, new DMatrixRMaj(WRENCH_DIMENSION, 1));
      }

      measurement = new DMatrixRMaj(nDoFs, 1);

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      filteredWholeSystemTorques = new AlphaFilteredYoMatrix("filteredWholeSystemTorques", postProcessingAlpha, nDoFs, 1, getRowNames(model), null, registry);
      doubleFilteredWholeSystemTorques = new AlphaFilteredYoMatrix("doubleFilteredWholeSystemTorques", postProcessingAlpha, nDoFs, 1, getRowNames(model), null, registry);
      filteredMeasurement = new AlphaFilteredYoMatrix("filteredMeasurement", postProcessingAlpha, nDoFs, 1, getRowNames(model), null, registry);
      doubleFilteredMeasurement = new AlphaFilteredYoMatrix("doubleFilteredMeasurement", postProcessingAlpha, nDoFs, 1, getRowNames(model), null, registry);

      bias = new DMatrixRMaj(nDoFs, 1);
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
      measurement.set(torqueFromNominal);
      CommonOps_DDRM.multAdd(regressorForEstimates, parametersForEstimate, measurement);
      for (RobotSide side : RobotSide.values)
      {
         // NOTE: the minus for the contact wrench contribution
         CommonOps_DDRM.multAddTransA(-1.0, contactJacobians.get(side), contactWrenches.get(side), measurement);
      }

      CommonOps_DDRM.addEquals(measurement, bias);

      filter(measurement, filteredMeasurement);
      filter(filteredMeasurement, doubleFilteredMeasurement);
      measurement.set(doubleFilteredMeasurement);

      return measurement;
   }

   @Override
   public DMatrixRMaj calculateEstimate(DMatrix wholeSystemTorques)
   {
      filter(wholeSystemTorques, filteredWholeSystemTorques);
      filter(filteredWholeSystemTorques, doubleFilteredWholeSystemTorques);
      return super.calculateEstimate(doubleFilteredWholeSystemTorques);
   }

   public void filter(DMatrix matrixToFilter, AlphaFilteredYoMatrix filterContainer)
   {
      filterContainer.setAndSolve(matrixToFilter);
   }

   public void setRegressor(DMatrixRMaj regressorForEstimates)
   {
      this.regressorForEstimates.set(regressorForEstimates);
   }

   public void setTorqueFromNominal(DMatrixRMaj torqueFromNominal)
   {
      this.torqueFromNominal.set(torqueFromNominal);
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

   public void setBias(DMatrixRMaj bias)
   {
      this.bias.set(bias);
   }

   public void setPostProcessingAlpha(double postProcessingAlpha)
   {
      filteredWholeSystemTorques.setAlpha(postProcessingAlpha);
      doubleFilteredWholeSystemTorques.setAlpha(postProcessingAlpha);

      filteredMeasurement.setAlpha(postProcessingAlpha);
      doubleFilteredMeasurement.setAlpha(postProcessingAlpha);
   }

   public DMatrixRMaj getProcessCovariance()
   {
      return processCovariance;
   }

   public DMatrixRMaj getMeasurementCovariance()
   {
      return measurementCovariance;
   }

   private String[] getRowNames(FullRobotModel model)
   {
      List<String> names = new ArrayList<>();

      // Root joint is handled specially
      for (int i = 0; i < model.getRootJoint().getDegreesOfFreedom(); i++)
      {
         String suffix;
         switch(i)
         {
            case 0 -> suffix = "wX";
            case 1 -> suffix = "wY";
            case 2 -> suffix = "wZ";
            case 3 -> suffix = "x";
            case 4 -> suffix = "y";
            case 5 -> suffix = "z";
            default -> throw new RuntimeException("Unhandled case: " + i);
         }
         names.add(model.getRootJoint().getName() + "_" + suffix);
      }

      // One DoF joints
      OneDoFJointReadOnly[] oneDoFJoints = model.getOneDoFJoints();
      for (OneDoFJointReadOnly joint : oneDoFJoints)
      {
         names.add(joint.getName());
      }

      return names.toArray(new String[0]);
   }
}
