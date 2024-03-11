package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.configurations.InertialEstimationParameters;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator.SpatialInertiaBasisOption;
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

public class InertialKalmanFilter extends ExtendedKalmanFilter implements OnlineInertialEstimator
{
   private static final int WRENCH_DIMENSION = 6;

   private final DMatrixRMaj identity;

   private final DMatrixRMaj torqueFromNominal;
   private final DMatrixRMaj torqueFromEstimates;
   private final DMatrixRMaj torqueFromContactWrenches;
   private final DMatrixRMaj torqueFromBias;

   private final DMatrixRMaj regressor;

   private final SideDependentList<DMatrixRMaj> contactJacobians = new SideDependentList<>();
   private final SideDependentList<DMatrixRMaj> contactWrenches = new SideDependentList<>();

   /** This is used as a container to build up a measurement from different contributions, see {@link #measurementModel(DMatrixRMaj)}. */
   private final DMatrixRMaj measurement;

   private final AlphaFilteredYoMatrix filteredResidual;

   public InertialKalmanFilter(FullRobotModel model, InertialEstimationParameters parameters, YoRegistry parentRegistry)
   {
      super(parameters.getURDFParameters(parameters.getBasisSets()),
            CommonOps_DDRM.identity(parameters.getNumberOfParameters()),
            CommonOps_DDRM.identity(parameters.getNumberOfParameters()),
            CommonOps_DDRM.identity(MultiBodySystemTools.computeDegreesOfFreedom(model.getRootJoint().subtreeArray())));

      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(model.getRootJoint().subtreeArray());
      Set<SpatialInertiaBasisOption>[] basisSets = parameters.getBasisSets();
      int[] partitionSizes = RegressorTools.sizePartitions(basisSets);

      identity = CommonOps_DDRM.identity(partitionSizes[0]);

      torqueFromNominal = new DMatrixRMaj(nDoFs, 1);
      torqueFromEstimates = new DMatrixRMaj(nDoFs, 1);
      torqueFromContactWrenches = new DMatrixRMaj(nDoFs, 1);
      torqueFromBias = new DMatrixRMaj(nDoFs, 1);

      regressor = new DMatrixRMaj(nDoFs, partitionSizes[0]);

      for (RobotSide side : RobotSide.values)
      {
         contactJacobians.put(side, new DMatrixRMaj(WRENCH_DIMENSION, nDoFs));
         contactWrenches.put(side, new DMatrixRMaj(WRENCH_DIMENSION, 1));
      }

      measurement = new DMatrixRMaj(nDoFs, 1);

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      // TODO: change
      double postProcessingAlpha = 0.0;
      filteredResidual = new AlphaFilteredYoMatrix("filteredResidual_", postProcessingAlpha, nDoFs, 1, getRowNames(model), null, registry);

      setNormalizedInnovationThreshold(parameters.getNormalizedInnovationThreshold());
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
      return regressor;
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
      // Torque from inverse dynamics on nominal model
      measurement.set(torqueFromNominal);

      // Torque from regressor on estimated parameters
      CommonOps_DDRM.mult(regressor, parametersForEstimate, torqueFromEstimates);
      CommonOps_DDRM.addEquals(measurement, torqueFromEstimates);

      // Torque from contact wrenches
      torqueFromContactWrenches.zero();
      for (RobotSide side : RobotSide.values)
      {
         // NOTE: the minus for the contact wrench contribution
         CommonOps_DDRM.multAddTransA(-1.0, contactJacobians.get(side), contactWrenches.get(side), torqueFromContactWrenches);
      }
      CommonOps_DDRM.addEquals(measurement, torqueFromContactWrenches);

      // Torque from bias
      CommonOps_DDRM.addEquals(measurement, torqueFromBias);

      return measurement;
   }

   @Override
   public void preUpdateHook()
   {
      filter(getMeasurementResidual(), filteredResidual);
      getMeasurementResidual().set(filteredResidual);
   }

   protected void filter(DMatrix matrixToFilter, AlphaFilteredYoMatrix filterContainer)
   {
      filterContainer.setAndSolve(matrixToFilter);
   }

   @Override
   public void setRegressor(DMatrix regressorForEstimates)
   {
      this.regressor.set(regressorForEstimates);
   }

   @Override
   public void setTorqueFromNominal(DMatrix torqueFromNominal)
   {
      this.torqueFromNominal.set(torqueFromNominal);
   }

   @Override
   public void setContactWrenches(SideDependentList<DMatrixRMaj> contactWrenches)
   {
      for (RobotSide side : RobotSide.values)
         this.contactWrenches.get(side).set(contactWrenches.get(side));
   }

   @Override
   public void setProcessCovariance(DMatrix processCovariance)
   {
      this.processCovariance.set(processCovariance);
   }

   @Override
   public void setMeasurementCovariance(DMatrix measurementCovariance)
   {
      this.measurementCovariance.set(measurementCovariance);
   }

   @Override
   public double getNormalizedInnovation()
   {
      return 0;
   }

   @Override
   public void setTorqueFromBias(DMatrix bias)
   {
      this.torqueFromBias.set(bias);
   }

   @Override
   public void setContactJacobians(SideDependentList<DMatrixRMaj> jacobians)
   {
      for (RobotSide side : RobotSide.values)
         contactJacobians.get(side).set(jacobians.get(side));
   }

   public void setPostProcessingAlpha(double postProcessingAlpha)
   {
      filteredResidual.setAlpha(postProcessingAlpha);
   }

   public DMatrixRMaj getProcessCovariance()
   {
      return processCovariance;
   }

   public void setProcessCovariance(DMatrixRMaj processCovariance)
   {
      this.processCovariance.set(processCovariance);
   }

   public DMatrixRMaj getMeasurementCovariance()
   {
      return measurementCovariance;
   }

   public void setMeasurementCovariance(DMatrixRMaj measurementCovariance)
   {
      this.measurementCovariance.set(measurementCovariance);
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
