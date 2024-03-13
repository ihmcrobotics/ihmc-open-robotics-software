package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.configurations.InertialEstimationParameters;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator.SpatialInertiaBasisOption;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.parameterEstimation.ExtendedKalmanFilter;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParameters;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParametersTools;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.robotics.math.filters.AlphaFilteredYoMatrix;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * An implementation of the {@link ExtendedKalmanFilter} for estimating inertial parameters of a robot in a physically-consistent manner (no negative masses or
 * invalid inertia matrices).
 * <p>
 * Features of the filter:
 * <li> The process model of the filter is how the inertial parameters are expected to change over time. As we cannot predict this, we consider the inertial
 * parameters to be constant and therefore process model is the identity mapping.
 * <li> The measurement of the filter is the torque contribution from the inertial parameters being estimated. We form a model of this by totaling up the other
 * torque contributions: the contribution from the current estimate of the inertial parameters, the contribution from the nominal inertial parameters that are
 * considered known and fixed, the contribution from the contact wrenches, and the contribution from a bias that approximates model mismatch.
 * <li> The measurement model uses a physically-consistent parameterization of rigid bodies that ensures that: mass is always positive, moment of inertia is
 * always positive definite, and the principal moments of inertia satisfy a triangle inequality.
 * </p>
 *
 * @author James Foster
 */
class InertialPhysicallyConsistentKalmanFilter extends ExtendedKalmanFilter implements OnlineInertialEstimator
{
   private static final int WRENCH_DIMENSION = 6;
   private final DMatrixRMaj IDENTITY;

   private final DMatrixRMaj torqueFromNominal;
   private final DMatrixRMaj torqueFromContactWrenches;
   private final DMatrixRMaj torqueFromBias;

   private final DMatrixRMaj regressor;

   private final SideDependentList<DMatrixRMaj> contactJacobians = new SideDependentList<>();
   private final SideDependentList<DMatrixRMaj> contactWrenches = new SideDependentList<>();

   private int nBodies;
   private final List<RigidBodyInertialParameters> inertialParameters = new ArrayList<>();
   private final List<YoMatrix> inertialParametersPiBasisWatchers = new ArrayList<>();
   private final List<YoMatrix> inertialParametersThetaBasisWatchers = new ArrayList<>();

   private final DMatrixRMaj measurementJacobianBlock;
   private final DMatrixRMaj measurementJacobianContainer;

   private final DMatrixRMaj regressorBlock;
   private final DMatrixRMaj torqueContributionFromBody;

   private final DMatrixRMaj parameterThetaBasisContainer;

   private final DMatrixRMaj kalmanGainBlockContainer;

   private final DMatrixRMaj measurement;

   private final AlphaFilteredYoMatrix filteredResidual;

   public InertialPhysicallyConsistentKalmanFilter(FullRobotModel model, InertialEstimationParameters parameters, double dt, YoRegistry parentRegistry)
   {
      super(parameters.getURDFParameters(parameters.getBasisSets()),
            CommonOps_DDRM.identity(parameters.getNumberOfParameters()),
            CommonOps_DDRM.identity(parameters.getNumberOfParameters()),
            CommonOps_DDRM.identity(MultiBodySystemTools.computeDegreesOfFreedom(model.getRootJoint().subtreeArray())));

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(model.getRootJoint().subtreeArray());
      Set<SpatialInertiaBasisOption>[] basisSets = parameters.getBasisSets();
      int[] partitionSizes = RegressorTools.sizePartitions(basisSets);

      IDENTITY = CommonOps_DDRM.identity(partitionSizes[0]);

      torqueFromNominal = new DMatrixRMaj(nDoFs, 1);
      torqueFromContactWrenches = new DMatrixRMaj(nDoFs, 1);
      torqueFromBias = new DMatrixRMaj(nDoFs, 1);

      regressor = new DMatrixRMaj(nDoFs, RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);

      for (RobotSide side : RobotSide.values)
      {
         contactJacobians.put(side, new DMatrixRMaj(WRENCH_DIMENSION, nDoFs));
         contactWrenches.put(side, new DMatrixRMaj(WRENCH_DIMENSION, 1));
      }

      nBodies = 0;
      RigidBodyReadOnly[] modelBodies = model.getRootBody().subtreeArray();
      for (int i = 0; i < modelBodies.length; ++i)
      {
         if (basisSets[i].isEmpty())
            continue;

         if (basisSets[i].size() != RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY)
         {
            throw new RuntimeException("This filter requires the full body to be estimated, not just certain parameters in the body.");
         }
         else
         {
            inertialParameters.add(new RigidBodyInertialParameters(modelBodies[i].getInertia()));
            inertialParametersPiBasisWatchers.add(new YoMatrix("pi_" + modelBodies[i].getName() + "_", 10, 1, RigidBodyInertialParametersTools.getNamesForPiBasis(), null, registry));
            inertialParametersThetaBasisWatchers.add(new YoMatrix("theta_" + modelBodies[i].getName() + "_", 10, 1, RigidBodyInertialParametersTools.getNamesForThetaBasis(), null, registry));
            nBodies++;
         }
      }

      measurementJacobianBlock = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                                 RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
      measurementJacobianContainer = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY * nBodies,
                                                     RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY * nBodies);

      regressorBlock = new DMatrixRMaj(measurementCovariance.getNumRows(), RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
      torqueContributionFromBody = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1);

      parameterThetaBasisContainer = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1);
      kalmanGainBlockContainer = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, measurementCovariance.getNumRows());

      measurement = new DMatrixRMaj(nDoFs, 1);

      double postProcessingAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForPostProcessing(), dt);
      filteredResidual = new AlphaFilteredYoMatrix("filteredResidual_", postProcessingAlpha, nDoFs, 1, parameters.getMeasurementNames(), null, registry);

      setNormalizedInnovationThreshold(parameters.getNormalizedInnovationThreshold());
   }

   /** For inertial parameters, the process model is the identity mapping -- we assume that the parameters are constant. */
   @Override
   protected DMatrixRMaj processModel(DMatrixRMaj parametersToEstimate)
   {
      return parametersToEstimate;
   }

   /**
    * For inertial parameters, the measurement model is the sum of:
    * <li> the regressor contribution from the parameters that are being estimated
    * <li> the regressor contribution from the nominal parameters
    * <li> the contribution from the contact wrenches mapped through the contact Jacobians
    */
   @Override
   protected DMatrixRMaj measurementModel(DMatrixRMaj parametersForEstimate)
   {
      // Torque from inverse dynamics on nominal model
      measurement.set(torqueFromNominal);

      // Torque from regressor on estimated parameters -- we do this bodywise because the physically-consistent parameterization is a bodywise mapping
      for (int i = 0; i < nBodies; ++i)
      {
         // Pack regressor block
         MatrixMissingTools.setMatrixBlock(regressorBlock,
                                           0,
                                           0,
                                           regressor,
                                           0,
                                           i * regressorBlock.getNumCols(),
                                           regressorBlock.getNumRows(),
                                           regressorBlock.getNumCols(),
                                           1.0);
         CommonOps_DDRM.mult(regressorBlock, inertialParameters.get(i).getParameterVectorPiBasis(), torqueContributionFromBody);
         CommonOps_DDRM.addEquals(measurement, torqueContributionFromBody);
      }

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

   /** For inertial parameters, the process Jacobian is the identity matrix. */
   @Override
   protected DMatrixRMaj linearizeProcessModel(DMatrixRMaj previousParametersToEstimate)
   {
      return IDENTITY;
   }

   /** The measurement Jacobian for the physically-consistent parameterization is the regressor multiplied by the mapping Jacobian. */
   @Override
   protected DMatrixRMaj linearizeMeasurementModel(DMatrixRMaj predictedState)
   {
      for (int i = 0; i < nBodies; ++i)
      {
         // Set the relevant block of the process jacobian
         RigidBodyInertialParameters.fromThetaBasisToPiBasisJacobian(inertialParameters.get(i).getParameterVectorThetaBasis(), measurementJacobianBlock);
         MatrixMissingTools.setMatrixBlock(measurementJacobianContainer,
                                           i * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                           i * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                           measurementJacobianBlock,
                                           0,
                                           0,
                                           RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                           RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                           1.0);
      }

      // Jacobian is Y * G(theta)
      CommonOps_DDRM.mult(regressor, measurementJacobianContainer, measurementJacobian);
      return measurementJacobian;
   }

   /**
    * The default update step of the extended Kalman filter is overwritten to account for the physically-consistent parameterization, which can only be
    * performed bodywise instead of the usual linear algebra approach of one big matrix.
    */
   @Override
   public void updateStep()
   {
      calculateKalmanGain();

      // Here, we deviate from the superclass and add the Kalman gain on bodywise, because we only have access to the Jacobian of the measurement model
      // in that way
      for (int i = 0; i < nBodies; ++i)
      {
         // For each body, get the Kalman gain block corresponding to it, and update the theta basis parameter vector with the residual
         MatrixMissingTools.setMatrixBlock(kalmanGainBlockContainer,
                                           0,
                                           0,
                                           kalmanGain,
                                           i * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                           0,
                                           RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                           measurementCovariance.getNumRows(),
                                           1.0);
         parameterThetaBasisContainer.set(inertialParameters.get(i).getParameterVectorThetaBasis());
         CommonOps_DDRM.multAdd(kalmanGainBlockContainer, getMeasurementResidual(), parameterThetaBasisContainer);

         // Update inertial parameters
         inertialParameters.get(i).setParameterVectorThetaBasis(parameterThetaBasisContainer);
         inertialParameters.get(i).update();

         // Pack updated pi basis into state for the next iteration
         MatrixMissingTools.setMatrixRows(updatedState,
                                          i * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                          inertialParameters.get(i).getParameterVectorPiBasis(),
                                          0,
                                          RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
      }
      calculateUpdatedCovariance();

      state.set(updatedState);
      covariance.set(updatedCovariance);
   }

   /** Before the update step, low-pass filter the measurement residual to reduce the effect of contact impulses on acceleration, wrench, and joint torques. */
   @Override
   public void preUpdateHook()
   {
      filter(getMeasurementResidual(), filteredResidual);
      getMeasurementResidual().set(filteredResidual);
   }

   /** Post solve, update the watchers for the inertial parameters. */
   @Override
   public void postSolveHook()
   {
      updateWatchers();
   }

   @Override
   public void setRegressor(DMatrix regressor)
   {
      this.regressor.set(regressor);
   }

   @Override
   public void setTorqueFromNominal(DMatrix torqueFromNominal)
   {
      this.torqueFromNominal.set(torqueFromNominal);
   }

   @Override
   public void setTorqueFromBias(DMatrix torqueFromBias)
   {
      this.torqueFromBias.set(torqueFromBias);
   }

   @Override
   public void setContactJacobians(SideDependentList<DMatrixRMaj> jacobians)
   {
      for (RobotSide side : RobotSide.values)
         contactJacobians.get(side).set(jacobians.get(side));
   }

   @Override
   public void setContactWrenches(SideDependentList<DMatrixRMaj> wrenches)
   {
      for (RobotSide side : RobotSide.values)
         contactWrenches.get(side).set(wrenches.get(side));
   }

   private void filter(DMatrix matrixToFilter, AlphaFilteredYoMatrix filterContainer)
   {
      filterContainer.setAndSolve(matrixToFilter);
   }

   private void updateWatchers()
   {
      for (int i = 0; i < nBodies; ++i)
      {
         inertialParametersPiBasisWatchers.get(i).set(inertialParameters.get(i).getParameterVectorPiBasis());
         inertialParametersThetaBasisWatchers.get(i).set(inertialParameters.get(i).getParameterVectorThetaBasis());
      }
   }
}
