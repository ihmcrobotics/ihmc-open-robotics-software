package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.configurations.InertialEstimationParameters;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator.SpatialInertiaBasisOption;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParameters;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParametersTools;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

class InertialPhysicallyConsistentKalmanFilter extends InertialKalmanFilter
{
   private final List<RigidBodyInertialParameters> inertialParameters = new ArrayList<>();
   private final List<YoMatrix> inertialParametersPiBasisWatchers = new ArrayList<>();
   private final List<YoMatrix> inertialParametersThetaBasisWatchers = new ArrayList<>();

   private final DMatrixRMaj measurementJacobianBlock;
   private final DMatrixRMaj measurementJacobianContainer;

   private final DMatrixRMaj regressorBlock;
   private final DMatrixRMaj torqueContributionFromBody;

   private final DMatrixRMaj parameterThetaBasisContainer;
   private final DMatrixRMaj kalmanGainBlockContainer;

   private int nBodies;

   public InertialPhysicallyConsistentKalmanFilter(FullRobotModel model,
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

      nBodies = 0;
      RigidBodyReadOnly[] modelBodies = model.getRootBody().subtreeArray();
      for (int i = 0; i < modelBodies.length; ++i)
      {
         if (!basisSets[i].isEmpty())
         {
            if (basisSets[i].size() != RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY)
            {
               throw new RuntimeException("This filter requires the full body to be estimated, not just certain parameters in the body.");
            }
            else
            {
               inertialParameters.add(new RigidBodyInertialParameters(modelBodies[i].getInertia()));
               inertialParametersPiBasisWatchers.add(new YoMatrix("inertialParametersPiBasis" + modelBodies[i].getName(), 10, 1, RigidBodyInertialParametersTools.getNamesForPiBasis(), null, registry));
               inertialParametersThetaBasisWatchers.add(new YoMatrix("inertialParametersThetaBasis" + modelBodies[i].getName(), 10, 1, RigidBodyInertialParametersTools.getNamesForThetaBasis(), null, registry));
               nBodies++;
            }
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
   }

   @Override
   protected void updateStep(DMatrix actual)
   {
      calculateMeasurementResidual(actual);

      measurementJacobian.set(linearizeMeasurementModel(predictedState));  // NOTE: measurement model linearization occurs at x_(k|k-1)

      calculateResidualCovarianceAndInverse();

      calculateKalmanGain();

      // Here, we deviate from the supeclass and add the Kalman gain on bodywise, because we only have access to the Jacobian of the measurement model
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
         CommonOps_DDRM.multAdd(kalmanGainBlockContainer, measurementResidual, parameterThetaBasisContainer);
      }

      gateMeasurementWithNormalizationThreshold();

      // Finally -- not relevant to the algorithm, but let's do a physical consistency check on the updated parameters
      checkPhysicalConsistency();
   }

   @Override
   public void gateMeasurementWithNormalizationThreshold()
   {
      if (calculateNormalizedInnovation() > normalizedInnovationThreshold)
      {
         // If the normalized innovation is too large, the measurement is likely an outlier. Do not update the state.
         updatedState.set(predictedState);
         updatedCovariance.set(predictedCovariance);
      }
      else
      {
         for (int i = 0; i < nBodies; ++i)
         {
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
      }
   }

   private void checkPhysicalConsistency()
   {
      for (int i = 0; i < nBodies; ++i)
      {
         if (!RigidBodyInertialParametersTools.isFullyPhysicallyConsistent(inertialParameters.get(i)))
         {
            LogTools.error("Inertial parameters are not fully physically consistent");
            if (!RigidBodyInertialParametersTools.isPhysicallyConsistent(inertialParameters.get(i)))
            {
               LogTools.error("Inertial parameters are not physically consistent");
            }
         }
      }
   }

   @Override
   public DMatrixRMaj calculateEstimate(DMatrix wholeSystemTorques)
   {
      filterTorques(wholeSystemTorques);
      predictionStep();
      updateStep(doubleFilteredWholeSystemTorques);
      cleanupStep();

      updateWatchers();

      return updatedState;
   }

   @Override
   protected DMatrixRMaj measurementModel(DMatrixRMaj parametersForEstimate)
   {
      // Torque from inverse dynamics on nominal model
      measurement.set(torqueFromNominal);

      // Torque from regressor on estimated parameters
      for (int i = 0; i < nBodies; ++i)
      {
         // Pack regressor block
         MatrixMissingTools.setMatrixBlock(regressorBlock,
                                           0,
                                           0,
                                           regressorForEstimates,
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

      filter(measurement, filteredMeasurement);
      filter(filteredMeasurement, doubleFilteredMeasurement);
      measurement.set(doubleFilteredMeasurement);

      if (MORE_YOVARIABLES)
      {
         yoTorqueFromNominal.set(torqueFromNominal);
         yoTorqueFromEstimates.set(torqueFromEstimates);
         yoTorqueFromContactWrenches.set(torqueFromContactWrenches);
         yoTorqueFromBias.set(torqueFromBias);
      }

      return measurement;
   }

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
      CommonOps_DDRM.mult(regressorForEstimates, measurementJacobianContainer, measurementJacobian);
      return measurementJacobian;
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
