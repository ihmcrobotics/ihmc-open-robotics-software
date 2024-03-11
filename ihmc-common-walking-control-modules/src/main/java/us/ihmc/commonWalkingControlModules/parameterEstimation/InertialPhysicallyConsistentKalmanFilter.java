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
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

class InertialPhysicallyConsistentKalmanFilter extends ExtendedKalmanFilter implements OnlineInertialEstimator
{
   private static final int WRENCH_DIMENSION = 6;

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

   private final DMatrixRMaj torqueFromNominal;

   private final DMatrixRMaj torqueFromContactWrenches;

   private final DMatrixRMaj torqueFromBias;

   private final DMatrixRMaj regressor;

   private final DMatrixRMaj measurementJacobian;

   private final DMatrixRMaj identity;

   private final DMatrixRMaj measurement;

   private final SideDependentList<DMatrixRMaj> contactJacobians = new SideDependentList<>();

   private final SideDependentList<DMatrixRMaj> contactWrenches = new SideDependentList<>();

   public InertialPhysicallyConsistentKalmanFilter(FullRobotModel model, InertialEstimationParameters parameters, YoRegistry parentRegistry)
   {
      super(parameters.getURDFParameters(parameters.getBasisSets()),
            CommonOps_DDRM.identity(parameters.getNumberOfParameters()),
            CommonOps_DDRM.identity(parameters.getNumberOfParameters()),
            CommonOps_DDRM.identity(MultiBodySystemTools.computeDegreesOfFreedom(model.getRootJoint().subtreeArray())));

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      Set<SpatialInertiaBasisOption>[] basisSets = parameters.getBasisSets();


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

      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(model.getRootJoint().subtreeArray());
      int[] partitionSizes = RegressorTools.sizePartitions(basisSets);

      for (RobotSide side : RobotSide.values)
      {
         contactJacobians.put(side, new DMatrixRMaj(WRENCH_DIMENSION, nDoFs));
         contactWrenches.put(side, new DMatrixRMaj(WRENCH_DIMENSION, 1));
      }

      torqueFromNominal = new DMatrixRMaj(nDoFs, 1);
      torqueFromContactWrenches = new DMatrixRMaj(nDoFs, 1);
      torqueFromBias = new DMatrixRMaj(nDoFs, 1);
      regressor = new DMatrixRMaj(nDoFs, RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
      measurementJacobian = new DMatrixRMaj(nDoFs, RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
      identity = CommonOps_DDRM.identity(partitionSizes[0]);
      measurement = new DMatrixRMaj(nDoFs, 1);

      measurementJacobianBlock = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                                 RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
      measurementJacobianContainer = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY * nBodies,
                                                     RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY * nBodies);

      regressorBlock = new DMatrixRMaj(measurementCovariance.getNumRows(), RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
      torqueContributionFromBody = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1);

      parameterThetaBasisContainer = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1);
      kalmanGainBlockContainer = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, measurementCovariance.getNumRows());

      setNormalizedInnovationThreshold(parameters.getNormalizedInnovationThreshold());
   }

   @Override
   public void preUpdateHook()
   {

   }

   @Override
   public void updateStep()
   {
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

   @Override
   protected DMatrixRMaj linearizeProcessModel(DMatrixRMaj previousState)
   {
      return identity;
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

   @Override
   protected DMatrixRMaj processModel(DMatrixRMaj state)
   {
      return state;
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
      return calculateNormalizedInnovation();
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
