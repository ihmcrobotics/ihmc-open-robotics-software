package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.configurations.InertialEstimationParameters;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.parameterEstimation.ExtendedKalmanFilter;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.math.filters.AlphaFilteredYoMatrix;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class InertialKalmanFilter extends ExtendedKalmanFilter
{
   protected static final boolean MORE_YOVARIABLES = true;

   private static final int WRENCH_DIMENSION = 6;

   protected final YoRegistry registry;

   private final DMatrixRMaj identity;

   protected final DMatrixRMaj torqueFromNominal;
   protected final DMatrixRMaj torqueFromEstimates;
   protected final DMatrixRMaj torqueFromContactWrenches;
   protected final DMatrixRMaj torqueFromBias;

   protected final DMatrixRMaj regressorForEstimates;

   protected final SideDependentList<DMatrixRMaj> contactJacobians = new SideDependentList<>();
   protected final SideDependentList<DMatrixRMaj> contactWrenches = new SideDependentList<>();

   /** This is used as a container to build up a measurement from different contributions, see {@link #measurementModel(DMatrixRMaj)}. */
   protected final DMatrixRMaj measurement;

   protected final AlphaFilteredYoMatrix filteredWholeSystemTorques;
   protected final AlphaFilteredYoMatrix doubleFilteredWholeSystemTorques;
   protected final AlphaFilteredYoMatrix filteredMeasurement;
   protected final AlphaFilteredYoMatrix doubleFilteredMeasurement;

   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   private final SideDependentList<RigidBodyReadOnly> feet;

   /** MORE_YOVARIABLES **/
   protected YoMatrix yoTorqueFromNominal = null;
   protected YoMatrix yoTorqueFromEstimates = null;
   protected YoMatrix yoTorqueFromContactWrenches = null;
   protected YoMatrix yoTorqueFromBias = null;

   private final FullRobotModel model;
   private final Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets;

   public InertialKalmanFilter(FullRobotModel model, Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets,
                               InertialEstimationParameters parameters,
                               DMatrixRMaj initialParametersForEstimate, DMatrixRMaj initialParameterCovariance,
                               DMatrixRMaj processCovariance, DMatrixRMaj measurementCovariance,
                               double postProcessingAlpha, YoRegistry parentRegistry)
   {
      super(initialParametersForEstimate, initialParameterCovariance, processCovariance, measurementCovariance);

      this.model = model;
      this.basisSets = basisSets;

      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(model.getRootJoint().subtreeArray());
      int[] partitionSizes = RegressorTools.sizePartitions(basisSets);

      identity = CommonOps_DDRM.identity(partitionSizes[0]);

      torqueFromNominal = new DMatrixRMaj(nDoFs, 1);
      torqueFromEstimates = new DMatrixRMaj(nDoFs, 1);
      torqueFromContactWrenches = new DMatrixRMaj(nDoFs, 1);
      torqueFromBias = new DMatrixRMaj(nDoFs, 1);

      regressorForEstimates = new DMatrixRMaj(nDoFs, partitionSizes[0]);

      for (RobotSide side : RobotSide.values)
      {
         contactJacobians.put(side, new DMatrixRMaj(WRENCH_DIMENSION, nDoFs));
         contactWrenches.put(side, new DMatrixRMaj(WRENCH_DIMENSION, 1));
      }

      measurement = new DMatrixRMaj(nDoFs, 1);

      registry = new YoRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      filteredWholeSystemTorques = new AlphaFilteredYoMatrix("filteredWholeSystemTorques", postProcessingAlpha, nDoFs, 1, getRowNames(model), null, registry);
      doubleFilteredWholeSystemTorques = new AlphaFilteredYoMatrix("doubleFilteredWholeSystemTorques", postProcessingAlpha, nDoFs, 1, getRowNames(model), null, registry);
      filteredMeasurement = new AlphaFilteredYoMatrix("filteredMeasurement", postProcessingAlpha, nDoFs, 1, getRowNames(model), null, registry);
      doubleFilteredMeasurement = new AlphaFilteredYoMatrix("doubleFilteredMeasurement", postProcessingAlpha, nDoFs, 1, getRowNames(model), null, registry);

      if (MORE_YOVARIABLES)
      {
         yoTorqueFromNominal = new YoMatrix("torqueFromNominal", nDoFs, 1, getRowNames(model), null, registry);
         yoTorqueFromEstimates = new YoMatrix("torqueFromEstimates", nDoFs, 1, getRowNames(model), null, registry);
         yoTorqueFromContactWrenches = new YoMatrix("torqueFromContactWrenches", nDoFs, 1, getRowNames(model), null, registry);
         yoTorqueFromBias = new YoMatrix("torqueFromBias", nDoFs, 1, getRowNames(model), null, registry);
      }

      inverseDynamicsCalculator = new InverseDynamicsCalculator(model.getElevator());
      inverseDynamicsCalculator.setGravitationalAcceleration(-9.81);
      feet = getRobotFeet(model);
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
      packEstimatedParametersIntoModel(parametersForEstimate);

      inverseDynamicsCalculator.compute();
      measurement.set(inverseDynamicsCalculator.getJointTauMatrix());

      // Torque from inverse dynamics on nominal model
//      measurement.set(torqueFromNominal);

      // Torque from regressor on estimated parameters
//      CommonOps_DDRM.mult(regressorForEstimates, parametersForEstimate, torqueFromEstimates);
//      CommonOps_DDRM.addEquals(measurement, torqueFromEstimates);

      // Torque from contact wrenches
//      torqueFromContactWrenches.zero();
//      for (RobotSide side : RobotSide.values)
//      {
//         // NOTE: the minus for the contact wrench contribution
//         CommonOps_DDRM.multAddTransA(-1.0, contactJacobians.get(side), contactWrenches.get(side), torqueFromContactWrenches);
//      }
//      CommonOps_DDRM.addEquals(measurement, torqueFromContactWrenches);

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
   public DMatrixRMaj calculateEstimate(DMatrix wholeSystemTorques)
   {
      filterTorques(wholeSystemTorques);
      return super.calculateEstimate(doubleFilteredWholeSystemTorques);
   }

   protected void filterTorques(DMatrix torques)
   {
      filter(torques, filteredWholeSystemTorques);
      filter(filteredWholeSystemTorques, doubleFilteredWholeSystemTorques);
   }

   protected void filter(DMatrix matrixToFilter, AlphaFilteredYoMatrix filterContainer)
   {
      filterContainer.setAndSolve(matrixToFilter);
   }

   public void setRegressor(DMatrixRMaj regressorForEstimates)
   {
      this.regressorForEstimates.set(regressorForEstimates);
   }

   public void setContactWrenches(SideDependentList<Wrench> contactWrenches)
   {
      for (RobotSide side : RobotSide.values)
         inverseDynamicsCalculator.setExternalWrench(feet.get(side), contactWrenches.get(side));
   }

   public void setBias(DMatrix bias)
   {
      this.torqueFromBias.set(bias);
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

   private void packEstimatedParametersIntoModel(DMatrixRMaj parameters)
   {
      RigidBodyBasics[] bodies = model.getRootBody().subtreeArray();

      int parameterIndex = 0;
      for (int i = 0; i < basisSets.length; ++i)
      {
         SpatialInertiaBasics inertia = bodies[i].getInertia();

         for (JointTorqueRegressorCalculator.SpatialInertiaBasisOption option : JointTorqueRegressorCalculator.SpatialInertiaBasisOption.values)
         {
            if (basisSets[i].contains(option))
            {
               double parameter = parameters.get(parameterIndex, 0);
               switch (option)
               {
                  case M -> inertia.setMass(parameter);
                  case MCOM_X -> inertia.getCenterOfMassOffset().setX(parameter);
                  case MCOM_Y -> inertia.getCenterOfMassOffset().setY(parameter);
                  case MCOM_Z -> inertia.getCenterOfMassOffset().setZ(parameter);
                  case I_XX -> inertia.getMomentOfInertia().setM00(parameter);
                  case I_XY -> inertia.getMomentOfInertia().setM01(parameter);
                  case I_XZ -> inertia.getMomentOfInertia().setM02(parameter);
                  case I_YY -> inertia.getMomentOfInertia().setM11(parameter);
                  case I_YZ -> inertia.getMomentOfInertia().setM12(parameter);
                  case I_ZZ -> inertia.getMomentOfInertia().setM22(parameter);
               }
               parameterIndex++;
            }
         }
      }
   }

   private SideDependentList<RigidBodyReadOnly> getRobotFeet(FullRobotModel model)
   {
      SideDependentList<RigidBodyReadOnly> feet = new SideDependentList<>();
      for (RigidBodyReadOnly body : model.getRootBody().subtreeArray())
      {
         if (body.getName().contains("LEFT_FOOT"))
            feet.put(RobotSide.LEFT, body);
         else if (body.getName().contains("RIGHT_FOOT"))
            feet.put(RobotSide.RIGHT, body);
      }

      return feet;
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
