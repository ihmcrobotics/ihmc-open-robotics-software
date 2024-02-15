package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.configurations.InertialEstimationParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.InertialParameterManagerFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator.SpatialInertiaBasisOption;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParameters;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParametersTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelWrapper;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.math.filters.*;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.*;

public class InertialParameterManager implements SCS2YoGraphicHolder
{
   private static final int WRENCH_DIMENSION = 6;

   private final YoBoolean enableFilter;
   private final YoBoolean resetFilter;
   private final YoBoolean resetConfig;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final int nDoFs;
   private final int nParameters;

   private final FullHumanoidRobotModel actualRobotModel;
   private final RigidBodyBasics[] actualModelBodies;
   private final List<? extends JointBasics> actualModelJoints;

   private final FullHumanoidRobotModel estimateRobotModel;
   private final RigidBodyBasics[] estimateModelBodies;
   private final ArrayList<YoInertiaEllipsoid> yoInertiaEllipsoids;
   private final YoGraphicDefinition ellipsoidGraphicGroup;

   private final FullHumanoidRobotModel inverseDynamicsRobotModel;
   private final List<? extends JointBasics> inverseDynamicsModelJoints;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   private final FullHumanoidRobotModel regressorRobotModel;
   private final List<? extends JointBasics> regressorModelJoints;
   private final JointTorqueRegressorCalculator regressorCalculator;

   private final SideDependentList<? extends FootSwitchInterface> footSwitches;
   private final SideDependentList<DMatrix> contactWrenches;

   private final JointIndexHandler jointIndexHandler;
   private final SideDependentList<JointBasics[]> legJoints;
   private final SideDependentList<GeometricJacobian> compactContactJacobians;
   private final SideDependentList<DMatrixRMaj> fullContactJacobians;

   private final DMatrixRMaj wholeSystemTorques;

   private final DMatrixRMaj rootJointVelocity;
   private final YoDouble[] rootJointVelocities;
   private final FilteredVelocityYoVariable[] rootJointAccelerations;
   private final DMatrixRMaj rootJointAcceleration;

   private final YoDouble[] jointVelocities;
   private final FilteredVelocityYoVariable[] jointAccelerations;

   private final Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets;
   private final RigidBodyBasics[] regressorModelBodiesToProcess;
   private final DMatrixRMaj[] regressorBlocks;
   private final DMatrixRMaj regressor;

   private InertialKalmanFilter filter;
   private final YoMatrix estimate;

   private final AlphaFilteredYoMatrix filteredEstimate;
   private final AlphaFilteredYoMatrix doubleFilteredEstimate;

   /** We specify the process covariances for every parameter in a rigid body, then use the same for all bodies */
   private YoDouble[] processCovariancesForSingleBody;
   private final YoDouble floatingBaseMeasurementCovariance;
   private final YoDouble legsMeasurementCovariance;
   private final YoDouble armsMeasurementCovariance;
   private final YoDouble spineMeasurementCovariance;

   private final YoBoolean reduceProcessCovarianceWhileWalking;
   private final YoDouble processCovarianceWalkingMultiplier;
   private final YoMatrix processCovariancePassedToFilter;

   private final YoMatrix residual;

   private final YoBoolean calculateBias;
   private final InertialBiasCompensator biasCompensator;
   private final YoMatrix bias;
   private final YoBoolean excludeBias;
   private final YoBoolean eraseBias;

   private final InertialEstimationParameters parameters;
   private final double postProcessingAlpha;
   private final double estimateFilteringAlpha;
   private final double accelerationCalculationAlpha;

   private final YoDouble normalizedInnovation;
   private final YoDouble normalizedInnovationThreshold;

   private final ExecutionTimer regressorTimer = new ExecutionTimer("RegressorTimer", registry);

   private double[] defaultProcessCovariances;  // keep track for resetting purposes

   private final YoBoolean passThroughEstimatesToController;

   private final YoBoolean tare;
   private final SpatialInertiaReadOnly[] urdfSpatialInertias;
   private final YoSpatialInertiaTemporary[] tareSpatialInertias;

   private final SpatialInertiaBasisOption[] processBasisOptions;

   public InertialParameterManager(InertialParameterManagerFactory.EstimatorType type, HighLevelHumanoidControllerToolbox toolbox, InertialEstimationParameters inertialEstimationParameters, YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      this.parameters = inertialEstimationParameters;

      enableFilter = new YoBoolean("enableFilter", registry);
      enableFilter.set(false);
      resetFilter = new YoBoolean("resetFilter", registry);
      resetFilter.set(false);
      resetConfig = new YoBoolean("resetConfig", registry);
      resetConfig.set(false);

      actualRobotModel = toolbox.getFullRobotModel();
      actualModelBodies = actualRobotModel.getRootBody().subtreeArray();
      actualModelJoints = actualRobotModel.getRootJoint().subtreeList();

      RigidBodyBasics clonedElevatorForEstimates = MultiBodySystemFactories.cloneMultiBodySystem(actualRobotModel.getElevator(),
                                                                                     actualRobotModel.getModelStationaryFrame(),
                                                                                     "_estimate");
      estimateRobotModel = new FullHumanoidRobotModelWrapper(clonedElevatorForEstimates, false);
      estimateModelBodies = estimateRobotModel.getRootBody().subtreeArray();
      yoInertiaEllipsoids = InertiaVisualizationTools.createYoInertiaEllipsoids(actualRobotModel.getRootBody(), registry);
      ellipsoidGraphicGroup = InertiaVisualizationTools.getInertiaEllipsoidGroup(yoInertiaEllipsoids);

      RigidBodyBasics clonedElevatorForInverseDynamics = MultiBodySystemFactories.cloneMultiBodySystem(actualRobotModel.getElevator(),
                                                                                                 actualRobotModel.getModelStationaryFrame(),
                                                                                                 "_inverseDynamics");
      inverseDynamicsRobotModel = new FullHumanoidRobotModelWrapper(clonedElevatorForInverseDynamics, false);
      RigidBodyBasics[] inverseDynamicsModelBodies = inverseDynamicsRobotModel.getRootBody().subtreeArray();
      inverseDynamicsModelJoints = inverseDynamicsRobotModel.getRootJoint().subtreeList();
      inverseDynamicsCalculator = new InverseDynamicsCalculator(inverseDynamicsRobotModel.getElevator());
      inverseDynamicsCalculator.setGravitationalAcceleration(-toolbox.getGravityZ());

      RigidBodyBasics clonedElevatorForRegressor = MultiBodySystemFactories.cloneMultiBodySystem(actualRobotModel.getElevator(),
                                                                                                 actualRobotModel.getModelStationaryFrame(),
                                                                                                 "_regressor");
      regressorRobotModel = new FullHumanoidRobotModelWrapper(clonedElevatorForRegressor, false);
      RigidBodyBasics[] regressorModelBodies = regressorRobotModel.getRootBody().subtreeArray();
      regressorModelJoints = regressorRobotModel.getRootJoint().subtreeList();
      regressorCalculator = new JointTorqueRegressorCalculator(regressorRobotModel.getElevator());
      regressorCalculator.setGravitationalAcceleration(-toolbox.getGravityZ());

      int nOneDoFJoints = estimateRobotModel.getOneDoFJoints().length;
      nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(estimateRobotModel.getRootJoint().subtreeArray());
      basisSets = parameters.getParametersToEstimate();
      int[] sizes = RegressorTools.sizePartitions(basisSets);
      nParameters = sizes[0];
      int nNonEmptyBasisSets = 0;
      for (Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption> basisSet : basisSets)
         if (!basisSet.isEmpty())
            nNonEmptyBasisSets++;

      regressorBlocks = new DMatrixRMaj[nNonEmptyBasisSets];
      for (int i = 0; i < nNonEmptyBasisSets; i++)
         regressorBlocks[i] = new DMatrixRMaj(nDoFs, RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
      regressor = new DMatrixRMaj(nDoFs, nParameters);

      zeroInverseDynamicsParameters(inverseDynamicsModelBodies, basisSets);

      regressorModelBodiesToProcess = new RigidBodyBasics[nNonEmptyBasisSets];
      int regressorIndex = 0;
      for (int i = 0; i < basisSets.length; ++i)
      {
         if (!basisSets[i].isEmpty())
         {
            regressorModelBodiesToProcess[regressorIndex] = regressorModelBodies[i];
            regressorIndex++;
         }
      }

      this.footSwitches = toolbox.getFootSwitches();
      contactWrenches = new SideDependentList<>();
      jointIndexHandler = new JointIndexHandler(actualRobotModel.getElevator().subtreeJointStream().toArray(JointBasics[]::new));
      legJoints = new SideDependentList<>();
      compactContactJacobians = new SideDependentList<>();
      fullContactJacobians = new SideDependentList<>();

      // NOTE: for the leg joints and compact jacobians, we use the actual robot model because it has the full model information, including all joint names
      for (RobotSide side : RobotSide.values)
      {
         contactWrenches.put(side, new YoMatrix("wrench" + side.getPascalCaseName(), WRENCH_DIMENSION, 1, null, null, registry));
         legJoints.put(side, MultiBodySystemTools.createJointPath(actualRobotModel.getElevator(), actualRobotModel.getFoot(side)));
         compactContactJacobians.put(side, new GeometricJacobian(legJoints.get(side), footSwitches.get(side).getMeasurementFrame()));
         fullContactJacobians.put(side, new DMatrixRMaj(WRENCH_DIMENSION, nDoFs));
      }

      wholeSystemTorques = new DMatrixRMaj(nDoFs, 1);

      double dt = toolbox.getControlDT();

      rootJointVelocity = new DMatrixRMaj(WRENCH_DIMENSION, 1);
      rootJointVelocities = new YoDouble[WRENCH_DIMENSION];
      rootJointAccelerations = new FilteredVelocityYoVariable[WRENCH_DIMENSION];
      rootJointAcceleration = new DMatrixRMaj(WRENCH_DIMENSION, 1);
      accelerationCalculationAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForAccelerationCalculation(), dt);
      for (int i = 0; i < WRENCH_DIMENSION; i++)
      {
         rootJointVelocities[i] = new YoDouble("rootJointVelocity_" + getNameForRootJoint(i), registry);
         rootJointAccelerations[i] = new FilteredVelocityYoVariable("rootJointAcceleration_" + getNameForRootJoint(i), "",
                                                                    accelerationCalculationAlpha, rootJointVelocities[i], dt, registry);
      }

      jointVelocities = new YoDouble[nOneDoFJoints];
      jointAccelerations = new FilteredVelocityYoVariable[nOneDoFJoints];
      for (int i = 0; i < nOneDoFJoints; i++)
      {
         jointVelocities[i] = new YoDouble("jointVelocity_" + actualRobotModel.getOneDoFJoints()[i].getName(), registry);
         jointAccelerations[i] = new FilteredVelocityYoVariable("jointAcceleration_" + actualRobotModel.getOneDoFJoints()[i].getName(), "",
                                                                accelerationCalculationAlpha, jointVelocities[i], dt, registry);
      }

      postProcessingAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForPostProcessing(), dt);

      estimate = new YoMatrix("inertialParameterEstimate", nParameters,
                              1,
                              getRowNamesForEstimates(basisSets, estimateModelBodies),
                              null,
                              registry);

      estimateFilteringAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForEstimateFiltering(), dt);
      filteredEstimate = new AlphaFilteredYoMatrix("filteredInertialParameterEstimate", estimateFilteringAlpha, nParameters,
                                                   1,
                                                   getRowNamesForEstimates(basisSets, estimateModelBodies),
                                                   null,
                                                   registry);
      doubleFilteredEstimate = new AlphaFilteredYoMatrix("doubleFilteredInertialParameterEstimate", estimateFilteringAlpha, nParameters,
                                                         1,
                                                         getRowNamesForEstimates(basisSets, estimateModelBodies),
                                                         null,
                                                         registry);

      // Process covariances are initialized here, but actually set later depending on the type of filter used
      // (and thus on the type of parameter -- which will require different covariances for different units)
      processCovariancesForSingleBody = new YoDouble[RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY];
      floatingBaseMeasurementCovariance = new YoDouble("floatingBaseMeasurementCovariance", registry);
      floatingBaseMeasurementCovariance.set(parameters.getFloatingBaseMeasurementCovariance());
      legsMeasurementCovariance = new YoDouble("legsMeasurementCovariance", registry);
      legsMeasurementCovariance.set(parameters.getLegMeasurementCovariance());
      armsMeasurementCovariance = new YoDouble("armsMeasurementCovariance", registry);
      armsMeasurementCovariance.set(parameters.getArmMeasurementCovariance());
      spineMeasurementCovariance = new YoDouble("spineMeasurementCovariance", registry);
      spineMeasurementCovariance.set(parameters.getSpineMeasurementCovariance());

      reduceProcessCovarianceWhileWalking = new YoBoolean("reduceProcessCovarianceWhileWalking", registry);
      processCovarianceWalkingMultiplier = new YoDouble("processCovarianceWalkingMultiplier", registry);
      processCovarianceWalkingMultiplier.set(parameters.getProcessCovarianceMultiplierForWalking());
      processCovariancePassedToFilter = new YoMatrix("processCovariancePassedToFilter", nParameters, nParameters, registry);

      String[] rowNames = getRowNamesForJoints(nDoFs);
      residual = new YoMatrix("residual", nDoFs, 1, rowNames, registry);

      int windowSizeInTicks = (int) (parameters.getBiasCompensationWindowSizeInSeconds() / dt);
      calculateBias = new YoBoolean("calculateBias", registry);
      biasCompensator = new InertialBiasCompensator(nDoFs, windowSizeInTicks, getRowNamesForJoints(nDoFs), registry);
      bias = new YoMatrix("bias", nDoFs, 1, rowNames, null, registry);
      excludeBias = new YoBoolean("excludeBias", registry);
      excludeBias.set(false);
      eraseBias = new YoBoolean("eraseBias", registry);
      eraseBias.set(false);

      normalizedInnovation = new YoDouble("normalizedInnovation", registry);
      normalizedInnovation.set(0.0);

      normalizedInnovationThreshold = new YoDouble("normalizedInnovationThreshold", registry);
      normalizedInnovationThreshold.set(parameters.getNormalizedInnovationThreshold());

      // Construct the type of filter used based on enum value
      setFilter(type);

      passThroughEstimatesToController = new YoBoolean("passThroughEstimatesToController", registry);
      passThroughEstimatesToController.set(false);

      tare = new YoBoolean("tare", registry);
      tare.set(false);
      tareSpatialInertias = new YoSpatialInertiaTemporary[estimateModelBodies.length];
      urdfSpatialInertias = new SpatialInertiaBasics[estimateModelBodies.length];
      for (int i = 0; i < estimateModelBodies.length; i++)
      {
         tareSpatialInertias[i] = new YoSpatialInertiaTemporary(estimateModelBodies[i].getInertia(), "_tare", registry);
         urdfSpatialInertias[i] = new SpatialInertia(estimateModelBodies[i].getInertia());
      }

      processBasisOptions = new SpatialInertiaBasisOption[nParameters];
      int index = 0;
      for (Set<SpatialInertiaBasisOption> basisSet : basisSets)
      {
         for (SpatialInertiaBasisOption basisOption : SpatialInertiaBasisOption.values)
         {
            if (basisSet.contains(basisOption))
            {
               processBasisOptions[index] = basisOption;
               index++;
            }
         }
      }

   }

   private void setFilter(InertialParameterManagerFactory.EstimatorType type)
   {
      defaultProcessCovariances = parameters.getProcessModelCovarianceForBody();
      switch (type)
      {
         case KF ->
         {
            filter = new InertialKalmanFilter(estimateRobotModel,
                                              basisSets,
                                              parameters,
                                              parameters.getURDFParameters(basisSets),
                                              CommonOps_DDRM.identity(nParameters),
                                              CommonOps_DDRM.identity(nParameters),
                                              CommonOps_DDRM.identity(nDoFs), postProcessingAlpha,
                                              registry);
            createProcessCovariances(RigidBodyInertialParametersTools.getNamesForPiBasis(), defaultProcessCovariances);
            filter.setNormalizedInnovationThreshold(parameters.getNormalizedInnovationThreshold());
         }
         case CONSTRAINED_KF ->
         {
            filter = new InertialConstrainedKalmanFilter(estimateRobotModel,
                                                         basisSets,
                                                         parameters,
                                                         parameters.getURDFParameters(basisSets),
                                                         CommonOps_DDRM.identity(nParameters),
                                                         CommonOps_DDRM.identity(nParameters),
                                                         CommonOps_DDRM.identity(nDoFs), postProcessingAlpha,
                                                         registry);
            createProcessCovariances(RigidBodyInertialParametersTools.getNamesForPiBasis(), defaultProcessCovariances);
            filter.setNormalizedInnovationThreshold(parameters.getNormalizedInnovationThreshold());
         }
         case PHYSICALLY_CONSISTENT_EKF ->
         {
            filter = new InertialPhysicallyConsistentKalmanFilter(estimateRobotModel,
                                                                  basisSets,
                                                                  parameters,
                                                                  parameters.getURDFParameters(basisSets),
                                                                  CommonOps_DDRM.identity(nParameters),
                                                                  CommonOps_DDRM.identity(nParameters),
                                                                  CommonOps_DDRM.identity(nDoFs), postProcessingAlpha,
                                                                  registry);
            createProcessCovariances(RigidBodyInertialParametersTools.getNamesForThetaBasis(), defaultProcessCovariances);
            filter.setNormalizedInnovationThreshold(parameters.getNormalizedInnovationThreshold());
         }
      }
   }

   private void createProcessCovariances(String[] names, double[] defaults)
   {
      for (int i = 0; i < RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY; ++i)
      {
         processCovariancesForSingleBody[i] = new YoDouble("processCovariance_" + names[i], registry);
         processCovariancesForSingleBody[i].set(defaults[i]);
      }
   }

   public void reset()
   {
      for (RobotSide side : RobotSide.values)
      {
         fullContactJacobians.get(side).zero();
         contactWrenches.get(side).zero();
      }

      wholeSystemTorques.zero();
      rootJointVelocity.zero();
      rootJointAcceleration.zero();
      for (int i = 0; i < WRENCH_DIMENSION; i++)
      {
         rootJointVelocities[i].set(0.0);
         rootJointAccelerations[i].set(0.0);
         rootJointAccelerations[i].reset();
      }

      for (int i = 0; i < jointVelocities.length; i++)
      {
         jointVelocities[i].set(0.0);
         jointAccelerations[i].set(0.0);
         jointAccelerations[i].reset();
      }

      estimate.zero();
      filteredEstimate.zero();
      doubleFilteredEstimate.zero();

      residual.zero();
      bias.zero();
   }

   public void resetConfig()
   {
      for (int i = 0; i < RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY; ++i)
         processCovariancesForSingleBody[i].set(defaultProcessCovariances[i]);

      floatingBaseMeasurementCovariance.set(parameters.getFloatingBaseMeasurementCovariance());
      legsMeasurementCovariance.set(parameters.getLegMeasurementCovariance());
      armsMeasurementCovariance.set(parameters.getArmMeasurementCovariance());
      spineMeasurementCovariance.set(parameters.getSpineMeasurementCovariance());

      filter.setPostProcessingAlpha(postProcessingAlpha);
      filteredEstimate.setAlpha(estimateFilteringAlpha);
      doubleFilteredEstimate.setAlpha(estimateFilteringAlpha);

      for (int i = 0; i < WRENCH_DIMENSION; i++)
         rootJointAccelerations[i].setAlpha(accelerationCalculationAlpha);

      for (int i = 0; i < jointAccelerations.length; i++)
         jointAccelerations[i].setAlpha(accelerationCalculationAlpha);

   }

   public void update()
   {
      if (resetFilter.getValue())
      {
         reset();
         resetFilter.set(false);
         enableFilter.set(false);  // Also turn off the estimator if we reset
      }

      if (resetConfig.getValue())
      {
         resetConfig();
         resetConfig.set(false);
      }

      if (calculateBias.getValue())
      {
         if (biasCompensator.isWindowFilled())
         {
            biasCompensator.calculateBias();
            calculateBias.set(false);
         }
         else
         {
            biasCompensator.update(residual);
            biasCompensator.incrementCounter();
         }
      }

      if (eraseBias.getValue())
      {
         biasCompensator.reset();
         calculateBias.set(false);
         eraseBias.set(false);
      }

      if (tare.getValue())
      {
         updateTareSpatialInertias();
         tare.set(false);
      }

      if (enableFilter.getValue())
      {
         updateFilterCovariances();

         updateRegressorModelJointStates();

         updateContactJacobians();
         updateContactWrenches();
         updateWholeSystemTorques();

         regressorTimer.startMeasurement();
         inverseDynamicsCalculator.compute();
         regressorCalculator.compute(regressorModelBodiesToProcess);
         regressorTimer.stopMeasurement();

         for (int i = 0; i < regressorModelBodiesToProcess.length; i++)
            regressorBlocks[i].set(regressorCalculator.getJointTorqueRegressorMatrixBlock(regressorModelBodiesToProcess[i]));
         packRegressorFromBlocks(regressorBlocks, basisSets, regressor);

         filter.setTorqueFromNominal(inverseDynamicsCalculator.getJointTauMatrix());
         filter.setRegressor(regressor);
         filter.setContactJacobians(fullContactJacobians);
         filter.setContactWrenches(contactWrenches);

         if (excludeBias.getValue())
            filter.setBias(biasCompensator.getZero());
         else
            filter.setBias(biasCompensator.getBias());

         filter.setNormalizedInnovationThreshold(normalizedInnovationThreshold.getValue());

         estimate.set(filter.calculateEstimate(wholeSystemTorques));
         filter.getMeasurementResidual(residual);

         filteredEstimate.setAndSolve(estimate);
         doubleFilteredEstimate.setAndSolve(filteredEstimate);

         normalizedInnovation.set(filter.calculateNormalizedInnovation());

         // Pack smoothed estimate back into estimate robot bodies
         RegressorTools.packRigidBodies(basisSets, doubleFilteredEstimate, estimateModelBodies);

         // Pass through estimates to controller
         if (passThroughEstimatesToController.getValue())
            updateActualRobotModel();

         updateVisuals();
      }
   }

   private void updateFilterCovariances()
   {
      // Set diagonal of process covariance
      DMatrixRMaj processCovariance = filter.getProcessCovariance();
      for (int i = 0; i < processCovariance.getNumRows(); ++i)  // we'll set the diagonals
      {
         // Mod by the number of parameters per rigid body to cycle through the parameters
         processCovariance.set(i, i, processCovariancesForSingleBody[processBasisOptions[i % RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY].ordinal()].getValue());
      }

      if (reduceProcessCovarianceWhileWalking.getValue())
      {
         // If process covariance modification while walking is enabled, we need to see if the robot is walking by inspecting the foot switches
         // NOTE: the XOR on the foot switches
         if (footSwitches.get(RobotSide.LEFT).hasFootHitGroundFiltered() ^ footSwitches.get(RobotSide.RIGHT).hasFootHitGroundFiltered())
            CommonOps_DDRM.scale(processCovarianceWalkingMultiplier.getValue(), processCovariance);
      }
      processCovariancePassedToFilter.set(processCovariance);

      // Set diagonal entries of measurement covariance according to the part of the body
      for (int j = 0; j < actualModelJoints.size(); ++j)
      {
         JointReadOnly joint = actualModelJoints.get(j);
         int[] indices = jointIndexHandler.getJointIndices(joint);

         if (joint.getName().contains("PELVIS"))
            MatrixMissingTools.setMatrixDiagonal(indices, floatingBaseMeasurementCovariance.getValue(), filter.getMeasurementCovariance());
         else if (joint.getName().contains("HIP") || joint.getName().contains("KNEE") || joint.getName().contains("ANKLE"))
            MatrixMissingTools.setMatrixDiagonal(indices, legsMeasurementCovariance.getValue(), filter.getMeasurementCovariance());
         else if (joint.getName().contains("SHOULDER") || joint.getName().contains("ELBOW") || joint.getName().contains("WRIST"))
            MatrixMissingTools.setMatrixDiagonal(indices, armsMeasurementCovariance.getValue(), filter.getMeasurementCovariance());
         else if (joint.getName().contains("SPINE"))
            MatrixMissingTools.setMatrixDiagonal(indices, spineMeasurementCovariance.getValue(), filter.getMeasurementCovariance());
         else
            LogTools.info("Joint " + joint.getName() + " not found for measurement covariance");
      }
   }

   private void updateContactWrenches()
   {
      for (RobotSide side : RobotSide.values)
         footSwitches.get(side).getMeasuredWrench().get(contactWrenches.get(side));
   }

   private void updateRegressorModelJointStates()
   {
      MultiBodySystemTools.copyJointsState(actualModelJoints, regressorModelJoints, JointStateType.CONFIGURATION);
      MultiBodySystemTools.copyJointsState(actualModelJoints, regressorModelJoints, JointStateType.VELOCITY);

      MultiBodySystemTools.copyJointsState(actualModelJoints, inverseDynamicsModelJoints, JointStateType.CONFIGURATION);
      MultiBodySystemTools.copyJointsState(actualModelJoints, inverseDynamicsModelJoints, JointStateType.VELOCITY);

      // Update joint accelerations by processing joint velocities
      calculateJointAccelerations();
      OneDoFJointBasics[] regressorOneDoFJoints = regressorRobotModel.getOneDoFJoints();
      OneDoFJointBasics[] inverseDynamicsOneDoFJoints = inverseDynamicsRobotModel.getOneDoFJoints();
      for (int i = 0; i < jointAccelerations.length; i++)
      {
         regressorOneDoFJoints[i].setQdd(jointAccelerations[i].getValue());
         inverseDynamicsOneDoFJoints[i].setQdd(jointAccelerations[i].getValue());
      }

      // Update root joint acceleration, which is not populated by default
      calculateRootJointAccelerations();
      regressorRobotModel.getRootJoint().setJointAcceleration(0, rootJointAcceleration);
      inverseDynamicsRobotModel.getRootJoint().setJointAcceleration(0, rootJointAcceleration);

      regressorRobotModel.updateFrames();
      inverseDynamicsRobotModel.updateFrames();
   }

   private void updateWholeSystemTorques()
   {
      actualRobotModel.getRootJoint().getJointTau(0, wholeSystemTorques);
      OneDoFJointBasics[] actualOneDoFJoints = actualRobotModel.getOneDoFJoints();
      for (int i = 0; i < actualOneDoFJoints.length; ++i)
      {
         OneDoFJointReadOnly joint = actualOneDoFJoints[i];
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);
         joint.getJointTau(jointIndex, wholeSystemTorques);
      }
   }

   private void updateContactJacobians()
   {
      for (RobotSide side : RobotSide.values)
      {
         compactContactJacobians.get(side).compute();
         jointIndexHandler.compactBlockToFullBlock(legJoints.get(side), compactContactJacobians.get(side).getJacobianMatrix(), fullContactJacobians.get(side));
      }
   }

   private void calculateRootJointAccelerations()
   {
      actualRobotModel.getRootJoint().getJointVelocity(0, rootJointVelocity);
      for (int i = 0; i < WRENCH_DIMENSION; i++)
      {
         rootJointVelocities[i].set(rootJointVelocity.get(i, 0));
         rootJointAccelerations[i].update(rootJointVelocities[i].getValue());
         rootJointAcceleration.set(i, 0, rootJointAccelerations[i].getValue());
      }
   }

   private void calculateJointAccelerations()
   {
      OneDoFJointBasics[] actualOneDoFJoints = actualRobotModel.getOneDoFJoints();
      for (int i = 0; i < jointVelocities.length; i++)
      {
         jointVelocities[i].set(actualOneDoFJoints[i].getQd());
         jointAccelerations[i].update(jointVelocities[i].getValue());
      }
   }

   private void zeroInverseDynamicsParameters(RigidBodyBasics[] bodies, Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets)
   {
      if (bodies.length != basisSets.length)
         throw new RuntimeException("Mismatch between bodies and basis sets");

      for (int i = 0; i < basisSets.length; ++i)
      {
         if (!basisSets[i].isEmpty())
            for (JointTorqueRegressorCalculator.SpatialInertiaBasisOption option : basisSets[i])
            {
               switch (option)
               {
                  case M -> bodies[i].getInertia().setMass(0.0);
                  case MCOM_X -> bodies[i].getInertia().getCenterOfMassOffset().setX(0.0);
                  case MCOM_Y -> bodies[i].getInertia().getCenterOfMassOffset().setY(0.0);
                  case MCOM_Z -> bodies[i].getInertia().getCenterOfMassOffset().setZ(0.0);
                  case I_XX -> bodies[i].getInertia().getMomentOfInertia().setM00(0.0);
                  case I_XY ->
                  {
                     bodies[i].getInertia().getMomentOfInertia().setM01(0.0);
                     bodies[i].getInertia().getMomentOfInertia().setM10(0.0);
                  }
                  case I_YY -> bodies[i].getInertia().getMomentOfInertia().setM11(0.0);
                  case I_XZ ->
                  {
                     bodies[i].getInertia().getMomentOfInertia().setM02(0.0);
                     bodies[i].getInertia().getMomentOfInertia().setM20(0.0);
                  }
                  case I_YZ ->
                  {
                     bodies[i].getInertia().getMomentOfInertia().setM12(0.0);
                     bodies[i].getInertia().getMomentOfInertia().setM21(0.0);
                  }
                  case I_ZZ -> bodies[i].getInertia().getMomentOfInertia().setM22(0.0);
               }
            }
      }
   }

   private void packRegressorFromBlocks(DMatrixRMaj[] regressorBlocks, Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets, DMatrixRMaj regressorToPack)
   {
      int regressorBlockIndex = 0;
      int regressorToPackColumnIndex = 0;
      for (Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption> basisSet : basisSets)
      {
         if (!basisSet.isEmpty())
         {
            for (int i = 0; i < JointTorqueRegressorCalculator.SpatialInertiaBasisOption.values.length; ++i)
            {
               if (basisSet.contains(JointTorqueRegressorCalculator.SpatialInertiaBasisOption.values[i]))
               {
                  MatrixMissingTools.setMatrixColumn(regressorToPack, regressorToPackColumnIndex, regressorBlocks[regressorBlockIndex], i);
                  regressorToPackColumnIndex++;
                  }
            }
            regressorBlockIndex++;
         }
      }
   }

   private void updateActualRobotModel()
   {
      for (int i = 0; i < estimateModelBodies.length; i++)
      {
         if (!basisSets[i].isEmpty())  // Only update the bodies we're estimating
         {
            SpatialInertiaReadOnly estimateBodyInertia = estimateModelBodies[i].getInertia();
            SpatialInertiaReadOnly tareBodyInertia = tareSpatialInertias[i];
            SpatialInertiaReadOnly urdfBodyInertia = urdfSpatialInertias[i];

            SpatialInertiaBasics actualBodyInertia = actualModelBodies[i].getInertia();

            // TODO: put some safety checks here so we don't send nutty stuff to the robot maybe innovation gating?
            // Calculate the current differences from the tare value, these deltas are what we send to the controller
            double deltaMass = estimateBodyInertia.getMass() - tareBodyInertia.getMass();
            actualBodyInertia.setMass(urdfBodyInertia.getMass() + deltaMass);

            double deltaCoMX = estimateBodyInertia.getCenterOfMassOffset().getX() - tareBodyInertia.getCenterOfMassOffset().getX();
            double deltaCoMY = estimateBodyInertia.getCenterOfMassOffset().getY() - tareBodyInertia.getCenterOfMassOffset().getY();
            double deltaCoMZ = estimateBodyInertia.getCenterOfMassOffset().getZ() - tareBodyInertia.getCenterOfMassOffset().getZ();
            actualBodyInertia.setCenterOfMassOffset(urdfBodyInertia.getCenterOfMassOffset().getX() + deltaCoMX,
                                                   urdfBodyInertia.getCenterOfMassOffset().getY() + deltaCoMY,
                                                   urdfBodyInertia.getCenterOfMassOffset().getZ() + deltaCoMZ);

            double deltaIxx = estimateBodyInertia.getMomentOfInertia().getM00() - tareBodyInertia.getMomentOfInertia().getM00();
            double deltaIxy = estimateBodyInertia.getMomentOfInertia().getM01() - tareBodyInertia.getMomentOfInertia().getM01();
            double deltaIxz = estimateBodyInertia.getMomentOfInertia().getM02() - tareBodyInertia.getMomentOfInertia().getM02();
            double deltaIyy = estimateBodyInertia.getMomentOfInertia().getM11() - tareBodyInertia.getMomentOfInertia().getM11();
            double deltaIyz = estimateBodyInertia.getMomentOfInertia().getM12() - tareBodyInertia.getMomentOfInertia().getM12();
            double deltaIzz = estimateBodyInertia.getMomentOfInertia().getM22() - tareBodyInertia.getMomentOfInertia().getM22();
            actualBodyInertia.setMomentOfInertia(urdfBodyInertia.getMomentOfInertia().getM00() + deltaIxx,
                                               urdfBodyInertia.getMomentOfInertia().getM01() + deltaIxy,
                                               urdfBodyInertia.getMomentOfInertia().getM02() + deltaIxz,
                                               urdfBodyInertia.getMomentOfInertia().getM11() + deltaIyy,
                                               urdfBodyInertia.getMomentOfInertia().getM12() + deltaIyz,
                                               urdfBodyInertia.getMomentOfInertia().getM22() + deltaIzz);
         }
      }
   }

   private void updateTareSpatialInertias()
   {
      for (int i = 0; i < estimateModelBodies.length; i++)
         if (!basisSets[i].isEmpty())  // Only tare the bodies we're estimating
            tareSpatialInertias[i].set(estimateModelBodies[i].getInertia());
   }

   private void updateVisuals()
   {
      for (int i = 0; i < estimateModelBodies.length; i++)
      {
         RigidBodyReadOnly actualBody = actualModelBodies[i];
         RigidBodyReadOnly estimateBody = estimateModelBodies[i];

         double scale = EuclidCoreTools.clamp(estimateBody.getInertia().getMass() / actualBody.getInertia().getMass() / 2.0, 0.0, 1.0);

         if (estimateBody.getInertia() != null && actualBody.getInertia() != null)
         {
            yoInertiaEllipsoids.get(i).update(scale);
         }
      }
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(ellipsoidGraphicGroup);
      return group;
   }

   private String getNameForRootJoint(int i)
   {
      return switch (i)
      {
         case 0 -> "wX";
         case 1 -> "wY";
         case 2 -> "wZ";
         case 3 -> "x";
         case 4 -> "y";
         case 5 -> "z";
         default -> throw new RuntimeException("Unhandled case: " + i);
      };
   }

   private String[] getRowNamesForJoints(int nDoFs)
   {
      String[] rowNames = new String[nDoFs];
      int index = 0;
      for (JointReadOnly joint : actualModelJoints)
      {
         if (joint.getDegreesOfFreedom() > 1)
         {
            for (int i = 0; i < joint.getDegreesOfFreedom(); i++)
               rowNames[index + i] = joint.getName() + "_" + getNameForRootJoint(i);
         }
         else
         {
            rowNames[index] = joint.getName();
         }
         index += joint.getDegreesOfFreedom();
      }
      return rowNames;
   }

   private String[] getRowNamesForEstimates(Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets, RigidBodyReadOnly[] bodies)
   {
      List<String> rowNames = new ArrayList<>();
      for (int i = 0; i < basisSets.length; ++i)
      {
         for (JointTorqueRegressorCalculator.SpatialInertiaBasisOption option : JointTorqueRegressorCalculator.SpatialInertiaBasisOption.values)
         {
            if (basisSets[i].contains(option))
               rowNames.add(bodies[i].getName() + "_" + option.name());
         }
      }

      return rowNames.toArray(new String[0]);
   }
}
