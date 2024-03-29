package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.configurations.InertialEstimationParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
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
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.*;

/**
 * This class is used to manage the estimation of the inertial parameters of a robot. It is designed to be used in a control loop, where the inertial parameters
 * are being estimated in parallel.
 *
 * @author James Foster
 */
public class InertialParameterManager implements SCS2YoGraphicHolder
{
   private static final int WRENCH_DIMENSION = 6;

   private final YoBoolean enableFilter;

   private final MultipleHumanoidModelHandler<RobotModelTask> modelHandler;
   private final HumanoidModelCovarianceHelper covarianceHelper;
   private final InertialBaselineCalculator baselineCalculator;

   private final ArrayList<YoInertiaEllipsoid> yoInertiaEllipsoids;
   private final YoGraphicDefinition ellipsoidGraphicGroup;

   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   private final JointTorqueRegressorCalculator regressorCalculator;

   private final SideDependentList<? extends FootSwitchInterface> footSwitches;
   private final SideDependentList<DMatrixRMaj> contactWrenches;
   private final JointIndexHandler jointIndexHandler;
   private final SideDependentList<JointBasics[]> legJoints;
   private final SideDependentList<GeometricJacobian> compactContactJacobians;
   private final SideDependentList<DMatrixRMaj> fullContactJacobians;

   private final DMatrixRMaj wholeSystemTorques;

   private final DMatrixRMaj jointVelocitiesContainer;
   private final FilteredVelocityYoVariable[] jointAccelerations;

   private final Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets;
   private final RigidBodyBasics[] regressorModelBodiesToProcess;
   private final DMatrixRMaj[] regressorBlocks;
   private final DMatrixRMaj regressor;

   private OnlineInertialEstimator filter;
   private final YoMatrix estimate;
   private final AlphaFilteredYoMatrix filteredEstimate;

   private final YoMatrix residual;

   private final InertialBiasCompensator biasCompensator;
   private final YoBoolean calculateBias;
   private final YoBoolean excludeBias;
   private final YoBoolean eraseBias;

   private final YoDouble accelerationCalculationAlpha;  // useful to have a master setting, we want to filter all DoFs equally

   private final YoDouble normalizedInnovation;
   private final YoDouble normalizedInnovationThreshold;

   private final YoBoolean passThroughEstimatesToController;
   private final YoBoolean tare;

   private final YoBoolean areParametersPhysicallyConsistent;

   public InertialParameterManager(HighLevelHumanoidControllerToolbox toolbox, InertialEstimationParameters inertialEstimationParameters, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);
      basisSets = inertialEstimationParameters.getBasisSets();

      enableFilter = new YoBoolean("enableFilter", registry);
      enableFilter.set(false);

      modelHandler = new MultipleHumanoidModelHandler<>(RobotModelTask.class);
      FullHumanoidRobotModel controllerRobotModel = toolbox.getFullRobotModel();
      modelHandler.putRobotModel(RobotModelTask.CONTROLLER, controllerRobotModel);

      for (RobotModelTask task : new RobotModelTask[] {RobotModelTask.ESTIMATE, RobotModelTask.INVERSE_DYNAMICS, RobotModelTask.REGRESSOR})
      {
         RigidBodyBasics clonedElevator = MultiBodySystemFactories.cloneMultiBodySystem(controllerRobotModel.getElevator(),
                                                                                       controllerRobotModel.getModelStationaryFrame(),
                                                                                       "_" + task.name());
         FullHumanoidRobotModel clonedRobotModel = new FullHumanoidRobotModelWrapper(clonedElevator, false);
         modelHandler.putRobotModel(task, clonedRobotModel);
      }

      covarianceHelper = new HumanoidModelCovarianceHelper(controllerRobotModel, inertialEstimationParameters, registry);
      baselineCalculator = new InertialBaselineCalculator(modelHandler.getRobotModel(RobotModelTask.ESTIMATE), inertialEstimationParameters, 1.0, registry);

      yoInertiaEllipsoids = InertiaVisualizationTools.createYoInertiaEllipsoids(controllerRobotModel.getRootBody(), registry);
      ellipsoidGraphicGroup = InertiaVisualizationTools.getInertiaEllipsoidGroup(yoInertiaEllipsoids);

      inverseDynamicsCalculator = new InverseDynamicsCalculator(modelHandler.getRobotModel(RobotModelTask.INVERSE_DYNAMICS).getElevator());
      inverseDynamicsCalculator.setGravitationalAcceleration(-toolbox.getGravityZ());
      zeroInverseDynamicsParameters(basisSets);
      regressorCalculator = new JointTorqueRegressorCalculator(modelHandler.getRobotModel(RobotModelTask.REGRESSOR).getElevator());
      regressorCalculator.setGravitationalAcceleration(-toolbox.getGravityZ());

      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(controllerRobotModel.getRootJoint().subtreeArray());
      int nParameters = inertialEstimationParameters.getNumberOfParameters();
      int nNonEmptyBasisSets = inertialEstimationParameters.getNumberOfNonEmptyBasisSets();

      // To be as efficient as possible, we construct a compact regressor that only contains the bodies being estimated
      regressorBlocks = new DMatrixRMaj[nNonEmptyBasisSets];
      for (int i = 0; i < nNonEmptyBasisSets; i++)
         regressorBlocks[i] = new DMatrixRMaj(nDoFs, RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
      regressor = new DMatrixRMaj(nDoFs, nParameters);
      regressorModelBodiesToProcess = new RigidBodyBasics[nNonEmptyBasisSets];
      int regressorIndex = 0;
      for (int i = 0; i < basisSets.length; ++i)
      {
         if (basisSets[i].isEmpty())
            continue;

         regressorModelBodiesToProcess[regressorIndex] = modelHandler.getBodyArray(RobotModelTask.REGRESSOR)[i];
         regressorIndex++;
      }

      this.footSwitches = toolbox.getFootSwitches();
      contactWrenches = new SideDependentList<>();
      jointIndexHandler = new JointIndexHandler(controllerRobotModel.getRootJoint().subtreeArray());
      legJoints = new SideDependentList<>();
      compactContactJacobians = new SideDependentList<>();
      fullContactJacobians = new SideDependentList<>();
      // NOTE: for the leg joints and compact jacobians, we use the controller robot model because it has the full model information, including all joint names
      for (RobotSide side : RobotSide.values)
      {
         contactWrenches.put(side, new DMatrixRMaj(WRENCH_DIMENSION, 1));
         legJoints.put(side, MultiBodySystemTools.createJointPath(controllerRobotModel.getElevator(), controllerRobotModel.getFoot(side)));
         compactContactJacobians.put(side, new GeometricJacobian(legJoints.get(side), footSwitches.get(side).getMeasurementFrame()));
         fullContactJacobians.put(side, new DMatrixRMaj(WRENCH_DIMENSION, nDoFs));
      }

      wholeSystemTorques = new DMatrixRMaj(nDoFs, 1);

      String[] measurementNames = inertialEstimationParameters.getMeasurementNames();
      residual = new YoMatrix("residual_", nDoFs, 1, measurementNames, registry);

      double dt = toolbox.getControlDT();
      double defaultAccelerationCalculationAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(inertialEstimationParameters.getBreakFrequencyForAccelerationCalculation(), dt);
      accelerationCalculationAlpha = new YoDouble("accelerationCalculationAlpha", registry);
      accelerationCalculationAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(inertialEstimationParameters.getBreakFrequencyForAccelerationCalculation(), dt));
      jointVelocitiesContainer = new DMatrixRMaj(nDoFs, 1);
      jointAccelerations = new FilteredVelocityYoVariable[nDoFs];
      for (int i = 0; i < measurementNames.length; i++)
         jointAccelerations[i] = new FilteredVelocityYoVariable("jointAcceleration_" + measurementNames[i], "", defaultAccelerationCalculationAlpha, dt, registry);

      String[] estimateNames = inertialEstimationParameters.getEstimateNames();
      estimate = new YoMatrix("", nParameters, 1, estimateNames, null, registry);
      double defaultEstimateFilteringAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(inertialEstimationParameters.getBreakFrequencyForEstimateFiltering(), dt);
      filteredEstimate = new AlphaFilteredYoMatrix("filtered_", defaultEstimateFilteringAlpha, nParameters, 1, estimateNames, null, registry);

      int windowSizeInTicks = (int) (inertialEstimationParameters.getBiasCompensationWindowSizeInSeconds() / dt);
      calculateBias = new YoBoolean("calculateBias", registry);
      biasCompensator = new InertialBiasCompensator(nDoFs, windowSizeInTicks, measurementNames, registry);
      excludeBias = new YoBoolean("excludeBias", registry);
      excludeBias.set(false);
      eraseBias = new YoBoolean("eraseBias", registry);
      eraseBias.set(false);

      normalizedInnovation = new YoDouble("normalizedInnovation", registry);
      normalizedInnovation.set(0.0);
      normalizedInnovationThreshold = new YoDouble("normalizedInnovationThreshold", registry);
      normalizedInnovationThreshold.set(inertialEstimationParameters.getNormalizedInnovationThreshold());

      // Construct the type of filter used based on enum value
      switch (inertialEstimationParameters.getTypeOfEstimatorToUse())
      {
         case KF -> filter = new InertialKalmanFilter(modelHandler.getRobotModel(RobotModelTask.ESTIMATE), inertialEstimationParameters, dt, registry);
         case PHYSICALLY_CONSISTENT_EKF -> filter = new InertialPhysicallyConsistentKalmanFilter(modelHandler.getRobotModel(RobotModelTask.ESTIMATE),
                                                                                                 inertialEstimationParameters, dt, registry);
      }

      passThroughEstimatesToController = new YoBoolean("passThroughEstimatesToController", registry);
      passThroughEstimatesToController.set(false);

      tare = new YoBoolean("tare", registry);
      tare.set(false);

      areParametersPhysicallyConsistent = new YoBoolean("areParametersPhysicallyConsistent", registry);
      areParametersPhysicallyConsistent.set(false);
   }

   private enum RobotModelTask
   {
      // the robot model used in the controller that filtered inertial parameters are sent to
      CONTROLLER,
      // the model used to estimate inertial parameters and for visualization
      ESTIMATE,
      // a model where the parameters are being estimated are zeroed, so that an inverse dynamics call results in torques from only known parameters
      INVERSE_DYNAMICS,
      // a model to use for regressor calculations, which requires all bodies to be iteratively zeroed inertially
      REGRESSOR
   }

   public void update()
   {
      // Handle bias compensation
      if (calculateBias.getValue())
      {
         boolean isBiasCalculated = biasCompensator.update(residual);
         calculateBias.set(isBiasCalculated);
      }
      if (eraseBias.getValue())
      {
         biasCompensator.reset();
         eraseBias.set(false);
      }

      // Handle taring of spatial inertias
      if (tare.getValue())
      {
         updateTareSpatialInertias();
         tare.set(false);
      }

      // Main loop of inertial estimator
      if (enableFilter.getValue())
      {
         updateFilterCovariances();
         updateAccelerationCalculationFilterAlphas();

         updateContactJacobians();
         updateContactWrenches();
         updateWholeSystemTorques();

         updateRegressorAndInverseDynamicsModels();
         inverseDynamicsCalculator.compute();
         regressorCalculator.compute(regressorModelBodiesToProcess);

         for (int i = 0; i < regressorModelBodiesToProcess.length; i++)
            regressorBlocks[i].set(regressorCalculator.getJointTorqueRegressorMatrixBlock(regressorModelBodiesToProcess[i]));
         packRegressorFromBlocks(regressorBlocks, basisSets, regressor);

         filter.setTorqueFromNominal(inverseDynamicsCalculator.getJointTauMatrix());
         filter.setRegressor(regressor);
         filter.setContactJacobians(fullContactJacobians);
         filter.setContactWrenches(contactWrenches);

         if (excludeBias.getValue())
            filter.setTorqueFromBias(biasCompensator.getZero());
         else
            filter.setTorqueFromBias(biasCompensator.getBias());

         filter.setNormalizedInnovationThreshold(normalizedInnovationThreshold.getValue());

         estimate.set(filter.calculateEstimate(wholeSystemTorques));
         filteredEstimate.setAndSolve(estimate);
         residual.set(filter.getMeasurementResidual());

         normalizedInnovation.set(filter.getNormalizedInnovation());

         // Pack smoothed estimate back into estimate robot bodies
         RegressorTools.packRigidBodies(basisSets, filteredEstimate, modelHandler.getBodyArray(RobotModelTask.ESTIMATE));

         // Check physical consistency
         checkPhysicalConsistency();

         // Pass through estimates to controller
         if (passThroughEstimatesToController.getValue())
            updateControllerRobotModel();

         updateVisuals();
      }
   }

   private void updateFilterCovariances()
   {
      filter.setProcessCovariance(covarianceHelper.getProcessCovariance());
      filter.setMeasurementCovariance(covarianceHelper.getMeasurementCovariance());
   }

   private void updateAccelerationCalculationFilterAlphas()
   {
      for (FilteredVelocityYoVariable jointAcceleration : jointAccelerations)
         jointAcceleration.setAlpha(accelerationCalculationAlpha.getValue());
   }

   private void updateWholeSystemTorques()
   {
      modelHandler.extractJointsState(RobotModelTask.CONTROLLER, JointStateType.EFFORT, wholeSystemTorques);
   }

   private void updateContactJacobians()
   {
      for (RobotSide side : RobotSide.values)
      {
         compactContactJacobians.get(side).compute();
         jointIndexHandler.compactBlockToFullBlock(legJoints.get(side), compactContactJacobians.get(side).getJacobianMatrix(), fullContactJacobians.get(side));
      }
   }

   private void updateContactWrenches()
   {
      for (RobotSide side : RobotSide.values)
         footSwitches.get(side).getMeasuredWrench().get(contactWrenches.get(side));
   }

   private void updateRegressorAndInverseDynamicsModels()
   {
      modelHandler.copyJointsState(RobotModelTask.CONTROLLER, RobotModelTask.REGRESSOR, JointStateType.CONFIGURATION);
      modelHandler.copyJointsState(RobotModelTask.CONTROLLER, RobotModelTask.REGRESSOR, JointStateType.VELOCITY);
      modelHandler.copyJointsState(RobotModelTask.CONTROLLER, RobotModelTask.INVERSE_DYNAMICS, JointStateType.CONFIGURATION);
      modelHandler.copyJointsState(RobotModelTask.CONTROLLER, RobotModelTask.INVERSE_DYNAMICS, JointStateType.VELOCITY);

      // Update joint accelerations by processing joint velocities
      modelHandler.extractJointsState(RobotModelTask.CONTROLLER, JointStateType.VELOCITY, jointVelocitiesContainer);
      for (int i = 0; i < jointAccelerations.length; ++i)
      {
         jointAccelerations[i].update(jointVelocitiesContainer.get(i));
         // Reuse the joint velocities container to store the joint accelerations
         jointVelocitiesContainer.set(i ,0, jointAccelerations[i].getValue());
      }
      modelHandler.insertJointsState(RobotModelTask.REGRESSOR, JointStateType.ACCELERATION, jointVelocitiesContainer);
      modelHandler.insertJointsState(RobotModelTask.INVERSE_DYNAMICS, JointStateType.ACCELERATION, jointVelocitiesContainer);

      modelHandler.getRobotModel(RobotModelTask.REGRESSOR).updateFrames();
      modelHandler.getRobotModel(RobotModelTask.INVERSE_DYNAMICS).updateFrames();
   }

   private void updateControllerRobotModel()
   {
      baselineCalculator.calculateRateLimitedParameterDeltas(modelHandler.getBodyArray(RobotModelTask.ESTIMATE));
      baselineCalculator.addRateLimitedParameterDeltas(modelHandler.getBodyArray(RobotModelTask.CONTROLLER));
   }

   private void updateTareSpatialInertias()
   {
      baselineCalculator.updateTareSpatialInertias(modelHandler.getBodyArray(RobotModelTask.ESTIMATE));
   }

   private void zeroInverseDynamicsParameters(Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets)
   {
      RigidBodyBasics[] bodies = modelHandler.getBodyArray(RobotModelTask.INVERSE_DYNAMICS);
      for (int i = 0; i < basisSets.length; ++i)
      {
         if (basisSets[i].isEmpty())
            continue;

         for (JointTorqueRegressorCalculator.SpatialInertiaBasisOption option : basisSets[i])
            RigidBodyInertialParametersTools.zeroInertialParameter(bodies[i], option);
      }
   }

   private void packRegressorFromBlocks(DMatrixRMaj[] regressorBlocks, Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets, DMatrixRMaj regressorToPack)
   {
      int regressorBlockIndex = 0;
      int regressorToPackColumnIndex = 0;
      for (Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption> basisSet : basisSets)
      {
         if (basisSet.isEmpty())
            continue;

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

   /**
    * We can only check for physical consistency online, and not *full* physical consistency as the latter requires an expensive eigendecomposition to find the
    * principal moments of inertia.
    */
   private void checkPhysicalConsistency()
   {
      areParametersPhysicallyConsistent.set(true);
      for (RigidBodyBasics estimateModelBody : modelHandler.getBodyArray(RobotModelTask.ESTIMATE))
      {
         if (!RigidBodyInertialParametersTools.isPhysicallyConsistent(estimateModelBody.getInertia()))
         {
            LogTools.error("Inertial parameter estimate for " + estimateModelBody.getName() + " is not physically consistent");
            areParametersPhysicallyConsistent.set(false);
            break;
         }
      }
   }

   private void updateVisuals()
   {
      RigidBodyReadOnly[] controllerModelBodies = modelHandler.getBodyArray(RobotModelTask.CONTROLLER);
      SpatialInertiaReadOnly[] tareSpatialInertias = baselineCalculator.getURDFSpatialInertias();
      for (int i = 0; i < controllerModelBodies.length; i++)
      {
         RigidBodyReadOnly controllerBody = controllerModelBodies[i];
         SpatialInertiaReadOnly tareForBody = tareSpatialInertias[i];

         double scale = EuclidCoreTools.clamp(controllerBody.getInertia().getMass() / tareForBody.getMass() / 2.0, 0.0, 1.0);

         if (controllerBody.getInertia() != null)
            yoInertiaEllipsoids.get(i).update(scale);
      }
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(ellipsoidGraphicGroup);
      return group;
   }
}