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

public class InertialParameterManager implements SCS2YoGraphicHolder
{
   private static final int WRENCH_DIMENSION = 6;

   private final YoBoolean enableFilter;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final int nDoFs;
   private final int nParameters;
   private final int nBodies;

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

   private final YoBoolean calculateBias;
   private final InertialBiasCompensator biasCompensator;
   private final YoBoolean excludeBias;
   private final YoBoolean eraseBias;

   private final InertialEstimationParameters parameters;
   private final YoDouble accelerationCalculationAlpha;  // useful to have a master setting, we want to filter all DoFs equally

   private final YoDouble normalizedInnovation;
   private final YoDouble normalizedInnovationThreshold;

   private final YoBoolean passThroughEstimatesToController;

   private final YoBoolean tare;

   private final YoBoolean areParametersPhysicallyConsistent;

   private final MultipleHumanoidModelHandler<Task> modelHandler;
   private final HumanoidModelCovarianceHelper covarianceHelper;
   private final InertialBaselineCalculator baselineCalculator;

   public InertialParameterManager(HighLevelHumanoidControllerToolbox toolbox, InertialEstimationParameters inertialEstimationParameters, YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      this.parameters = inertialEstimationParameters;
      basisSets = parameters.getBasisSets();

      enableFilter = new YoBoolean("enableFilter", registry);
      enableFilter.set(false);

      modelHandler = new MultipleHumanoidModelHandler<>(Task.class);
      FullHumanoidRobotModel actualRobotModel = toolbox.getFullRobotModel();
      modelHandler.addRobotModel(Task.ACTUAL, actualRobotModel);

      for (Task task : new Task[] {Task.ESTIMATE, Task.INVERSE_DYNAMICS, Task.REGRESSOR})
      {
         RigidBodyBasics clonedElevator = MultiBodySystemFactories.cloneMultiBodySystem(actualRobotModel.getElevator(),
                                                                                       actualRobotModel.getModelStationaryFrame(),
                                                                                       "_" + task.name());
         FullHumanoidRobotModel clonedRobotModel = new FullHumanoidRobotModelWrapper(clonedElevator, false);
         modelHandler.addRobotModel(task, clonedRobotModel);
      }

      covarianceHelper = new HumanoidModelCovarianceHelper(actualRobotModel, parameters, registry);
      baselineCalculator = new InertialBaselineCalculator(modelHandler.getRobotModel(Task.ESTIMATE), parameters, 1.0, registry);

      yoInertiaEllipsoids = InertiaVisualizationTools.createYoInertiaEllipsoids(actualRobotModel.getRootBody(), registry);
      ellipsoidGraphicGroup = InertiaVisualizationTools.getInertiaEllipsoidGroup(yoInertiaEllipsoids);

      inverseDynamicsCalculator = new InverseDynamicsCalculator(modelHandler.getRobotModel(Task.INVERSE_DYNAMICS).getElevator());
      inverseDynamicsCalculator.setGravitationalAcceleration(-toolbox.getGravityZ());
      zeroInverseDynamicsParameters(basisSets);
      regressorCalculator = new JointTorqueRegressorCalculator(modelHandler.getRobotModel(Task.REGRESSOR).getElevator());
      regressorCalculator.setGravitationalAcceleration(-toolbox.getGravityZ());

      nBodies = actualRobotModel.getRootBody().subtreeArray().length;
      nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(actualRobotModel.getRootJoint().subtreeArray());
      nParameters = parameters.getNumberOfParameters();
      int nNonEmptyBasisSets = parameters.getNumberOfNonEmptyBasisSets();

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

         regressorModelBodiesToProcess[regressorIndex] = modelHandler.getBodyArray(Task.REGRESSOR)[i];
         regressorIndex++;
      }

      this.footSwitches = toolbox.getFootSwitches();
      contactWrenches = new SideDependentList<>();
      jointIndexHandler = new JointIndexHandler(actualRobotModel.getRootJoint().subtreeArray());
      legJoints = new SideDependentList<>();
      compactContactJacobians = new SideDependentList<>();
      fullContactJacobians = new SideDependentList<>();
      // NOTE: for the leg joints and compact jacobians, we use the actual robot model because it has the full model information, including all joint names
      for (RobotSide side : RobotSide.values)
      {
         contactWrenches.put(side, new DMatrixRMaj(WRENCH_DIMENSION, 1));
         legJoints.put(side, MultiBodySystemTools.createJointPath(actualRobotModel.getElevator(), actualRobotModel.getFoot(side)));
         compactContactJacobians.put(side, new GeometricJacobian(legJoints.get(side), footSwitches.get(side).getMeasurementFrame()));
         fullContactJacobians.put(side, new DMatrixRMaj(WRENCH_DIMENSION, nDoFs));
      }

      wholeSystemTorques = new DMatrixRMaj(nDoFs, 1);

      double dt = toolbox.getControlDT();
      double defaultAccelerationCalculationAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForAccelerationCalculation(), dt);
      accelerationCalculationAlpha = new YoDouble("accelerationCalculationAlpha", registry);
      accelerationCalculationAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForAccelerationCalculation(), dt));
      jointVelocitiesContainer = new DMatrixRMaj(nDoFs, 1);
      jointAccelerations = new FilteredVelocityYoVariable[nDoFs];
      for (JointReadOnly joint : modelHandler.getJointList(Task.ACTUAL))
      {
         if (joint instanceof OneDoFJointReadOnly)
         {
            int jointIndex = jointIndexHandler.getOneDoFJointIndex((OneDoFJointReadOnly) joint);
            jointAccelerations[jointIndex] = new FilteredVelocityYoVariable("jointAcceleration_" + joint.getName(), "", defaultAccelerationCalculationAlpha, dt, registry);
         }
         else
         {
            int[] jointIndices = jointIndexHandler.getJointIndices(joint);
            String[] rootJointNames = getNamesForRootJoint((SixDoFJointReadOnly) joint);
            for (int jointIndex : jointIndices)
               jointAccelerations[jointIndex] = new FilteredVelocityYoVariable("jointAcceleration_" + rootJointNames[jointIndex], "", defaultAccelerationCalculationAlpha, dt, registry);
         }
      }

      estimate = new YoMatrix("", nParameters, 1, getRowNamesForEstimates(basisSets, modelHandler.getBodyArray(Task.ESTIMATE)), null, registry);
      double defaultEstimateFilteringAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForEstimateFiltering(), dt);
      filteredEstimate = new AlphaFilteredYoMatrix("filtered_", defaultEstimateFilteringAlpha, nParameters, 1, getRowNamesForEstimates(basisSets, modelHandler.getBodyArray(Task.ESTIMATE)), null, registry);

      String[] rowNames = getRowNamesForJoints(nDoFs);
      residual = new YoMatrix("residual", nDoFs, 1, rowNames, registry);

      int windowSizeInTicks = (int) (parameters.getBiasCompensationWindowSizeInSeconds() / dt);
      calculateBias = new YoBoolean("calculateBias", registry);
      biasCompensator = new InertialBiasCompensator(nDoFs, windowSizeInTicks, getRowNamesForJoints(nDoFs), registry);
      excludeBias = new YoBoolean("excludeBias", registry);
      excludeBias.set(false);
      eraseBias = new YoBoolean("eraseBias", registry);
      eraseBias.set(false);

      normalizedInnovation = new YoDouble("normalizedInnovation", registry);
      normalizedInnovation.set(0.0);
      normalizedInnovationThreshold = new YoDouble("normalizedInnovationThreshold", registry);
      normalizedInnovationThreshold.set(parameters.getNormalizedInnovationThreshold());

      // Construct the type of filter used based on enum value
      switch (parameters.getTypeOfEstimatorToUse())
      {
         case KF -> filter = new InertialKalmanFilter(modelHandler.getRobotModel(Task.ESTIMATE), parameters, registry);
         case PHYSICALLY_CONSISTENT_EKF -> filter = new InertialPhysicallyConsistentKalmanFilter(modelHandler.getRobotModel(Task.ESTIMATE), parameters, registry);
      }

      passThroughEstimatesToController = new YoBoolean("passThroughEstimatesToController", registry);
      passThroughEstimatesToController.set(false);

      tare = new YoBoolean("tare", registry);
      tare.set(false);

      areParametersPhysicallyConsistent = new YoBoolean("areParametersPhysicallyConsistent", registry);
   }

   private enum Task
   {
      ACTUAL, ESTIMATE, INVERSE_DYNAMICS, REGRESSOR;

      public static final Task[] values = values();
   }

   public void update()
   {
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

         updateAccelerationCalculationFilterAlphas();

         updateModelJointStates();

         updateContactJacobians();
         updateContactWrenches();
         updateWholeSystemTorques();

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
         residual.set(filter.getMeasurementResidual());
         filteredEstimate.setAndSolve(estimate);

         normalizedInnovation.set(filter.getNormalizedInnovation());

         // Pack smoothed estimate back into estimate robot bodies
         RegressorTools.packRigidBodies(basisSets, filteredEstimate, modelHandler.getBodyArray(Task.ESTIMATE));

         // Check physical consistency
         checkPhysicalConsistency();

         // Pass through estimates to controller
         if (passThroughEstimatesToController.getValue())
            updateActualRobotModel();

         updateVisuals();
      }
   }

   /**
    * We can only check for physical consistency online, and not *full* physical consistency as the latter requires an expensive eigendecomposition to find the
    * principal moments of inertia.
    */
   private void checkPhysicalConsistency()
   {
      areParametersPhysicallyConsistent.set(true);
      for (RigidBodyBasics estimateModelBody : modelHandler.getBodyArray(Task.ESTIMATE))
      {
         if (!RigidBodyInertialParametersTools.isPhysicallyConsistent(estimateModelBody.getInertia()))
         {
            LogTools.error("Inertial parameter estimate for " + estimateModelBody.getName() + " is not physically consistent");
            areParametersPhysicallyConsistent.set(false);
            break;
         }
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

   private void updateModelJointStates()
   {
      modelHandler.copyJointsState(Task.ACTUAL, Task.REGRESSOR, JointStateType.CONFIGURATION);
      modelHandler.copyJointsState(Task.ACTUAL, Task.REGRESSOR, JointStateType.VELOCITY);

      modelHandler.copyJointsState(Task.ACTUAL, Task.INVERSE_DYNAMICS, JointStateType.CONFIGURATION);
      modelHandler.copyJointsState(Task.ACTUAL, Task.INVERSE_DYNAMICS, JointStateType.VELOCITY);

      // Update joint accelerations by processing joint velocities
      modelHandler.extractJointsState(Task.ACTUAL, JointStateType.VELOCITY, jointVelocitiesContainer);
      for (int i = 0; i < jointAccelerations.length; ++i)
      {
         jointAccelerations[i].update(jointVelocitiesContainer.get(i));
         // Reuse the joint velocities container to store the joint accelerations
         jointVelocitiesContainer.set(i ,0, jointAccelerations[i].getValue());
      }
      modelHandler.insertJointsState(Task.REGRESSOR, JointStateType.ACCELERATION, jointVelocitiesContainer);
      modelHandler.insertJointsState(Task.INVERSE_DYNAMICS, JointStateType.ACCELERATION, jointVelocitiesContainer);

      modelHandler.getRobotModel(Task.REGRESSOR).updateFrames();
      modelHandler.getRobotModel(Task.INVERSE_DYNAMICS).updateFrames();
   }

   private void updateWholeSystemTorques()
   {
      modelHandler.extractJointsState(Task.ACTUAL, JointStateType.EFFORT, wholeSystemTorques);
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

   private void zeroInverseDynamicsParameters(Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets)
   {
      RigidBodyBasics[] bodies = modelHandler.getBodyArray(Task.INVERSE_DYNAMICS);
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
      baselineCalculator.calculateRateLimitedParameterDeltas(modelHandler.getBodyArray(Task.ESTIMATE));
      baselineCalculator.addRateLimitedParameterDeltas(modelHandler.getBodyArray(Task.ACTUAL));
   }

   private void updateTareSpatialInertias()
   {
      baselineCalculator.updateTareSpatialInertias(modelHandler.getBodyArray(Task.ESTIMATE));
   }

   private void updateVisuals()
   {
      RigidBodyBasics[] estimateModelBodies = modelHandler.getBodyArray(Task.ESTIMATE);
      RigidBodyReadOnly[] actualModelBodies = modelHandler.getBodyArray(Task.ACTUAL);
      for (int i = 0; i < nBodies; i++)
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

   private String[] getNamesForRootJoint(SixDoFJointReadOnly rootJoint)
   {
      List<String> jointNames = new ArrayList<>();
      String name = rootJoint.getName();
      String[] suffixes = new String[] {"wX", "wY", "wZ", "x", "y", "z"};
      for (String suffix : suffixes)
         jointNames.add(name + "_" + suffix);

      return jointNames.toArray(new String[0]);
   }

   private String[] getRowNamesForJoints(int nDoFs)
   {
      List<? extends JointBasics> actualModelJoints = modelHandler.getJointList(Task.ACTUAL);  // TODO: probably shouldn't be herer
      String[] rowNames = new String[nDoFs];
      int index = 0;
      for (JointReadOnly joint : actualModelJoints)
      {
         if (joint.getDegreesOfFreedom() > 1)
         {
            String[] rootJointNames = getNamesForRootJoint((SixDoFJointReadOnly) joint);
            for (int i = 0; i < joint.getDegreesOfFreedom(); i++)
               rowNames[index + i] = joint.getName() + "_" + rootJointNames[i];
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
