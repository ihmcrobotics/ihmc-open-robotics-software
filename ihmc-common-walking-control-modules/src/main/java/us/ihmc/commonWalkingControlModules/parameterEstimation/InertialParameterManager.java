package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
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
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FullHumanoidRobotModel actualRobotModel;
   private final RigidBodyReadOnly[] actualModelBodies;
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
   private final SideDependentList<DMatrixRMaj> contactWrenches;

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

   private final List<RigidBodyInertialParameters> inertialParameters = new ArrayList<>();
   private final List<YoMatrix> inertialParametersPiBasisWatchers = new ArrayList<>();
   private final List<YoMatrix> inertialParametersThetaBasisWatchers = new ArrayList<>();

   private final Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets;
   private final RigidBodyBasics[] regressorModelBodiesToProcess;
   private final DMatrixRMaj[] regressorBlocks;
   private final DMatrixRMaj regressor;

   private final InertialKalmanFilter inertialKalmanFilter;
   private final YoMatrix inertialKalmanFilterEstimate;

   private final AlphaFilteredYoMatrix filteredEstimate;
   private final AlphaFilteredYoMatrix doubleFilteredEstimate;

   /** Using just one value for process model, applying it to all parameters. */
   private final YoDouble processCovariance;
   private final YoDouble floatingBaseMeasurementCovariance;
   private final YoDouble legsMeasurementCovariance;
   private final YoDouble armsMeasurementCovariance;
   private final YoDouble spineMeasurementCovariance;

   private final YoMatrix residual;

   private final YoBoolean enableBiasCompensator;
   private final InertialBiasCompensator biasCompensator;
   private final DMatrixRMaj bias;
   private final YoBoolean eraseBias;

   public InertialParameterManager(HighLevelHumanoidControllerToolbox toolbox, InertialEstimationParameters parameters, YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      enableFilter = new YoBoolean("enableFilter", registry);
      enableFilter.set(false);

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

      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(estimateRobotModel.getRootJoint().subtreeArray());
      int nOneDoFJoints = estimateRobotModel.getOneDoFJoints().length;
      basisSets = parameters.getParametersToEstimate();
      int[] sizes = RegressorTools.sizePartitions(basisSets);
      int estimateSize = sizes[0];
      int nNonEmptyBasisSets = 0;
      for (Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption> basisSet : basisSets)
         if (!basisSet.isEmpty())
            nNonEmptyBasisSets++;

      regressorBlocks = new DMatrixRMaj[nNonEmptyBasisSets];
      for (int i = 0; i < nNonEmptyBasisSets; i++)
         regressorBlocks[i] = new DMatrixRMaj(nDoFs, RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
      regressor = new DMatrixRMaj(nDoFs, estimateSize);

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
         contactWrenches.put(side, new DMatrixRMaj(WRENCH_DIMENSION, 1));
         legJoints.put(side, MultiBodySystemTools.createJointPath(actualRobotModel.getElevator(), actualRobotModel.getFoot(side)));
         compactContactJacobians.put(side, new GeometricJacobian(legJoints.get(side), footSwitches.get(side).getMeasurementFrame()));
         fullContactJacobians.put(side, new DMatrixRMaj(WRENCH_DIMENSION, nDoFs));
      }

      wholeSystemTorques = new DMatrixRMaj(nDoFs, 1);

      // Check that the reference frames of the contact wrenches and the contact Jacobians match
      // (pretty sure they do)
//      for (RobotSide side : RobotSide.values)
//      {
//         ReferenceFrame wrenchBodyFrame = footSwitches.get(side).getMeasuredWrench().getReferenceFrame();
//         ReferenceFrame wrenchExpressedInFrame = footSwitches.get(side).getMeasuredWrench().getReferenceFrame();
//         ReferenceFrame jacobianFrame = compactContactJacobians.get(side).getJacobianFrame();
//         wrenchBodyFrame.checkReferenceFrameMatch(jacobianFrame);
//         wrenchExpressedInFrame.checkReferenceFrameMatch(jacobianFrame);
//      }

      double dt = toolbox.getControlDT();

      rootJointVelocity = new DMatrixRMaj(WRENCH_DIMENSION, 1);
      rootJointVelocities = new YoDouble[WRENCH_DIMENSION];
      rootJointAccelerations = new FilteredVelocityYoVariable[WRENCH_DIMENSION];
      rootJointAcceleration = new DMatrixRMaj(WRENCH_DIMENSION, 1);
      double accelerationCalculationAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForAccelerationCalculation(), dt);
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

      double postProcessingAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForPostProcessing(), dt);

      inertialKalmanFilter = new InertialKalmanFilter(estimateRobotModel,
                                                      basisSets,
                                                      parameters.getURDFParameters(basisSets),
                                                      CommonOps_DDRM.identity(estimateSize),
                                                      CommonOps_DDRM.identity(estimateSize),
                                                      CommonOps_DDRM.identity(nDoFs), postProcessingAlpha,
                                                      registry);
      inertialKalmanFilterEstimate = new YoMatrix("inertialParameterEstimate",
                                                  estimateSize,
                                                  1,
                                                  getRowNamesForEstimates(basisSets, estimateModelBodies),
                                                  null,
                                                  registry);

      double estimateFilteringAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForEstimateFiltering(), dt);
      filteredEstimate = new AlphaFilteredYoMatrix("filteredInertialParameterEstimate", estimateFilteringAlpha,
                                                   estimateSize,
                                                   1,
                                                   getRowNamesForEstimates(basisSets, estimateModelBodies),
                                                   null,
                                                   registry);
      doubleFilteredEstimate = new AlphaFilteredYoMatrix("doubleFilteredInertialParameterEstimate", estimateFilteringAlpha,
                                                         estimateSize,
                                                         1,
                                                         getRowNamesForEstimates(basisSets, estimateModelBodies),
                                                         null,
                                                         registry);

      processCovariance = new YoDouble("processCovariance", registry);
      processCovariance.set(parameters.getProcessModelCovariance());
      floatingBaseMeasurementCovariance = new YoDouble("floatingBaseMeasurementCovariance", registry);
      floatingBaseMeasurementCovariance.set(parameters.getFloatingBaseMeasurementCovariance());
      legsMeasurementCovariance = new YoDouble("legsMeasurementCovariance", registry);
      legsMeasurementCovariance.set(parameters.getLegMeasurementCovariance());
      armsMeasurementCovariance = new YoDouble("armsMeasurementCovariance", registry);
      armsMeasurementCovariance.set(parameters.getArmMeasurementCovariance());
      spineMeasurementCovariance = new YoDouble("spineMeasurementCovariance", registry);
      spineMeasurementCovariance.set(parameters.getSpineMeasurementCovariance());

      String[] rowNames = getRowNamesForJoints(nDoFs);
      residual = new YoMatrix("residual", nDoFs, 1, rowNames, registry);

      double windowSizeInSeconds = parameters.getBiasCompensationWindowSizeInSeconds();
      int windowSizeInTicks = (int) (windowSizeInSeconds / dt);
      enableBiasCompensator = new YoBoolean("enableBiasCompensator", registry);
      enableBiasCompensator.set(false);
      biasCompensator = new InertialBiasCompensator(nDoFs, windowSizeInTicks, rowNames, registry);
      bias = new DMatrixRMaj(nDoFs, 1);
      eraseBias = new YoBoolean("eraseBias", registry);
      eraseBias.set(false);
   }

   private final ExecutionTimer regressorTimer = new ExecutionTimer("RegressorTimer", registry);

   public void update()
   {
      if (enableFilter.getValue())
      {
         if (enableBiasCompensator.getValue())
            computeTorqueBias();

         if (eraseBias.getValue())
         {
            bias.zero();
            biasCompensator.zero();
            eraseBias.set(false);
         }

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

         // KF stuff
         inertialKalmanFilter.setTorqueFromNominal(inverseDynamicsCalculator.getJointTauMatrix());
         inertialKalmanFilter.setRegressor(regressor);
         inertialKalmanFilter.setContactJacobians(fullContactJacobians);
         inertialKalmanFilter.setContactWrenches(contactWrenches);
         CommonOps_DDRM.addEquals(wholeSystemTorques, bias);  // adding in bias
         inertialKalmanFilterEstimate.set(inertialKalmanFilter.calculateEstimate(wholeSystemTorques));
         inertialKalmanFilter.getMeasurementResidual(residual);

         filteredEstimate.setAndSolve(inertialKalmanFilterEstimate);
         doubleFilteredEstimate.setAndSolve(filteredEstimate);
         // Pack smoothed estimate back into estimate robot bodies
         RegressorTools.packRigidBodies(basisSets, doubleFilteredEstimate, estimateModelBodies);

         updateVisuals();

         //         updateWatchers();
      }
   }

   private void updateFilterCovariances()
   {
      // Set diagonal of process covariance
      CommonOps_DDRM.setIdentity(inertialKalmanFilter.getProcessCovariance());
      CommonOps_DDRM.scale(processCovariance.getValue(), inertialKalmanFilter.getProcessCovariance());

      // Set diagonal entries of measurement covariance according to the part of the body
      for (int i = 0; i < actualModelJoints.size(); ++i)
      {
         JointReadOnly joint = actualModelJoints.get(i);
         int[] indices = jointIndexHandler.getJointIndices(joint);

         if (joint.getName().contains("PELVIS"))
            MatrixMissingTools.setMatrixDiagonal(indices, floatingBaseMeasurementCovariance.getValue(), inertialKalmanFilter.getMeasurementCovariance());
         else if (joint.getName().contains("HIP") || joint.getName().contains("KNEE") || joint.getName().contains("ANKLE"))
            MatrixMissingTools.setMatrixDiagonal(indices, legsMeasurementCovariance.getValue(), inertialKalmanFilter.getMeasurementCovariance());
         else if (joint.getName().contains("SHOULDER") || joint.getName().contains("ELBOW") || joint.getName().contains("WRIST"))
            MatrixMissingTools.setMatrixDiagonal(indices, armsMeasurementCovariance.getValue(), inertialKalmanFilter.getMeasurementCovariance());
         else if (joint.getName().contains("SPINE"))
            MatrixMissingTools.setMatrixDiagonal(indices, spineMeasurementCovariance.getValue(), inertialKalmanFilter.getMeasurementCovariance());
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

   //   private void updateWatchers()
   //   {
   //      for (int i = 0; i < inertialParameters.size(); i++)
   //      {
   //         inertialParameters.get(i).getParameterVectorPiBasis(inertialParameterPiBasisContainer);
   //         inertialParametersPiBasisWatchers.get(i).set(inertialParameterPiBasisContainer);
   //
   //         inertialParameters.get(i).getParameterVectorThetaBasis(inertialParameterThetaBasisContainer);
   //         inertialParametersThetaBasisWatchers.get(i).set(inertialParameterThetaBasisContainer);
   //      }
   //   }

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

   private void computeTorqueBias()
   {
      if (biasCompensator.isWindowFilled())
      {
         biasCompensator.calculateBias();
         for (int i = 0; i < bias.getNumRows(); i++)
         {
            bias.set(i, 0, biasCompensator.getBias(i));
         }
         biasCompensator.resetCounter();
         enableBiasCompensator.set(false);
      }
      else
      {
         for (int j = 0; j < actualModelJoints.size(); ++j)
         {
            JointReadOnly joint = actualModelJoints.get(j);
            int[] indices = jointIndexHandler.getJointIndices(joint);
            for (int k = 0; k < indices.length; ++k)
            {
               biasCompensator.update(indices[k], residual.get(indices[k], 0));
            }
         }
         biasCompensator.incrementCounter();
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

   private enum ParameterRepresentation
   {
      PI_BASIS, THETA_BASIS;

      private static String[] getRowNames(ParameterRepresentation representation)
      {
         String[] rowNames = new String[RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY];
         switch (representation)
         {
            case PI_BASIS:
            {
               rowNames[0] = "m";
               rowNames[1] = "comX";
               rowNames[2] = "comY";
               rowNames[3] = "comZ";
               rowNames[4] = "Ixx";
               rowNames[5] = "Ixy";
               rowNames[6] = "Iyy";
               rowNames[7] = "Ixz";
               rowNames[8] = "Iyz";
               rowNames[9] = "Izz";
               return rowNames;
            }
            case THETA_BASIS:
            {
               rowNames[0] = "alpha";
               rowNames[1] = "dx";
               rowNames[2] = "dy";
               rowNames[3] = "dz";
               rowNames[4] = "sxy";
               rowNames[5] = "sxz";
               rowNames[6] = "syz";
               rowNames[7] = "tx";
               rowNames[8] = "ty";
               rowNames[9] = "tz";
               return rowNames;
            }
            default:
               throw new RuntimeException("Unhandled case: " + representation);
         }
      }
   }
}
