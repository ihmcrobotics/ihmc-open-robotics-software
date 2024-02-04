package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.configurations.InertialParameterManagerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.log.LogTools;
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
import us.ihmc.robotics.math.filters.AlphaFilteredYoMatrix;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
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

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class InertialParameterManager implements SCS2YoGraphicHolder
{
   private static final int WRENCH_DIMENSION = 6;

   private final YoBoolean enableFilter;

   private final FullHumanoidRobotModel actualRobotModel;
   private final RigidBodyReadOnly[] actualModelBodies;
   private final List<? extends JointBasics> actualModelJoints;

   private final FullHumanoidRobotModel estimateRobotModel;
   private final RigidBodyBasics[] estimateModelBodies;
   private final ArrayList<YoInertiaEllipsoid> yoInertiaEllipsoids;
   private final YoGraphicDefinition ellipsoidGraphicGroup;

   private final FullHumanoidRobotModel regressorRobotModel;
   private final List<? extends JointBasics> regressorModelJoints;
   private final JointTorqueRegressorCalculator regressorCalculator;

   private final int nDoFs;

   private final AlphaFilteredYoMatrix filteredJointVelocities;
   private final AlphaFilteredYoMatrix doubleFilteredJointVelocities;

   private final DMatrixRMaj rootJointVelocity;
   private final FilteredVelocityYoVariable[] filteredRootJointAccelerations;
   private final AlphaFilteredYoVariable[] doubleFilteredRootJointAccelerations;

   private final AlphaFilteredYoMatrix filteredJointAccelerations;
   private final AlphaFilteredYoMatrix doubleFilteredJointAccelerations;

   private final SideDependentList<? extends FootSwitchInterface> footSwitches;
   private final SideDependentList<DMatrixRMaj> contactWrenches;
   private final SideDependentList<AlphaFilteredYoMatrix> filteredContactWrenches;
   private final SideDependentList<AlphaFilteredYoMatrix> doubleFilteredContactWrenches;

   private final JointIndexHandler jointIndexHandler;
   private final SideDependentList<JointBasics[]> legJoints;
   private final SideDependentList<GeometricJacobian> compactContactJacobians;
   private final SideDependentList<DMatrixRMaj> fullContactJacobians;

   private final DMatrixRMaj wholeSystemTorques;
   private final AlphaFilteredYoMatrix filteredWholeSystemTorques;
   private final AlphaFilteredYoMatrix doubleFilteredWholeSystemTorques;

   private final List<RigidBodyInertialParameters> inertialParameters = new ArrayList<>();
   private final List<YoMatrix> inertialParametersPiBasisWatchers = new ArrayList<>();
   private final List<YoMatrix> inertialParametersThetaBasisWatchers = new ArrayList<>();

   private final Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets;
   private final DMatrixRMaj[] regressorPartitions;
   private final DMatrixRMaj[] parameterPartitions;
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
   private final YoDouble loadedMeasurementCovarianceMultiplier;

   /** DEBUG variables */
   private static final boolean DEBUG = true;
   private final YoMatrix residual;

   public InertialParameterManager(HighLevelHumanoidControllerToolbox toolbox, InertialParameterManagerParameters parameters, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      enableFilter = new YoBoolean("enableFilter", registry);
      enableFilter.set(false);

      actualRobotModel = toolbox.getFullRobotModel();
      actualModelBodies = actualRobotModel.getRootBody().subtreeArray();
      actualModelJoints = actualRobotModel.getRootJoint().subtreeList();

      RigidBodyBasics clonedElevatorForEstimates = MultiBodySystemFactories.cloneMultiBodySystem(actualRobotModel.getElevator(),
                                                                                     actualRobotModel.getModelStationaryFrame(),
                                                                                     "_estimate");
      estimateRobotModel = new FullHumanoidRobotModelWrapper(clonedElevatorForEstimates, true);
      estimateModelBodies = estimateRobotModel.getRootBody().subtreeArray();
      yoInertiaEllipsoids = InertiaVisualizationTools.createYoInertiaEllipsoids(actualRobotModel.getRootBody(), registry);
      ellipsoidGraphicGroup = InertiaVisualizationTools.getInertiaEllipsoidGroup(yoInertiaEllipsoids);


      RigidBodyBasics clonedElevatorForRegressor = MultiBodySystemFactories.cloneMultiBodySystem(actualRobotModel.getElevator(),
                                                                                                 actualRobotModel.getModelStationaryFrame(),
                                                                                                 "_regressor",
                                                                                                 MultiBodySystemFactories.DEFAULT_RIGID_BODY_BUILDER,
                                                                                                 MultiBodySystemFactories.DEFAULT_JOINT_BUILDER);
      regressorRobotModel = new FullHumanoidRobotModelWrapper(clonedElevatorForRegressor, true);
      regressorModelJoints = regressorRobotModel.getRootJoint().subtreeList();
      regressorCalculator = new JointTorqueRegressorCalculator(regressorRobotModel.getElevator());
      regressorCalculator.setGravitationalAcceleration(-toolbox.getGravityZ());

      nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(estimateRobotModel.getRootJoint().subtreeArray());

      basisSets = parameters.getParametersToEstimate();

      filteredJointVelocities = new AlphaFilteredYoMatrix("filteredJointVelocities",
                                                          AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForTorqueFiltering(), toolbox.getControlDT()),
                                                          nDoFs, 1,
                                                          registry);
      doubleFilteredJointVelocities = new AlphaFilteredYoMatrix("doubleFilteredJointVelocities",
                                                                AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForTorqueFiltering(), toolbox.getControlDT()),
                                                                nDoFs, 1,
                                                                registry);

      rootJointVelocity = new DMatrixRMaj(WRENCH_DIMENSION, 1);
      filteredRootJointAccelerations = new FilteredVelocityYoVariable[WRENCH_DIMENSION];
      doubleFilteredRootJointAccelerations = new AlphaFilteredYoVariable[WRENCH_DIMENSION];
      for (int i = 0; i < WRENCH_DIMENSION; ++i)
      {
         filteredRootJointAccelerations[i] = new FilteredVelocityYoVariable("filteredRootJointAcceleration_" + i, null,
                                                                            AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForTorqueFiltering(), toolbox.getControlDT()),
                                                                            toolbox.getControlDT(),
                                                                            registry);
         doubleFilteredRootJointAccelerations[i] = new AlphaFilteredYoVariable("doubleFilteredRootJointAcceleration_" + i, registry,
                                                                               AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForTorqueFiltering(), toolbox.getControlDT()));
      }

      filteredJointAccelerations = new AlphaFilteredYoMatrix("filteredJointAccelerations",
                                                             AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForTorqueFiltering(), toolbox.getControlDT()),
                                                             nDoFs, 1,
                                                             registry);
      doubleFilteredJointAccelerations = new AlphaFilteredYoMatrix("doubleFilteredJointAccelerations",
                                                                   AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForTorqueFiltering(), toolbox.getControlDT()),
                                                                   nDoFs, 1,
                                                                   registry);

      this.footSwitches = toolbox.getFootSwitches();
      contactWrenches = new SideDependentList<>();
      filteredContactWrenches = new SideDependentList<>();
      doubleFilteredContactWrenches = new SideDependentList<>();
      jointIndexHandler = new JointIndexHandler(actualRobotModel.getElevator().subtreeJointStream().toArray(JointBasics[]::new));
      legJoints = new SideDependentList<>();
      compactContactJacobians = new SideDependentList<>();
      fullContactJacobians = new SideDependentList<>();

      // NOTE: for the leg joints and compact jacobians, we use the actual robot model because it has the full model information, including all joint names
      for (RobotSide side : RobotSide.values)
      {
         contactWrenches.put(side, new DMatrixRMaj(WRENCH_DIMENSION, 1));
         filteredContactWrenches.put(side, new AlphaFilteredYoMatrix("filteredContactWrench_" + side,
                                                                     AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForTorqueFiltering(), toolbox.getControlDT()),
                                                                     WRENCH_DIMENSION, 1,
                                                                     registry));
         doubleFilteredContactWrenches.put(side, new AlphaFilteredYoMatrix("doubleFilteredContactWrench_" + side,
                                                                           AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForTorqueFiltering(), toolbox.getControlDT()),
                                                                           WRENCH_DIMENSION, 1,
                                                                           registry));

         legJoints.put(side, MultiBodySystemTools.createJointPath(actualRobotModel.getElevator(), actualRobotModel.getFoot(side)));
         compactContactJacobians.put(side, new GeometricJacobian(legJoints.get(side), footSwitches.get(side).getMeasurementFrame()));
         fullContactJacobians.put(side, new DMatrixRMaj(WRENCH_DIMENSION, nDoFs));
      }

      wholeSystemTorques = new DMatrixRMaj(nDoFs, 1);
      filteredWholeSystemTorques = new AlphaFilteredYoMatrix("filteredWholeSystemTorques",
                                                            AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForTorqueFiltering(), toolbox.getControlDT()),
                                                            nDoFs, 1,
                                                            registry);
      doubleFilteredWholeSystemTorques = new AlphaFilteredYoMatrix("doubleFilteredWholeSystemTorques",
                                                                  AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForTorqueFiltering(), toolbox.getControlDT()),
                                                                  nDoFs, 1,
                                                                  registry);

      int[] partitionSizes = RegressorTools.sizePartitions(basisSets);
      regressorPartitions = RegressorTools.sizePartitionMatrices(regressorCalculator.getJointTorqueRegressorMatrix(), basisSets);
      parameterPartitions = RegressorTools.sizePartitionVectors(basisSets);
      inertialKalmanFilter = new InertialKalmanFilter(estimateRobotModel,
                                                      basisSets,
                                                      parameters.getURDFParameters(basisSets),
                                                      CommonOps_DDRM.identity(partitionSizes[0]),
                                                      CommonOps_DDRM.identity(partitionSizes[0]),
                                                      CommonOps_DDRM.identity(nDoFs));
      inertialKalmanFilterEstimate = new YoMatrix("inertialParameterEstimate",
                                                  partitionSizes[0], 1,
                                                  getRowNames(basisSets, estimateModelBodies), null,
                                                  registry);

      filteredEstimate = new AlphaFilteredYoMatrix("filteredInertialParameterEstimate",
                                                   AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForEstimateFiltering(), toolbox.getControlDT()),
                                                   partitionSizes[0], 1,
                                                   getRowNames(basisSets, estimateModelBodies), null,
                                                   registry);
      doubleFilteredEstimate = new AlphaFilteredYoMatrix("doubleFilteredInertialParameterEstimate",
                                                         AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForEstimateFiltering(), toolbox.getControlDT()),
                                                         partitionSizes[0], 1,
                                                         getRowNames(basisSets, estimateModelBodies), null,
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
      loadedMeasurementCovarianceMultiplier = new YoDouble("loadedMeasurementCovarianceMultiplier", registry);
      loadedMeasurementCovarianceMultiplier.set(parameters.getLoadedMeasurementCovarianceMultiplier());

      if (DEBUG)
      {
         String[] rowNames = new String[nDoFs];
         int index = 0;
         for (JointReadOnly joint : actualModelJoints)
         {
            if (joint.getDegreesOfFreedom() > 1)
            {
               for (int i = 0; i < joint.getDegreesOfFreedom(); i++)
                  rowNames[index + i] = joint.getName() + "_" + i;
            }
            else
            {
               rowNames[index] = joint.getName();
            }
            index += joint.getDegreesOfFreedom();
         }
         residual = new YoMatrix("residual", nDoFs, 1, rowNames, registry);
      }
   }

   public void update()
   {
      if (enableFilter.getBooleanValue())
      {
         updateFilterCovariances();

         updateRegressorModelJointStates();

         updateContactJacobians();
         updateContactWrenches();
         updateWholeSystemTorques();

         regressorCalculator.compute();

         RegressorTools.partitionRegressor(regressorCalculator.getJointTorqueRegressorMatrix(),
                                           basisSets,
                                           regressorPartitions[0],
                                           regressorPartitions[1],
                                           false);
         RegressorTools.partitionVector(regressorCalculator.getParameterVector(),
                                        basisSets,
                                        parameterPartitions[0],
                                        parameterPartitions[1],
                                        false);

         // KF stuff
         inertialKalmanFilter.setRegressorForEstimates(regressorPartitions[0]);
         inertialKalmanFilter.setRegressorForNominal(regressorPartitions[1]);
         inertialKalmanFilter.setParametersForNominal(parameterPartitions[1]);
         inertialKalmanFilter.setContactJacobians(fullContactJacobians);
         inertialKalmanFilter.setContactWrenches(contactWrenches);
         inertialKalmanFilterEstimate.set(inertialKalmanFilter.calculateEstimate(wholeSystemTorques));

         // Pack estimate back into estimate robot bodies
         RegressorTools.packRigidBodies(basisSets, inertialKalmanFilterEstimate, estimateModelBodies);

         filteredEstimate.setAndSolve(inertialKalmanFilterEstimate);
         doubleFilteredEstimate.setAndSolve(filteredEstimate);

         updateVisuals();

         if(DEBUG)
            packDebugResidual();

//         updateWatchers();
      }
   }

   private void updateFilterCovariances()
   {
      // Set diagonal of process covariance
      CommonOps_DDRM.setIdentity(inertialKalmanFilter.getProcessCovariance());
      CommonOps_DDRM.scale(processCovariance.getValue(), inertialKalmanFilter.getProcessCovariance());

      // Set diagonal entries of measurement covariance according to the part of the body
      for (JointReadOnly joint : actualModelJoints)
      {
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
      // Introduce multiplier on the left/right joints depending on whether that side of the robot is loaded
      for (RobotSide side : RobotSide.values)
      {
         if (footSwitches.get(side).hasFootHitGroundFiltered())
         {
            for (JointReadOnly joint : actualModelJoints)
            {
               if (joint.getName().contains(side.name()))
               {
                  int[] indices = jointIndexHandler.getJointIndices(joint);
                  MatrixMissingTools.scaleMatrixDiagonal(indices, loadedMeasurementCovarianceMultiplier.getValue(), inertialKalmanFilter.getMeasurementCovariance());
               }
            }
         }
         // TODO: will need a short circuit swith where we only apply the multiplier to one of the sides, at the moment it is possible to apply the multiplier
         //    to both sides if they are both in contact
      }
   }

   private void updateContactWrenches()
   {
      for (RobotSide side: RobotSide.values)
      {
         if (DEBUG)
         {
            // Check that the reference frames of the contact wrenches and the contact Jacobians match
            ReferenceFrame wrenchBodyFrame = footSwitches.get(side).getMeasuredWrench().getReferenceFrame();
            ReferenceFrame wrenchExpressedInFrame = footSwitches.get(side).getMeasuredWrench().getReferenceFrame();
            ReferenceFrame jacobianFrame =  compactContactJacobians.get(side).getJacobianFrame();
            wrenchBodyFrame.checkReferenceFrameMatch(jacobianFrame);
            wrenchExpressedInFrame.checkReferenceFrameMatch(jacobianFrame);
         }

         footSwitches.get(side).getMeasuredWrench().get(contactWrenches.get(side));

         filteredContactWrenches.get(side).setAndSolve(contactWrenches.get(side));
         doubleFilteredContactWrenches.get(side).setAndSolve(filteredContactWrenches.get(side));
         // Feeding through filtered versions back into contact wrenches for further calculations
         contactWrenches.get(side).set(doubleFilteredContactWrenches.get(side));
      }
   }

   private void updateRegressorModelJointStates()
   {
      for (JointStateType type : JointStateType.values())
      {
         if (type == JointStateType.VELOCITY)
         {
            int velocityIndex = 0;
            // Packing into filtered joint velocities
            for (JointReadOnly joint : actualModelJoints)
            {
               joint.getJointVelocity(velocityIndex, filteredJointVelocities);
               velocityIndex += joint.getDegreesOfFreedom();
            }
            filteredJointVelocities.solve();
            doubleFilteredJointVelocities.setAndSolve(filteredJointVelocities);
            // Now into regressor model joints
            velocityIndex = 0;
            for (JointBasics joint : regressorModelJoints)
            {
               joint.setJointVelocity(velocityIndex, doubleFilteredJointAccelerations);
               velocityIndex += joint.getDegreesOfFreedom();
            }
         }
         if (type == JointStateType.ACCELERATION)
         {
            int accelerationIndex = 0;
            // Packing into filtered joint accelerations
            for (JointReadOnly joint : actualModelJoints)
            {
               joint.getJointAcceleration(accelerationIndex, filteredJointAccelerations);
               accelerationIndex += joint.getDegreesOfFreedom();
            }
            filteredJointAccelerations.solve();
            doubleFilteredJointAccelerations.setAndSolve(filteredJointAccelerations);
            // Now into regressor model joints
            accelerationIndex = 0;
            for (JointBasics joint : regressorModelJoints)
            {
               joint.setJointAcceleration(accelerationIndex, doubleFilteredJointAccelerations);
               accelerationIndex += joint.getDegreesOfFreedom();
            }
         }
         else
         {
            MultiBodySystemTools.copyJointsState(actualModelJoints, regressorModelJoints, type);
         }
      }

      // Do root joint accelerations after as they are handled by a different process
      for (int i = 0; i < WRENCH_DIMENSION; ++i)
      {
         actualModelJoints.get(0).getJointVelocity(0, rootJointVelocity);
         filteredRootJointAccelerations[i].update(rootJointVelocity.get(i));
         doubleFilteredRootJointAccelerations[i].update(filteredRootJointAccelerations[i].getValue());
         // Now into regressor model joints, reuse rootJointVelocity as a container
         rootJointVelocity.set(i, doubleFilteredRootJointAccelerations[i].getValue());
      }
      // TODO: for some reason, doing this tanks performance of the filter. Why?
      regressorModelJoints.get(0).setJointAcceleration(0, rootJointVelocity);

      regressorRobotModel.getRootJoint().updateFramesRecursively();
   }

   private void updateWholeSystemTorques()
   {
      actualRobotModel.getRootJoint().getJointTau(0, wholeSystemTorques);
      for (OneDoFJointReadOnly joint : actualRobotModel.getOneDoFJoints())
      {
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);
         joint.getJointTau(jointIndex, wholeSystemTorques);
      }
      filteredWholeSystemTorques.setAndSolve(wholeSystemTorques);
      doubleFilteredWholeSystemTorques.setAndSolve(filteredWholeSystemTorques);
      // Back into whole system torques
      wholeSystemTorques.set(doubleFilteredWholeSystemTorques);
   }

   private void updateContactJacobians()
   {
      for (RobotSide side : RobotSide.values)
      {
         compactContactJacobians.get(side).compute();
         jointIndexHandler.compactBlockToFullBlock(legJoints.get(side), compactContactJacobians.get(side).getJacobianMatrix(), fullContactJacobians.get(side));
      }
   }

   private void updateVisuals()
   {
      for (int i = 0; i < estimateModelBodies.length; i++)
      {
         RigidBodyReadOnly actualBody = actualModelBodies[i];
         RigidBodyReadOnly estimateBody = estimateModelBodies[i];

         double scale = EuclidCoreTools.clamp(estimateBody.getInertia().getMass() / actualBody.getInertia().getMass()/2.0, 0.0, 1.0);

         if (estimateBody.getInertia() != null && actualBody.getInertia() != null)
         {
            yoInertiaEllipsoids.get(i).update(scale);
         }
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

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(ellipsoidGraphicGroup);
      return group;
   }

   private void packDebugResidual()
   {
      // Size the residual off of the generalized forces container
      DMatrixRMaj residualToPack = new DMatrixRMaj(residual);

      // Start with system torques
      residualToPack.set(wholeSystemTorques);

      // Minus contribution from contact wrenches
      for (RobotSide side : RobotSide.values)
         CommonOps_DDRM.multAddTransA(fullContactJacobians.get(side), contactWrenches.get(side), residualToPack);

      RegressorTools.partitionRegressor(regressorCalculator.getJointTorqueRegressorMatrix(),
                                        basisSets,
                                        regressorPartitions[0],
                                        regressorPartitions[1],
                                        true);
      RegressorTools.partitionVector(regressorCalculator.getParameterVector(),
                                     basisSets,
                                     parameterPartitions[0],
                                     parameterPartitions[1],
                                     true);

      // Iterate over each rigid body and subtract the contribution of its inertial parameters
      for (int i = 0; i < regressorPartitions.length; ++i)
      {
         CommonOps_DDRM.multAdd(-1.0, regressorPartitions[i], parameterPartitions[i], residualToPack);
      }
      residual.set(residualToPack);
   }

   private String[] getRowNames(Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets, RigidBodyReadOnly[] bodies)
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
