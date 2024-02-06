package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.configurations.InertialEstimationParameters;
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

   /** DEBUG variables */
   private static final boolean DEBUG = true;
   private final YoMatrix residual;

   public InertialParameterManager(HighLevelHumanoidControllerToolbox toolbox, InertialEstimationParameters parameters, YoRegistry parentRegistry)
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

      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(estimateRobotModel.getRootJoint().subtreeArray());
      int nOneDoFJoints = estimateRobotModel.getOneDoFJoints().length;

      basisSets = parameters.getParametersToEstimate();

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

      int[] partitionSizes = RegressorTools.sizePartitions(basisSets);
      regressorPartitions = RegressorTools.sizePartitionMatrices(regressorCalculator.getJointTorqueRegressorMatrix(), basisSets);
      parameterPartitions = RegressorTools.sizePartitionVectors(basisSets);
      double postProcessingAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForPostProcessing(), dt);

      inertialKalmanFilter = new InertialKalmanFilter(estimateRobotModel,
                                                      basisSets,
                                                      parameters.getURDFParameters(basisSets),
                                                      CommonOps_DDRM.identity(partitionSizes[0]),
                                                      CommonOps_DDRM.identity(partitionSizes[0]),
                                                      CommonOps_DDRM.identity(nDoFs), postProcessingAlpha,
                                                      registry);
      inertialKalmanFilterEstimate = new YoMatrix("inertialParameterEstimate",
                                                  partitionSizes[0],
                                                  1,
                                                  getRowNamesForEstimates(basisSets, estimateModelBodies),
                                                  null,
                                                  registry);

      double estimateFilteringAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getBreakFrequencyForEstimateFiltering(), dt);
      filteredEstimate = new AlphaFilteredYoMatrix("filteredInertialParameterEstimate", estimateFilteringAlpha,
                                                   partitionSizes[0],
                                                   1,
                                                   getRowNamesForEstimates(basisSets, estimateModelBodies),
                                                   null,
                                                   registry);
      doubleFilteredEstimate = new AlphaFilteredYoMatrix("doubleFilteredInertialParameterEstimate", estimateFilteringAlpha,
                                                         partitionSizes[0],
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

      if (DEBUG)
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
         residual = new YoMatrix("residual", nDoFs, 1, rowNames, registry);
      }
   }

   public void update()
   {
      if (enableFilter.getValue())
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
         RegressorTools.partitionVector(regressorCalculator.getParameterVector(), basisSets, parameterPartitions[0], parameterPartitions[1], false);

         // KF stuff
         inertialKalmanFilter.setRegressorForEstimates(regressorPartitions[0]);
         inertialKalmanFilter.setRegressorForNominal(regressorPartitions[1]);
         inertialKalmanFilter.setParametersForNominal(parameterPartitions[1]);
         inertialKalmanFilter.setContactJacobians(fullContactJacobians);
         inertialKalmanFilter.setContactWrenches(contactWrenches);
         inertialKalmanFilterEstimate.set(inertialKalmanFilter.calculateEstimate(wholeSystemTorques));


         filteredEstimate.setAndSolve(inertialKalmanFilterEstimate);
         doubleFilteredEstimate.setAndSolve(filteredEstimate);
         // Pack smoothed estimate back into estimate robot bodies
         RegressorTools.packRigidBodies(basisSets, doubleFilteredEstimate, estimateModelBodies);

         updateVisuals();

         if (DEBUG)
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
   }

   private void updateContactWrenches()
   {
      for (RobotSide side : RobotSide.values)
      {
         if (DEBUG)
         {
            // Check that the reference frames of the contact wrenches and the contact Jacobians match
            ReferenceFrame wrenchBodyFrame = footSwitches.get(side).getMeasuredWrench().getReferenceFrame();
            ReferenceFrame wrenchExpressedInFrame = footSwitches.get(side).getMeasuredWrench().getReferenceFrame();
            ReferenceFrame jacobianFrame = compactContactJacobians.get(side).getJacobianFrame();
            wrenchBodyFrame.checkReferenceFrameMatch(jacobianFrame);
            wrenchExpressedInFrame.checkReferenceFrameMatch(jacobianFrame);
         }

         footSwitches.get(side).getMeasuredWrench().get(contactWrenches.get(side));
      }
   }

   private void updateRegressorModelJointStates()
   {
      for (JointStateType type : JointStateType.values())
         MultiBodySystemTools.copyJointsState(actualModelJoints, regressorModelJoints, type);

      // Update root joint acceleration, which is not populated by default
      calculateRootJointAccelerations();
      regressorRobotModel.getRootJoint().setJointAcceleration(0, rootJointAcceleration);

      // Update joint accelerations by processing joint velocities
      calculateJointAccelerations();
      for (int i = 0; i < jointAccelerations.length; i++)
         regressorRobotModel.getOneDoFJoints()[i].setQdd(jointAccelerations[i].getValue());

      regressorRobotModel.updateFrames();
   }

   private void updateWholeSystemTorques()
   {
      actualRobotModel.getRootJoint().getJointTau(0, wholeSystemTorques);
      for (OneDoFJointReadOnly joint : actualRobotModel.getOneDoFJoints())
      {
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
      for (int i = 0; i < jointVelocities.length; i++)
      {
         jointVelocities[i].set(actualRobotModel.getOneDoFJoints()[i].getQd());
         jointAccelerations[i].update(jointVelocities[i].getValue());
      }
   }

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
