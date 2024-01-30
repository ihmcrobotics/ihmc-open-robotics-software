package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.configurations.InertialParameterManagerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParameters;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelWrapper;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

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

   private final List<RigidBodyInertialParameters> inertialParameters = new ArrayList<>();
   private final List<YoMatrix> inertialParametersPiBasisWatchers = new ArrayList<>();
   private final List<YoMatrix> inertialParametersThetaBasisWatchers = new ArrayList<>();

   private final Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets;
   private final DMatrixRMaj[] regressorPartitions;
   private final DMatrixRMaj[] parameterPartitions;
   private final InertialKalmanFilter inertialKalmanFilter;
   private final YoMatrix inertialKalmanFilterEstimate;

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

      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(estimateRobotModel.getRootJoint().subtreeArray());

      this.footSwitches = toolbox.getFootSwitches();
      contactWrenches = new SideDependentList<>(new DMatrixRMaj(WRENCH_DIMENSION, 1),
                                                new DMatrixRMaj(WRENCH_DIMENSION, 1));
      jointIndexHandler = new JointIndexHandler(actualRobotModel.getElevator().subtreeJointStream().toArray(JointBasics[]::new));
      legJoints = new SideDependentList<>();
      compactContactJacobians = new SideDependentList<>();
      fullContactJacobians = new SideDependentList<>();
      // NOTE: for the leg joints and compact jacobians, we use the actual robot model because it has the full model information, including all joint names
      for (RobotSide side : RobotSide.values)
      {
         legJoints.set(side, MultiBodySystemTools.createJointPath(actualRobotModel.getElevator(), actualRobotModel.getFoot(side)));
         compactContactJacobians.set(side, new GeometricJacobian(legJoints.get(side), footSwitches.get(side).getMeasurementFrame()));
         fullContactJacobians.set(side, new DMatrixRMaj(6, nDoFs));
      }

      wholeSystemTorques = new DMatrixRMaj(nDoFs, 1);

      basisSets = parameters.getParametersToEstimate();
      regressorPartitions = RegressorTools.sizePartitionMatrices(regressorCalculator.getJointTorqueRegressorMatrix(), basisSets);
      parameterPartitions = RegressorTools.sizePartitionVectors(basisSets);
      inertialKalmanFilter = new InertialKalmanFilter(estimateRobotModel,
                                                      basisSets,
                                                      parameters.getURDFParameters(basisSets),
                                                      CommonOps_DDRM.identity(RegressorTools.sizePartitions(basisSets)[0]),
                                                      CommonOps_DDRM.identity(RegressorTools.sizePartitions(basisSets)[0]),
                                                      CommonOps_DDRM.identity(nDoFs));
      inertialKalmanFilterEstimate = new YoMatrix("inertialParameterEstimate",
                                                  RegressorTools.sizePartitions(basisSets)[0], 1,
                                                  getRowNames(basisSets, estimateModelBodies), null,
                                                  registry);

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
         updateRegressorModelJointStates();

         updateContactJacobians();
         updateContactWrenches();
         updateWholeSystemTorques();

         regressorCalculator.compute();

         // TODO: perhaps a small inner data structure that has a nice interface for a pair of partition matrices/vectors?
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

         updateVisuals();

         if(DEBUG)
            packDebugResidual();

//         updateWatchers();
      }
   }

   private void updateContactWrenches()
   {
      for (RobotSide side: RobotSide.values)
         footSwitches.get(side).getMeasuredWrench().get(contactWrenches.get(side));
   }

   private void updateRegressorModelJointStates()
   {
      for (JointStateType type : JointStateType.values())
         MultiBodySystemTools.copyJointsState(actualModelJoints, regressorModelJoints, type);
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
