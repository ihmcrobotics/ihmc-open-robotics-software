package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.parameterEstimation.NadiaInertialExtendedKalmanFilterParameters;
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
import java.util.Random;

public class InertialParameterManager implements SCS2YoGraphicHolder
{
   private final YoBoolean enableFilter;

   private final FullHumanoidRobotModel actualRobotModel;
   private final List<? extends JointBasics> actualModelJoints;

   private final FullHumanoidRobotModel estimateRobotModel;
   private final List<? extends JointBasics> estimateModelJoints;

   private final int totalNumberOfDoFs;

   private final JointTorqueRegressorCalculator regressorCalculator;

   private final SideDependentList<? extends FootSwitchInterface> footSwitches;
   private final SideDependentList<DMatrixRMaj> contactWrenches;

   private final JointIndexHandler jointIndexHandler;
   private final SideDependentList<JointBasics[]> legJoints;
   private final SideDependentList<GeometricJacobian> compactContactJacobians;
   private final SideDependentList<DMatrixRMaj> fullContactJacobians;

   private final DMatrixRMaj wholeSystemTorques;

   /** What we do all the math in */
   private final DMatrixRMaj generalizedForcesContainer;

   private final List<RigidBodyInertialParameters> inertialParameters = new ArrayList<>();
   private final List<YoMatrix> inertialParametersPiBasisWatchers = new ArrayList<>();
   private final List<YoMatrix> inertialParametersThetaBasisWatchers = new ArrayList<>();

   private final List<DMatrixRMaj> parametersFromUrdf = new ArrayList<>();

   private final ArrayList<YoInertiaEllipsoid> yoInertiaEllipsoids;
   private final YoGraphicDefinition ellipsoidGraphicGroup;

   private final DMatrixRMaj[] regressorPartitions;
   private final DMatrixRMaj[] parameterPartitions;

   // DEBUG variables
   private static final boolean DEBUG = true;

   private final YoMatrix residual;

   private final InertialKalmanFilter inertialKalmanFilter;

   private final Random random = new Random(4567L);  // TODO: remove eventually

   public InertialParameterManager(HighLevelHumanoidControllerToolbox toolbox, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      enableFilter = new YoBoolean("enableFilter", registry);
      enableFilter.set(false);

      actualRobotModel = toolbox.getFullRobotModel();
      actualModelJoints = actualRobotModel.getRootJoint().subtreeList();

      RigidBodyBasics clonedElevator = MultiBodySystemFactories.cloneMultiBodySystem(actualRobotModel.getElevator(),
                                                                                     actualRobotModel.getModelStationaryFrame(),
                                                                                     "_estimate");
      estimateRobotModel = new FullHumanoidRobotModelWrapper(clonedElevator, true);
      estimateModelJoints = estimateRobotModel.getRootJoint().subtreeList();

      yoInertiaEllipsoids = InertiaVisualizationTools.createYoInertiaEllipsoids(estimateRobotModel.getRootBody(), registry);
      ellipsoidGraphicGroup = InertiaVisualizationTools.getInertiaEllipsoidGroup(actualRobotModel.getRootBody(), yoInertiaEllipsoids);

      totalNumberOfDoFs = actualRobotModel.getRootJoint().getDegreesOfFreedom() + actualRobotModel.getOneDoFJoints().length;

      regressorCalculator = new JointTorqueRegressorCalculator(estimateRobotModel.getElevator());
      regressorCalculator.setGravitationalAcceleration(-toolbox.getGravityZ());

      this.footSwitches = toolbox.getFootSwitches();
      contactWrenches = new SideDependentList<>(new DMatrixRMaj(6 ,1),
                                                new DMatrixRMaj(6, 1));  // TODO(jfoster): magic numbers

      jointIndexHandler = new JointIndexHandler(actualRobotModel.getElevator().subtreeJointStream().toArray(JointBasics[]::new));
      legJoints = new SideDependentList<>();
      compactContactJacobians = new SideDependentList<>();
      fullContactJacobians = new SideDependentList<>();
      // NOTE: for the leg joints and compact jacobians, we use the actual robot model because it has the full model information, including all joint names
      for (RobotSide side : RobotSide.values)
      {
         legJoints.set(side, MultiBodySystemTools.createJointPath(actualRobotModel.getElevator(), actualRobotModel.getFoot(side)));
         compactContactJacobians.set(side, new GeometricJacobian(legJoints.get(side), footSwitches.get(side).getMeasurementFrame()));
         fullContactJacobians.set(side, new DMatrixRMaj(6, totalNumberOfDoFs));
      }

      wholeSystemTorques = new DMatrixRMaj(totalNumberOfDoFs, 1);

      generalizedForcesContainer = new DMatrixRMaj(totalNumberOfDoFs, 1);

      regressorPartitions = RegressorTools.sizePartitionMatrices(regressorCalculator.getJointTorqueRegressorMatrix(),
                                                                 NadiaInertialExtendedKalmanFilterParameters.inertialParametersToEstimateHandsMass(estimateRobotModel));
      parameterPartitions = RegressorTools.sizePartitionVectors(NadiaInertialExtendedKalmanFilterParameters.inertialParametersToEstimateHandsMass(estimateRobotModel));

      if (DEBUG)
      {
         String[] rowNames = new String[totalNumberOfDoFs];
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
         residual = new YoMatrix("residual", totalNumberOfDoFs, 1, rowNames, registry);
      }

      List<RigidBodyReadOnly> bodies = new ArrayList<>();
      List<Boolean> isBodyEstimated = new ArrayList<>();
      for (RigidBodyReadOnly body : estimateRobotModel.getRootBody().subtreeArray())
      {
         if (body.getInertia() != null)
         {
            bodies.add(body);
            isBodyEstimated.add(false);
         }
      }

      // TODO: don't know if we're using the right robot model here
      inertialKalmanFilter = new InertialKalmanFilter(estimateRobotModel, NadiaInertialExtendedKalmanFilterParameters.inertialParametersToEstimateHandsMass(estimateRobotModel));
   }

   public void update()
   {
      if (enableFilter.getBooleanValue())
      {
         updateEstimatedModelJointState();

         updateContactJacobians();
         updateContactWrenches();
         updateWholeSystemTorques();

         setDummyEstimateBodies();
         updateVisuals();

         regressorCalculator.compute();

         if(DEBUG)
         {
            // Size the residual off of the generalized forces container
            DMatrixRMaj residualToPack = new DMatrixRMaj(generalizedForcesContainer);

            // Start with system torques
            generalizedForcesContainer.set(wholeSystemTorques);

            // Minus contribution from contact wrenches
            for (RobotSide side : RobotSide.values)
               CommonOps_DDRM.multAddTransA(fullContactJacobians.get(side), contactWrenches.get(side), generalizedForcesContainer);

            RegressorTools.partitionRegressor(regressorCalculator.getJointTorqueRegressorMatrix(),
                                              NadiaInertialExtendedKalmanFilterParameters.inertialParametersToEstimateHandsMass(estimateRobotModel),
                                              regressorPartitions[0],
                                              regressorPartitions[1],
                                              true);
            RegressorTools.partitionVector(regressorCalculator.getParameterVector(),
                                           NadiaInertialExtendedKalmanFilterParameters.inertialParametersToEstimateHandsMass(estimateRobotModel),
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

//         updateWatchers();
      }
   }

   private void updateContactWrenches()
   {
      for (RobotSide side: RobotSide.values)
         footSwitches.get(side).getMeasuredWrench().get(contactWrenches.get(side));
   }

   private void updateEstimatedModelJointState()
   {
      for (JointStateType type : JointStateType.values())
         MultiBodySystemTools.copyJointsState(actualModelJoints, estimateModelJoints, type);
      estimateRobotModel.getRootJoint().updateFramesRecursively();
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

   private void setDummyEstimateBodies()
   {
      for (int i = 0; i < estimateRobotModel.getRootBody().subtreeArray().length; i++)
      {
         RigidBodyReadOnly actualBody = actualRobotModel.getRootBody().subtreeArray()[i];
         RigidBodyBasics estimateBody = estimateRobotModel.getRootBody().subtreeArray()[i];

         double multiplier = random.nextDouble(0.9, 1.1);

         estimateBody.getInertia().setMass(actualBody.getInertia().getMass() * multiplier);

         // TODO: print out only the pelvis mass to show that randomisation is working
         if (i == 0)
            System.out.println("Pelvis mass: " + estimateBody.getInertia().getMass());
      }
   }

   private void updateVisuals()
   {
      for (int i = 0; i < estimateRobotModel.getRootBody().subtreeArray().length; i++)
      {
         RigidBodyReadOnly actualBody = actualRobotModel.getRootBody().subtreeArray()[i];
         RigidBodyReadOnly estimateBody = estimateRobotModel.getRootBody().subtreeArray()[i];
         // TODO: Verify this is working when the EKF is plugged in. Right now estimateBody
         double scale = EuclidCoreTools.clamp(estimateBody.getInertia().getMass() / actualBody.getInertia().getMass(), 0.0, 1.0);

         if (estimateBody.getInertia() != null && actualBody.getInertia() != null)
         {
            InertiaVisualizationTools.updateEllipsoid(yoInertiaEllipsoids.get(i), scale);
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

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(ellipsoidGraphicGroup);
      return group;
   }
}
