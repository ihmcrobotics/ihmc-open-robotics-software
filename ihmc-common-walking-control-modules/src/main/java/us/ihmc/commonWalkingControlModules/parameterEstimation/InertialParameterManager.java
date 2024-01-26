package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.parameterEstimation.ExtendedKalmanFilter;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParameters;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelWrapper;
import us.ihmc.robotics.MatrixMissingTools;
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
   private final DMatrixRMaj regressor;
   private final DMatrixRMaj regressorBlockContainer;

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
   private final DMatrixRMaj inertialParameterPiBasisContainer;
   private final DMatrixRMaj inertialParameterThetaBasisContainer;

   private final List<DMatrixRMaj> parametersFromUrdf = new ArrayList<>();

   private final ArrayList<YoInertiaEllipsoid> yoInertiaEllipsoids;
   private final YoGraphicDefinition ellipsoidGraphicGroup;

   // DEBUG variables
   private static final boolean DEBUG = true;

   private final YoMatrix residual;

   private final InertialExtendedKalmanFilter inertialExtendedKalmanFilter;

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

      yoInertiaEllipsoids = InertiaVisualizationTools.createYoInertiaEllipsoids(actualRobotModel.getRootBody(), registry);
      ellipsoidGraphicGroup = InertiaVisualizationTools.getInertiaEllipsoidGroup(actualRobotModel.getRootBody(), yoInertiaEllipsoids);

      totalNumberOfDoFs = actualRobotModel.getRootJoint().getDegreesOfFreedom() + actualRobotModel.getOneDoFJoints().length;

      regressorCalculator = new JointTorqueRegressorCalculator(estimateRobotModel.getElevator());
      regressorCalculator.setGravitationalAcceleration(-toolbox.getGravityZ());
      regressor = new DMatrixRMaj(totalNumberOfDoFs, RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY * estimateRobotModel.getRootBody().subtreeArray().length);
      regressorBlockContainer = new DMatrixRMaj(totalNumberOfDoFs, RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);

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

      inertialParameterPiBasisContainer = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1);
      inertialParameterThetaBasisContainer = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1);
      for (RigidBodyBasics body : estimateRobotModel.getRootBody().subtreeArray())
      {
         if (body.getInertia() != null)
         {
            inertialParameters.add(new RigidBodyInertialParameters(body.getInertia()));

            inertialParametersPiBasisWatchers.add(new YoMatrix(body.getName() + "_PiBasis", RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1, ParameterRepresentation.getRowNames(ParameterRepresentation.PI_BASIS), registry));
            inertialParametersThetaBasisWatchers.add(new YoMatrix(body.getName() + "_ThetaBasis", RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1, ParameterRepresentation.getRowNames(ParameterRepresentation.THETA_BASIS), registry));

            inertialParameters.get(inertialParameters.size() - 1).getParameterVectorPiBasis(inertialParameterPiBasisContainer);
            inertialParametersPiBasisWatchers.get(inertialParametersPiBasisWatchers.size() - 1).set(inertialParameterPiBasisContainer);
            inertialParameters.get(inertialParameters.size() - 1).getParameterVectorThetaBasis(inertialParameterThetaBasisContainer);
            inertialParametersThetaBasisWatchers.get(inertialParametersThetaBasisWatchers.size() - 1).set(inertialParameterThetaBasisContainer);

            parametersFromUrdf.add(new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1));
            parametersFromUrdf.get(parametersFromUrdf.size() - 1).set(inertialParameterPiBasisContainer);
         }
      }

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
      inertialExtendedKalmanFilter = new InertialExtendedKalmanFilter(bodies, isBodyEstimated);
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

         // Start with system torques
         generalizedForcesContainer.set(wholeSystemTorques);

         // Minus contribution from contact wrenches
         for (RobotSide side : RobotSide.values)
            CommonOps_DDRM.multAddTransA(fullContactJacobians.get(side), contactWrenches.get(side), generalizedForcesContainer);

         regressor.set(regressorCalculator.getJointTorqueRegressorMatrix());

         if(DEBUG)
         {
            DMatrixRMaj residualToPack = new DMatrixRMaj(generalizedForcesContainer);

            // Iterate over each rigid body and subtract the contribution of its inertial parameters
            for (int i = 0; i < inertialParameters.size(); ++i)
            {
               inertialParameters.get(i).getParameterVectorPiBasis(inertialParameterPiBasisContainer);
               MatrixMissingTools.setMatrixBlock(regressorBlockContainer, 0, 0, regressor, 0, i * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, totalNumberOfDoFs, RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1.0);
               CommonOps_DDRM.multAdd(-1.0, regressorBlockContainer, inertialParameterPiBasisContainer, residualToPack);
            }
            residual.set(residualToPack);
         }

         updateWatchers();
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

         double multiplier = random.nextDouble(0.0, 2.0);

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
         double scale = EuclidCoreTools.clamp(estimateBody.getInertia().getMass() / actualBody.getInertia().getMass()/2.0, 0.0, 1.0);

         if (estimateBody.getInertia() != null && actualBody.getInertia() != null)
         {
            yoInertiaEllipsoids.get(i).update(scale);
         }
      }
   }

   private void updateWatchers()
   {
      for (int i = 0; i < inertialParameters.size(); i++)
      {
         inertialParameters.get(i).getParameterVectorPiBasis(inertialParameterPiBasisContainer);
         inertialParametersPiBasisWatchers.get(i).set(inertialParameterPiBasisContainer);

         inertialParameters.get(i).getParameterVectorThetaBasis(inertialParameterThetaBasisContainer);
         inertialParametersThetaBasisWatchers.get(i).set(inertialParameterThetaBasisContainer);
      }
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

   private class InertialExtendedKalmanFilter extends ExtendedKalmanFilter
   {
      /** Process model Jacobian. */
      private final DMatrixRMaj F;
      /** Measurement model Jacobian. */
      private final DMatrixRMaj G;

      /** List of booleans indicating whether the parameters of the corresponding rigid body are being estimated. */
      private final List<Boolean> isBodyEstimated;
      /** List of column vectors containing the inertial parameters in pi basis from the URDF. */
      private final List<DMatrixRMaj> parametersFromUrdf = new ArrayList<>();
      /** Container variable for the inertial parameters in pi basis of one rigid body from the URDF. */
      private final DMatrixRMaj parametersFromUrdfContainer;

      /** Container variable storing a vertical slice of the regressor matrix that corresponds to one rigid body. */
      private final DMatrixRMaj regressorBlockContainer;
      /** Container variable storing the inertial parameters in pi basis of one rigid body. */
      private final DMatrixRMaj parameterContainer;

      /**
       * The EKF API demands that the parameters we're estimating be packed into a single vector. Notably, this means we must omit the parameters we
       * assume known.
       */
      private final DMatrixRMaj consideredParameters;

      private final DMatrixRMaj measurement;

      private final DMatrixRMaj generalizedContactWrenchesContainer;

      public InertialExtendedKalmanFilter(List<RigidBodyReadOnly> bodies, List<Boolean> isBodyEstimated)
      {
         super(countParametersToEstimate(isBodyEstimated), countDoFs(bodies));
         int numberOfParametersToEstimate = countParametersToEstimate(isBodyEstimated);
         int numberOfDoFs = countDoFs(bodies);

         F = CommonOps_DDRM.identity(numberOfParametersToEstimate, numberOfParametersToEstimate);
         G = new DMatrixRMaj(numberOfDoFs, numberOfParametersToEstimate);

         this.isBodyEstimated = isBodyEstimated;
         // TODO: maybe some logic checking that length of bodies and length of isBodyEstimated is the same, or find a better API than two lists
         parametersFromUrdfContainer = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1);
         for (int i = 0; i < bodies.size(); ++i)
            parametersFromUrdf.add(spatialInertiaToParameterVector(bodies.get(i).getInertia()));

         regressorBlockContainer = new DMatrixRMaj(totalNumberOfDoFs, RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
         parameterContainer = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1);

         consideredParameters = new DMatrixRMaj(numberOfParametersToEstimate, 1);

         measurement = new DMatrixRMaj(totalNumberOfDoFs, 1);

         generalizedContactWrenchesContainer = new DMatrixRMaj(totalNumberOfDoFs, 1);
      }

      /**
       * Because {@link #processModel(DMatrixRMaj)} is the identity mapping, the Jacobian of the process model is the identity matrix.
       */
      @Override
      protected DMatrixRMaj linearizeProcessModel(DMatrixRMaj previousState)
      {
         return F;  // In the constructor, F is set to the identity matrix
      }

      /**
       * The measurement Jacobian consists of vertical slices of the regressor that correspond to the bodies we're estimating parameters of, stacked
       * horizontally.
       * <p>
       * For example, if the system under consideration consists of four rigid bodies, then we can slice up Y as: Y = [Y_1, Y_2, Y_3, Y_4], where Y_i is a
       * n x 10 matrix. If we are only estimating the parameters of bodies 1 and 3, then the measurement Jacobian is: G = [Y_1, Y_3].
       * </p>
       */
      @Override
      protected DMatrixRMaj linearizeMeasurementModel(DMatrixRMaj predictedParameters)
      {
         G.zero();

         int index = 0;  // need a separate index to keep track of how many bodies we've packed into G
         for (int i = 0; i < isBodyEstimated.size(); ++i)
         {
            if (isBodyEstimated.get(i))
            {
               packRegressorBlockForBodyIndex(i);  // packs the relevant regressor block into regressorBlockContainer
               packMeasurementModelJacobianForBodyIndex(index);  // packs the relevant regressor block into G, note the use of index rather than i
               index++;
            }
         }
         return G;
      }

      /**
       * This is a wrapper method used in {@link #linearizeProcessModel(DMatrixRMaj)} and {@link #processModel(DMatrixRMaj)} which pulls out the parameters
       * we're estimating from the overall parameter vector
       */
      public DMatrixRMaj packConsideredParameters(DMatrixRMaj parameters)
      {
         int index = 0;
         for (int i = 0; i < isBodyEstimated.size(); ++i)
         {
            if (isBodyEstimated.get(i))
            {
               packParameterVectorForBodyIndex(index);
               index++;
            }
         }
         return parameterContainer;
      }

      /**
       * For inertial parameter estimation, the process model is the identity function. That is, the parameters are assumed to be constant.
       */
      @Override
      protected DMatrixRMaj processModel(DMatrixRMaj parameters)
      {
         return parameters;
      }

      /**
       *
       * @param parameters the state at the current time step.
       * @return
       */
      @Override
      protected DMatrixRMaj measurementModel(DMatrixRMaj parameters)
      {
         measurement.zero();

         // TODO: calling fullContactJacobians and contactWrenches from outer class might be dangerous, but getters are a PITA
         DMatrixRMaj generalizedContactWrenches = calculateGeneralizedContactWrenches(fullContactJacobians, contactWrenches);

         measurement.set(generalizedContactWrenches);

         int index = 0;
         for (int i = 0; i < isBodyEstimated.size(); ++i)
         {
            if (isBodyEstimated.get(i))
            {
               packRegressorBlockForBodyIndex(i);
               packParameterVectorForBodyIndex(index); // note the use of index here
               CommonOps_DDRM.multAdd(regressorBlockContainer, parameterContainer, measurement);
               index++;
            }
            else  // The parameters are assumed known if they are not being estimated
            {
               packRegressorBlockForBodyIndex(i);
               packUrdfParameterVectorForBodyIndex(i);
               CommonOps_DDRM.multAdd(regressorBlockContainer, parameterContainer, measurement);
            }
         }
         return measurement;
      }

      private DMatrixRMaj calculateGeneralizedContactWrenches(SideDependentList<DMatrixRMaj> contactJacobians, SideDependentList<DMatrixRMaj> wrenches)
      {
         generalizedContactWrenchesContainer.zero();
         for (RobotSide side : RobotSide.values)
            CommonOps_DDRM.multAdd(contactJacobians.get(side), wrenches.get(side), generalizedContactWrenchesContainer);
         return generalizedContactWrenchesContainer;
      }

      private DMatrixRMaj spatialInertiaToParameterVector(SpatialInertiaReadOnly spatialInertia)
      {
         double mass = spatialInertia.getMass();
         parametersFromUrdfContainer.set(0, mass);
         // TODO: check whether this is center of mass offset or first mass moment
         parametersFromUrdfContainer.set(1, spatialInertia.getCenterOfMassOffset().getX() * mass);
         parametersFromUrdfContainer.set(2, spatialInertia.getCenterOfMassOffset().getY() * mass);
         parametersFromUrdfContainer.set(3, spatialInertia.getCenterOfMassOffset().getZ() * mass);
         parametersFromUrdfContainer.set(4, spatialInertia.getMomentOfInertia().getM00());
         parametersFromUrdfContainer.set(5, spatialInertia.getMomentOfInertia().getM01());
         parametersFromUrdfContainer.set(6, spatialInertia.getMomentOfInertia().getM11());
         parametersFromUrdfContainer.set(7, spatialInertia.getMomentOfInertia().getM02());
         parametersFromUrdfContainer.set(8, spatialInertia.getMomentOfInertia().getM12());
         parametersFromUrdfContainer.set(9, spatialInertia.getMomentOfInertia().getM22());
         return parametersFromUrdfContainer;
      }

      /**
       * TODO: note this only packs one regressor block at a time into {@code regressorBlockContainer}
       */
      private void packRegressorBlockForBodyIndex(int index)
      {
         CommonOps_DDRM.extract(regressor,0, index * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, regressorBlockContainer);
      }

      private void packMeasurementModelJacobianForBodyIndex(int index)
      {
         CommonOps_DDRM.insert(regressorBlockContainer, G, 0, index * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
      }

      /**
       * TODO: note this only packs one parameter vector at a time into {@code parameterContainer}
       */
      private void packUrdfParameterVectorForBodyIndex(int index)
      {
         CommonOps_DDRM.insert(parametersFromUrdf.get(index), parameterContainer, 0, 0);
      }

      private void packParameterVectorForBodyIndex(int index)
      {
         CommonOps_DDRM.extract(consideredParameters, index * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 0, parameterContainer);
      }

      private static int countParametersToEstimate(List<Boolean> isBodyEstimated)
      {
         int count = 0;
         for (Boolean isBodyEstimatedBoolean : isBodyEstimated)
         {
            if (isBodyEstimatedBoolean)
               count++;
         }
         return RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY * count;
      }

      private static int countDoFs(List<RigidBodyReadOnly> bodies)
      {
         int dofs = 0;
         for (RigidBodyReadOnly body : bodies)
         {
            if (body.getInertia() != null)
               dofs += body.getParentJoint().getDegreesOfFreedom();
         }
         return dofs;
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
