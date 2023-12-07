package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.parameterEstimation.ExtendedKalmanFilter;
import us.ihmc.parameterEstimation.NadiaInertialExtendedKalmanFilterParameters;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParameters;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelWrapper;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.List;

public class InertialParameterManager
{
   private final YoBoolean enableFilter;

   private final FullHumanoidRobotModel actualRobotModel;
   private final List<? extends JointBasics> actualModelJoints;

   private final FullHumanoidRobotModel estimateRobotModel;
   private final List<? extends JointBasics> estimateModelJoints;

   private final int totalNumberOfDoFs;

   private final JointTorqueRegressorCalculator jointTorqueRegressorCalculator;
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

   // DEBUG variables
   private static final boolean DEBUG = true;

   private final YoMatrix residual;

   private final InertialExtendedKalmanFilter inertialExtendedKalmanFilter;

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

      totalNumberOfDoFs = actualRobotModel.getRootJoint().getDegreesOfFreedom() + actualRobotModel.getOneDoFJoints().length;

      jointTorqueRegressorCalculator = new JointTorqueRegressorCalculator(estimateRobotModel.getElevator());
      jointTorqueRegressorCalculator.setGravitationalAcceleration(-toolbox.getGravityZ());
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
      for (RigidBodyBasics body : estimateRobotModel.getElevator().subtreeList())
      {
         if (body.getInertia() != null)
         {
            inertialParameters.add(new RigidBodyInertialParameters(body.getInertia()));

            inertialParametersPiBasisWatchers.add(new YoMatrix(body.getName() + "_PiBasis", RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1, getRowNames(ParameterRepresentation.PI_BASIS), registry));
            inertialParametersThetaBasisWatchers.add(new YoMatrix(body.getName() + "_ThetaBasis", RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1, getRowNames(ParameterRepresentation.THETA_BASIS), registry));

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

      inertialExtendedKalmanFilter = new InertialExtendedKalmanFilter(0, totalNumberOfDoFs);
   }

   public void update()
   {
      if (enableFilter.getBooleanValue())
      {
         updateEstimatedModelJointState();

         updateContactJacobians();
         updateContactWrenches();
         updateWholeSystemTorques();

         jointTorqueRegressorCalculator.compute();

         // Start with system torques
         generalizedForcesContainer.set(wholeSystemTorques);

         // Minus contribution from contact wrenches
         for (RobotSide side : RobotSide.values)
            CommonOps_DDRM.multAddTransA(fullContactJacobians.get(side), contactWrenches.get(side), generalizedForcesContainer);

         regressor.set(jointTorqueRegressorCalculator.getJointTorqueRegressorMatrix());

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
      PI_BASIS, THETA_BASIS
   }

   private String[] getRowNames(ParameterRepresentation representation)
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

   private class InertialExtendedKalmanFilter extends ExtendedKalmanFilter
   {
      /** Process model Jacobian */
      private final DMatrixRMaj F;
      private final DMatrixRMaj G;

      private final DMatrixRMaj generalizedTorques;
      private final DMatrixRMaj generalizedContactWrenches;

      private final List<Boolean> isRigidBodyBeingEstimated;
      private final List<DMatrixRMaj> parametersFromUrdf;

      private final DMatrixRMaj parameterContainer;
      private final DMatrixRMaj measurement;

      public InertialExtendedKalmanFilter(int numberOfParametersToEstimate, int totalNumberOfDoFs)
      {
         super(numberOfParametersToEstimate, totalNumberOfDoFs);

         F = CommonOps_DDRM.identity(numberOfParametersToEstimate, numberOfParametersToEstimate);
         G = new DMatrixRMaj(totalNumberOfDoFs, numberOfParametersToEstimate);

         measurement = new DMatrixRMaj(totalNumberOfDoFs, 1);

         generalizedTorques = new DMatrixRMaj(totalNumberOfDoFs, 1);
         generalizedContactWrenches = new DMatrixRMaj(totalNumberOfDoFs, 1);
      }

      @Override
      protected DMatrixRMaj linearizeProcessModel(DMatrixRMaj previousState)
      {
         return F;  // In the constructor, F is set to the identity matrix
      }

      @Override
      protected DMatrixRMaj linearizeMeasurementModel(DMatrixRMaj predictedParameters)
      {
         G.zero();
         for (int i = 0; i < isRigidBodyBeingEstimated.size(); ++i)
         {
            if (isRigidBodyBeingEstimated.get(i))
            {
               MatrixMissingTools.setMatrixBlock(regressorBlockContainer, 0, 0, regressor, 0, i * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, totalNumberOfDoFs, RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1.0);
               MatrixMissingTools.setMatrixBlock(G, 0, i * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, regressorBlockContainer, 0, 0, totalNumberOfDoFs, RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1.0);
            }
         }
      }

      @Override
      protected DMatrixRMaj processModel(DMatrixRMaj parameters)
      {
         return parameters;
      }

      /**
       * Ensure that {@link #setTorquesContribution(DMatrixRMaj)} and {@link #setContactWrenchesContribution(DMatrixRMaj)} are called first.
       */
      @Override
      protected DMatrixRMaj measurementModel(DMatrixRMaj parameters)
      {
         // Subtract the contribution of the contact wrenches from the generalized torques
         measurement.set(generalizedTorques);
         CommonOps_DDRM.subtractEquals(generalizedTorques, generalizedContactWrenches);

         int bodiesToEstimateIndex = 0;

         // Looping over a list of length equal to the number of rigid bodies. If the rigid body is being estimated
         for (int i = 0; i < isRigidBodyBeingEstimated.size(); ++i)
         {
            MatrixMissingTools.setMatrixBlock(regressorBlockContainer, 0, 0, regressor, 0, i * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, totalNumberOfDoFs, RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1.0);

            if (isRigidBodyBeingEstimated.get(i))
            {
               MatrixMissingTools.setMatrixBlock(parameterContainer, 0, 0, parameters, bodiesToEstimateIndex * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 0, RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1, 1.0);
               CommonOps_DDRM.multAdd(-1.0, regressorBlockContainer, parameterContainer, measurement);
               bodiesToEstimateIndex += 1;
            }
            else
            {
               CommonOps_DDRM.multAdd(-1.0, regressorBlockContainer, parametersFromUrdf.get(i), measurement);
            }
         }
         return measurement;
      }

      public void setTorquesContribution(DMatrixRMaj torquesContribution)
      {
         generalizedTorques.set(torquesContribution);
      }

      public void setContactWrenchesContribution(DMatrixRMaj contactWrenchesContribution)
      {
         generalizedContactWrenches.set(contactWrenchesContribution);
      }
   }
}
