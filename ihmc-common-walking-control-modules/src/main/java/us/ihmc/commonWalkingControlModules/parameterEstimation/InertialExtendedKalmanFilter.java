package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.parameterEstimation.ExtendedKalmanFilter;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;

class InertialExtendedKalmanFilter extends ExtendedKalmanFilter
{
   /**
    * Process model Jacobian.
    */
   private final DMatrixRMaj F;
   /**
    * Measurement model Jacobian.
    */
   private final DMatrixRMaj G;

   /**
    * List of booleans indicating whether the parameters of the corresponding rigid body are being estimated.
    */
   private final List<Boolean> isBodyEstimated;
   /**
    * List of column vectors containing the inertial parameters in pi basis from the URDF.
    */
   private final List<DMatrixRMaj> parametersFromUrdf = new ArrayList<>();
   /**
    * Container variable for the inertial parameters in pi basis of one rigid body from the URDF.
    */
   private final DMatrixRMaj parametersFromUrdfContainer;

   /**
    * Container variable storing a vertical slice of the regressor matrix that corresponds to one rigid body.
    */
   private final DMatrixRMaj regressorBlockContainer = new DMatrixRMaj();
   /**
    * Container variable storing the inertial parameters in pi basis of one rigid body.
    */
   private final DMatrixRMaj parameterContainer;

   /**
    * The EKF API demands that the parameters we're estimating be packed into a single vector. Notably, this means we must omit the parameters we
    * assume known.
    */
   private final DMatrixRMaj consideredParameters;

   private final DMatrixRMaj measurement = new DMatrixRMaj();

   private final DMatrixRMaj generalizedContactWrenchesContainer = new DMatrixRMaj();

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

//      regressorBlockContainer = new DMatrixRMaj(totalNumberOfDoFs, RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
      parameterContainer = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1);

      consideredParameters = new DMatrixRMaj(numberOfParametersToEstimate, 1);

//      measurement = new DMatrixRMaj(totalNumberOfDoFs, 1);

//      generalizedContactWrenchesContainer = new DMatrixRMaj(totalNumberOfDoFs, 1);
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
            //               packRegressorBlockForBodyIndex(i);  // packs the relevant regressor block into regressorBlockContainer
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
    * @param parameters the state at the current time step.
    * @return
    */
   @Override
   protected DMatrixRMaj measurementModel(DMatrixRMaj parameters)
   {
      measurement.zero();

      // TODO: calling fullContactJacobians and contactWrenches from outer class might be dangerous, but getters are a PITA
//      DMatrixRMaj generalizedContactWrenches = calculateGeneralizedContactWrenches(fullContactJacobians, contactWrenches);

//      measurement.set(generalizedContactWrenches);

      int index = 0;
      for (int i = 0; i < isBodyEstimated.size(); ++i)
      {
         if (isBodyEstimated.get(i))
         {
            //               packRegressorBlockForBodyIndex(i);
            packParameterVectorForBodyIndex(index); // note the use of index here
            CommonOps_DDRM.multAdd(regressorBlockContainer, parameterContainer, measurement);
            index++;
         }
         else  // The parameters are assumed known if they are not being estimated
         {
            //               packRegressorBlockForBodyIndex(i);
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
   //      private void packRegressorBlockForBodyIndex(int index)
   //      {
   //         CommonOps_DDRM.extract(regressor,0, index * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, regressorBlockContainer);
   //      }
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
