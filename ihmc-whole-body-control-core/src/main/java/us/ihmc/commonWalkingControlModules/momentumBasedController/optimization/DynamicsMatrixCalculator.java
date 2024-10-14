package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.screwTheory.FloatingBaseRigidBodyDynamicsCalculator;
import us.ihmc.robotics.screwTheory.GravityCoriolisExternalWrenchMatrixCalculator;

/**
 * Helper class for computing the rigid body dynamics matrices. In general, the rigid body dynamics are represented by
 * <p>
 *    H(q) qDdot + C(q, qDot) + J<sup>T</sup>(q) &rho; = [0; &tau;<sup>T</sup>]<sup>T</sup>
 * </p>
 * <p>
 *    They can be further partition into the floating base component, demarcated by (.)<sub>base</sub>, and the actuated component, demarcated by
 *    (.)<sub>body</sub>. These correspond to the top rows in the above equation for the base and the bottom rows for the actuated portion.
 * </p>>
 * <p>
 *    In inverse dynamics classes, the joint accelerations (qDdot) and contact forces (&rho;) are typically solved for, and then the joint torques (&tau;) are
 *    computed. This class is designed to provide the affine terms to compute these values.
 * </p>
 */
public class DynamicsMatrixCalculator
{
   private final CompositeRigidBodyMassMatrixCalculator massMatrixCalculator;
   private final GravityCoriolisExternalWrenchMatrixCalculator coriolisMatrixCalculator;
   private final ContactWrenchMatrixCalculator contactWrenchMatrixCalculator;

   /** H(q) **/
   private final DMatrixRMaj massMatrix;
   /** C(q, qDdot) **/
   private final DMatrixRMaj gravityAndCoriolisVector;
   /** J(q) **/
   private final DMatrixRMaj contactForceJacobian;

   /** H<sub>base</sub>(q) **/
   private final DMatrixRMaj floatingBaseMassMatrix;
   /** C<sub>base</sub>(q, qDdot) **/
   private final DMatrixRMaj floatingBaseGravityAndCoriolisVector;
   /** J<sub>base</sub>(q) **/
   private final DMatrixRMaj floatingBaseContactForceJacobian;

   /** H<sub>body</sub>(q) **/
   private final DMatrixRMaj bodyMassMatrix;
   /** C<sub>body</sub>(q, qDdot) **/
   private final DMatrixRMaj bodyGravityAndCoriolisVector;
   /** J<sub>body</sub>(q) **/
   private final DMatrixRMaj bodyContactForceJacobian;
   /** J<sup>T</sup><sub>body</sub>(q) **/
   private final DMatrixRMaj bodyContactForceJacobianTranspose;

   private final DMatrixRMaj jointTorques;

   /**
    * When minimizing the joint torque, that be written as
    * <p>
    *    &tau; = H<sub>base</sub>(q) qDdot + C<sub>base</sub>(q, qDot) + J<sup>T</sup><sub>base</sub>(q) &rho;
    * </p>
    * or
    * <p>
    *    &tau; = [H<sub>base</sub>(q) J<sup>T</sup><sub>base</sub>(q)] [qDdot; &rho;] - C<sub>base</sub>(q, qDot) = J u - b
    * </p>
    * This is b in the above equations, which is equivalent to -C<sub>base</sub>(q, qDot)
    */
   private final DMatrixRMaj torqueMinimizationObjective;

   private final FloatingBaseRigidBodyDynamicsCalculator rbdCalculator = new FloatingBaseRigidBodyDynamicsCalculator();

   private boolean areRBDMatricesUpToDate = false;
   private boolean bodyContactForceJacobianTransposeIsUpToDate = false;
   private boolean torqueMinimizationObjectiveIsUpToDate = false;
   private boolean areFloatingBaseMatricesSegmented = false;
   private boolean areActuatedMatricesSegmented = false;

   private final int rhoSize;
   private final int degreesOfFreedom;
   private final int floatingBaseDoFs;
   private final int bodyDoFs;

   public DynamicsMatrixCalculator(WholeBodyControlCoreToolbox toolbox)
   {
      this(toolbox.getRootJoint(),
           toolbox.getJointIndexHandler(),
           toolbox.getMassMatrixCalculator(),
           toolbox.getGravityCoriolisExternalWrenchMatrixCalculator(),
           toolbox.getContactWrenchMatrixCalculator(),
           toolbox.getRhoSize());
   }

   public DynamicsMatrixCalculator(FloatingJointBasics rootJoint,
                                   JointIndexHandler jointIndexHandler,
                                   CompositeRigidBodyMassMatrixCalculator massMatrixCalculator,
                                   GravityCoriolisExternalWrenchMatrixCalculator coriolisMatrixCalculator,
                                   ContactWrenchMatrixCalculator contactWrenchMatrixCalculator,
                                   int rhoSize)
   {
      this.massMatrixCalculator = massMatrixCalculator;
      this.coriolisMatrixCalculator = coriolisMatrixCalculator;
      this.contactWrenchMatrixCalculator = contactWrenchMatrixCalculator;
      this.rhoSize = rhoSize;

      degreesOfFreedom = jointIndexHandler.getNumberOfDoFs();
      floatingBaseDoFs = rootJoint != null ? rootJoint.getDegreesOfFreedom() : 0;
      bodyDoFs = degreesOfFreedom - floatingBaseDoFs;

      jointTorques = new DMatrixRMaj(bodyDoFs, 1);

      massMatrix = new DMatrixRMaj(degreesOfFreedom, degreesOfFreedom);
      gravityAndCoriolisVector = new DMatrixRMaj(degreesOfFreedom, 1);
      contactForceJacobian = new DMatrixRMaj(rhoSize, degreesOfFreedom);

      floatingBaseMassMatrix = new DMatrixRMaj(floatingBaseDoFs, degreesOfFreedom);
      floatingBaseGravityAndCoriolisVector = new DMatrixRMaj(floatingBaseDoFs, 1);
      floatingBaseContactForceJacobian = new DMatrixRMaj(rhoSize, floatingBaseDoFs);

      bodyMassMatrix = new DMatrixRMaj(bodyDoFs, degreesOfFreedom);
      bodyGravityAndCoriolisVector = new DMatrixRMaj(bodyDoFs, 1);
      bodyContactForceJacobian = new DMatrixRMaj(rhoSize, bodyDoFs);
      bodyContactForceJacobianTranspose = new DMatrixRMaj(bodyDoFs, rhoSize);

      torqueMinimizationObjective = new DMatrixRMaj(bodyDoFs, 1);
   }

   public void setGravity(double gravityZ)
   {
      coriolisMatrixCalculator.setGravitionalAcceleration(-Math.abs(gravityZ));
   }

   public void reset()
   {
      coriolisMatrixCalculator.setExternalWrenchesToZero();

      areRBDMatricesUpToDate = false;
      bodyContactForceJacobianTransposeIsUpToDate = false;
      torqueMinimizationObjectiveIsUpToDate = false;
      areFloatingBaseMatricesSegmented = false;
      areActuatedMatricesSegmented = false;
   }

   /**
    * <p>
    * Sets an external force to be achieved. This is not a contactable body to use for balancing, as those forces come from the contact force vector later.
    * </p>
    *
    * @param rigidBody      body to which the wrench is applied.
    * @param externalWrench external wrench acting on body.
    */
   public void setExternalWrench(RigidBodyBasics rigidBody, WrenchReadOnly externalWrench)
   {
      coriolisMatrixCalculator.setExternalWrench(rigidBody, externalWrench);
   }

   /**
    * This updates the rigid body matrices, and also sets the flags to perform efficient updates of the other matrices to false, indicating they must also be
    * updated. This call performs no computation if {@link #reset()} has not been called.
    */
   public void compute()
   {
      if (areRBDMatricesUpToDate)
         return;

      bodyContactForceJacobianTransposeIsUpToDate = false;
      torqueMinimizationObjectiveIsUpToDate = false;
      areFloatingBaseMatricesSegmented = false;
      areActuatedMatricesSegmented = false;

      massMatrixCalculator.reset();
      coriolisMatrixCalculator.compute();

      // Gets the mass matrix
      massMatrix.set(massMatrixCalculator.getMassMatrix());

      // Gets the coriolis and gravity matrix
      gravityAndCoriolisVector.set(coriolisMatrixCalculator.getJointTauMatrix());

      // Gets the contact force jacobian
      contactWrenchMatrixCalculator.computeContactForceJacobian(contactForceJacobian);

      areRBDMatricesUpToDate = true;
   }

   /**
    * Partitions the inertial matrices to extract the floating base components
    */
   private void segmentFloatingBaseMatrices()
   {
      assertRigidBodyDynamicsMatricesAreUpToDate();

      CommonOps_DDRM.extract(massMatrix, 0, floatingBaseDoFs, 0, degreesOfFreedom, floatingBaseMassMatrix, 0, 0);
      CommonOps_DDRM.extract(gravityAndCoriolisVector, 0, floatingBaseDoFs, 0, 1, floatingBaseGravityAndCoriolisVector, 0, 0);
      CommonOps_DDRM.extract(contactForceJacobian, 0, rhoSize, 0, floatingBaseDoFs, floatingBaseContactForceJacobian, 0, 0);
      areFloatingBaseMatricesSegmented = true;
   }

   /**
    * Partitions the inertial matrices to extract the floating base components
    */
   private void segmentActuatedMatrices()
   {
      assertRigidBodyDynamicsMatricesAreUpToDate();

      CommonOps_DDRM.extract(massMatrix, floatingBaseDoFs, degreesOfFreedom, 0, degreesOfFreedom, bodyMassMatrix, 0, 0);
      CommonOps_DDRM.extract(gravityAndCoriolisVector, floatingBaseDoFs, degreesOfFreedom, 0, 1, bodyGravityAndCoriolisVector, 0, 0);
      CommonOps_DDRM.extract(contactForceJacobian, 0, rhoSize, floatingBaseDoFs, degreesOfFreedom, bodyContactForceJacobian, 0, 0);
      areActuatedMatricesSegmented = true;
   }

   private void computeTorqueMinimizationObjective()
   {
      assertRigidBodyDynamicsMatricesAreUpToDate();

      DMatrixRMaj bodyGravityAndCoriolisVector = getBodyGravityAndCoriolisVector();

      torqueMinimizationObjective.reshape(bodyGravityAndCoriolisVector.getNumRows(), bodyGravityAndCoriolisVector.getNumCols());
      CommonOps_DDRM.changeSign(bodyGravityAndCoriolisVector, torqueMinimizationObjective);
      torqueMinimizationObjectiveIsUpToDate = true;
   }

   private void computeBodyContactForceJacobianTranspose()
   {
      assertRigidBodyDynamicsMatricesAreUpToDate();

      DMatrixRMaj bodyContactForceJacobian = getBodyContactForceJacobian();
      CommonOps_DDRM.transpose(bodyContactForceJacobian, bodyContactForceJacobianTranspose);
      bodyContactForceJacobianTransposeIsUpToDate = true;
   }

   /**
    * <p>
    * Computes the joint torques that satisfy the rigid body dynamics for the desired joint
    * accelerations and contact forces
    * </p>
    *
    * @param jointAccelerations joint accelerations used to compute the joint torques. Not modified.
    * @param contactForces      contact forces used to compute the joint torques. Not modified.
    * @return jointTorques resulting joint torques from the rigid body dynamics.
    */
   public DMatrixRMaj computeJointTorques(DMatrixRMaj jointAccelerations, DMatrixRMaj contactForces)
   {
      computeJointTorques(jointTorques, jointAccelerations, contactForces);

      return jointTorques;
   }

   /**
    * <p>
    * Computes the joint torques that satisfy the rigid body dynamics for the desired joint
    * accelerations and contact forces
    * </p>
    *
    * @param jointTorquesToPack resulting joint torques from the rigid body dynamics. Modified.
    * @param jointAccelerations joint accelerations used to compute the joint torques. Not modified.
    * @param contactForces      contact forces used to compute the joint torques. Not modified.
    */
   public void computeJointTorques(DMatrixRMaj jointTorquesToPack, DMatrixRMaj jointAccelerations, DMatrixRMaj contactForces)
   {
      FloatingBaseRigidBodyDynamicsCalculator.computeJointTorquesGivenContactForcesAndJointAccelerations(getBodyMassMatrix(),
                                                                                                         getBodyGravityAndCoriolisVector(),
                                                                                                         getBodyContactForceJacobian(),
                                                                                                         jointAccelerations,
                                                                                                         contactForces,
                                                                                                         jointTorquesToPack);
   }

   public void getMassMatrix(DMatrixRMaj massMatrixToPack)
   {
      assertRigidBodyDynamicsMatricesAreUpToDate();
      massMatrixToPack.set(massMatrix);
   }

   public void getGravityAndCoriolisVector(DMatrixRMaj gravityAndCoriolisVectorToPack)
   {
      assertRigidBodyDynamicsMatricesAreUpToDate();
      gravityAndCoriolisVectorToPack.set(gravityAndCoriolisVector);
   }

   public void getBodyMassMatrix(DMatrixRMaj bodyMassMatrixToPack)
   {
      bodyMassMatrixToPack.set(getBodyMassMatrix());
   }

   public void getBodyGravityAndCoriolisVector(DMatrixRMaj bodyGravityAndCoriolisVectorToPack)
   {
      bodyGravityAndCoriolisVectorToPack.set(getBodyGravityAndCoriolisVector());
   }

   public DMatrixRMaj getFloatingBaseMassMatrix()
   {
      if (!areFloatingBaseMatricesSegmented)
         segmentFloatingBaseMatrices();
      return floatingBaseMassMatrix;
   }

   public DMatrixRMaj getFloatingBaseGravityAndCoriolisVector()
   {
      if (!areFloatingBaseMatricesSegmented)
         segmentFloatingBaseMatrices();
      return floatingBaseGravityAndCoriolisVector;
   }

   public DMatrixRMaj getFloatingBaseContactForceJacobian()
   {
      if (!areFloatingBaseMatricesSegmented)
         segmentFloatingBaseMatrices();
      return floatingBaseContactForceJacobian;
   }

   public DMatrixRMaj getBodyMassMatrix()
   {
      if (!areActuatedMatricesSegmented)
         segmentActuatedMatrices();
      return bodyMassMatrix;
   }

   public DMatrixRMaj getBodyGravityAndCoriolisVector()
   {
      if (!areActuatedMatricesSegmented)
         segmentActuatedMatrices();
      return bodyGravityAndCoriolisVector;
   }

   public DMatrixRMaj getBodyContactForceJacobian()
   {
      if (!areActuatedMatricesSegmented)
         segmentActuatedMatrices();
      return bodyContactForceJacobian;
   }

   public DMatrixRMaj getBodyContactForceJacobianTranspose()
   {
      if (!bodyContactForceJacobianTransposeIsUpToDate)
         computeBodyContactForceJacobianTranspose();
      return bodyContactForceJacobianTranspose;
   }

   public DMatrixRMaj getTorqueMinimizationObjective()
   {
      if (!torqueMinimizationObjectiveIsUpToDate)
         computeTorqueMinimizationObjective();
      return torqueMinimizationObjective;
   }

   private void assertRigidBodyDynamicsMatricesAreUpToDate()
   {
      if (!areRBDMatricesUpToDate)
         throw new RuntimeException("Rigid body dynamics matrices must be updated by calling DynamicsMatrixCalculator.compute() before accessing this value.");
   }

   /**
    * <p>
    * Computes the required contact forces given the joint accelerations using the rigid-body dynamics
    * for the floating body.  This is an interative method which seeks a consensus between the contact forces and the joint accelerations that satisfy the
    * floating base dynamics of the system. It should not be used for real time control, and is only used for testing.
    * </p>
    *
    * @param jointAccelerationsToPack resulting joint accelerations. Modified.
    * @param contactForcesToPack resulting contact forces. Should be initialized from somewhere. Modified.
    */
   void computeRequiredContactForcesAndJointAccelerationsFromInitialContactForcesGuess(DMatrixRMaj jointAccelerationsToPack, DMatrixRMaj contactForcesToPack)
   {
      computeRequiredRhoAndAchievableQddotGivenRhoImpl(jointAccelerationsToPack, contactForcesToPack, 0);
   }

   void computeRequiredRhoAndAchievableQddotGivenRhoImpl(DMatrixRMaj jointAccelerationsToPack, DMatrixRMaj contactForcesToPack, int iter)
   {
      // Compute the joint accelerations given the current contact force guess.
      rbdCalculator.computeJointAccelerationGivenContactForcesForFloatingSubsystem(getFloatingBaseMassMatrix(),
                                                                                   getFloatingBaseGravityAndCoriolisVector(),
                                                                                   getFloatingBaseContactForceJacobian(),
                                                                                   jointAccelerationsToPack,
                                                                                   contactForcesToPack);

      // Verify that this is a valid solution.
      if (!checkFloatingBaseDynamicsSatisfied(jointAccelerationsToPack, contactForcesToPack))
      {
         if (iter > 1000)
            throw new RuntimeException("Overflow in computation - cannot find a satisfactory qddot.");

         // Solution isn't valid. Refine the contact force guess from the joint accelerations.
         rbdCalculator.computeContactForcesGivenJointAccelerationsForFloatingSubsystem(getFloatingBaseMassMatrix(),
                                                                                       getFloatingBaseGravityAndCoriolisVector(),
                                                                                       getFloatingBaseContactForceJacobian(),
                                                                                       jointAccelerationsToPack,
                                                                                       contactForcesToPack);

         // Perform another iteration, computing the joint accelerations and then checking if the contact forces need further refinement.
         computeRequiredRhoAndAchievableQddotGivenRhoImpl(jointAccelerationsToPack, contactForcesToPack, ++iter);
      }
   }

   /**
    * <p>
    * Checks whether the floating base portion of the rigid body dynamics is satisfied by the given joint accelerations and contact forces.
    * </p>
    * <p>
    * Must satisfy the equation H<sub>base</sub> qDdot + C<sub>base</sub> + J<sub>base</sub><sup>T</sup> &rho; = 0
    * </p>
    *
    * @param jointAccelerations current joint accelerations. qDdot in the above equations. Not modified.
    * @param contactForces current contact forces. &rho; in the above equations. Not modified.
    * @return whether dynamics are satisfied
    */
   private boolean checkFloatingBaseDynamicsSatisfied(DMatrixRMaj jointAccelerations, DMatrixRMaj contactForces)
   {
      return rbdCalculator.areFloatingBaseDynamicsSatisfied(getFloatingBaseMassMatrix(),
                                                            getFloatingBaseGravityAndCoriolisVector(),
                                                            getFloatingBaseContactForceJacobian(),
                                                            jointAccelerations,
                                                            contactForces);
   }

   /**
    * <p>
    * Checks whether the rigid body dynamics is satisfied by the given joint accelerations, joint torques, and contact forces.
    * </p>
    * <p>
    * Must satisfy the equation H qDdot + C + J<sup>T</sup> &rho; = [0; &tau;<sup>T</sup>]<sup>T</sup>
    * </p>
    *
    * @param jointAccelerations current joint accelerations. qDdot in the above equations. Not modified.
    * @param jointTorques current joint torques. &tau; in the above equations. Not modified.
    * @param contactForces current contact forces. &rho; in the above equations. Not modified.
    * @return whether dynamics are satisfied
    */
   boolean checkRigidBodyDynamicsSatisfied(DMatrixRMaj jointAccelerations, DMatrixRMaj jointTorques, DMatrixRMaj contactForces)
   {
      return rbdCalculator.areFloatingBaseRigidBodyDynamicsSatisfied(getFloatingBaseMassMatrix(),
                                                                     getFloatingBaseGravityAndCoriolisVector(),
                                                                     getFloatingBaseContactForceJacobian(),
                                                                     getBodyMassMatrix(),
                                                                     getBodyGravityAndCoriolisVector(),
                                                                     getBodyContactForceJacobian(),
                                                                     jointAccelerations,
                                                                     jointTorques,
                                                                     contactForces);
   }
}
