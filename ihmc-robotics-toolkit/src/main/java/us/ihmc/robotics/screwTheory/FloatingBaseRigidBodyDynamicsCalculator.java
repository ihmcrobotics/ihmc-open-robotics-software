package us.ihmc.robotics.screwTheory;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.MatrixMissingTools;

public class FloatingBaseRigidBodyDynamicsCalculator
{
   private static final int large = 1000;
   private static final double tolerance = 0.0001;

   private final LinearSolverDense<DMatrixRMaj> pseudoInverseSolver = LinearSolverFactory_DDRM.pseudoInverse(true);

   private final DMatrixRMaj matrixTranspose = new DMatrixRMaj(large, large);
   private final DMatrixRMaj tempObjective = new DMatrixRMaj(large, large);

   public FloatingBaseRigidBodyDynamicsCalculator()
   {
   }

   /**
    * The rigid body dynamics are
    * <p>
    * [0; &tau;<sup>T</sup>]<sup>T</sup> = H(q) qDdot + C(q, qDot) + J<sub>c</sub><sup>T</sup> &rho;
    * </p>
    * <p>
    * It is worth noting that C(q, qDot) can be decomposed to C(q,qDot)qDot + G(q), where the first term captures the coriolis and centrifugal forces and
    * the second term captures the gravitational forces.
    * </p>
    * <p>
    * Based on this, it can observed that the floating base composes its own subsystem where there is no torque.  This system is underdefined. This function
    * solves for the accelerations. This can be done from the following equations.
    * </p>
    * <p>
    * 0 = H<sub>base</sub>(q) qDdot + C<sub>base</sub>(q,qDot) + J<sub>base</sub><sup>T</sup> &rho;
    * </p>
    * <p>so</p>
    * <p>
    * qDdot = -H<sub>base</sub><sup>+</sup>(C<sub>base</sub>(q,qDot) + J<sub>base</sub><sup>T</sup> &rho;)
    * </p>
    * <p>
    * where (.)<sup>+</sup> is the pseudo inverse operator.
    * </p>
    *
    * @param floatingBaseMassMatrix                 top six rows of the H(q) matrix above, which is H<sub>base</sub>(q). Not modified.
    * @param floatingBaseCoriolisMatrix             top six rows of the C(q, qDot) matrix above, which is C<sub>base</sub>(q, qDot). Not modified.
    * @param floatingBaseContactForceJacobianMatrix first six columns of the J matrix above, which is J<sub>base</sub>. Not modified.
    * @param jointAccelerationsToPack               resulting joint accelerations. &rho; in the above equations. Modified.
    * @param contactForces                          contact forces to use to compute the joint accelerations. qDdot in the above equations. Modified.
    */
   public void computeJointAccelerationGivenContactForcesForFloatingSubsystem(DMatrixRMaj floatingBaseMassMatrix,
                                                                              DMatrixRMaj floatingBaseCoriolisMatrix,
                                                                              DMatrixRMaj floatingBaseContactForceJacobianMatrix,
                                                                              DMatrixRMaj jointAccelerationsToPack,
                                                                              DMatrixRMaj contactForces)
   {
      tempObjective.set(floatingBaseCoriolisMatrix);
      CommonOps_DDRM.multAddTransA(floatingBaseContactForceJacobianMatrix, contactForces, tempObjective);

      pseudoInverseSolver.setA(floatingBaseMassMatrix);
      pseudoInverseSolver.solve(tempObjective, jointAccelerationsToPack);

      MatrixMissingTools.negate(jointAccelerationsToPack);
   }

   /**
    * The rigid body dynamics are
    * <p>
    * [0; &tau;<sup>T</sup>]<sup>T</sup> = H(q) qDdot + C(q, qDot) + J<sub>c</sub><sup>T</sup> &rho;
    * </p>
    * <p>
    * It is worth noting that C(q, qDot) can be decomposed to C(q,qDot)qDot + G(q), where the first term captures the coriolis and centrifugal forces and
    * the second term captures the gravitational forces.
    * </p>
    * <p>
    * Based on this, it can observed that the floating base composes its own subsystem where there is no torque. This system is underdefined. This function
    * solves for the contact forces. This can be done from the following equations.
    * </p>
    * <p>
    * 0 = H<sub>base</sub>(q) qDdot + C<sub>base</sub>(q,qDot) + J<sub>base</sub><sup>T</sup> &rho;
    * </p>
    * <p>so</p>
    * <p>
    * &rho; = -(J<sub>base</sub><sup>T</sup> )<sup>+</sup> (H<sub>base</sub><sup>+</sup>qDdot + C<sub>base</sub>(q,qDot))
    * </p>
    * <p>
    * where (.)<sup>+</sup> is the pseudo inverse operator.
    * </p>
    *
    * @param floatingBaseMassMatrix                 top six rows of the H<sub>base</sub>(q) matrix above. Not modified.
    * @param floatingBaseCoriolisMatrix             top six rows of the C<sub>base</sub>(q, qDot) matrix above. Not modified.
    * @param floatingBaseContactForceJacobianMatrix first six columns of the J<sub>base</sub> matrix above. Not modified.
    * @param jointAccelerations                     contact forces to compute the joint accelerations. &rho; in the above equations. Not modified.
    * @param contactForcesToPack                    resulting contact forces. qDdot in the above equations. Modified.
    */
   public void computeContactForcesGivenJointAccelerationsForFloatingSubsystem(DMatrixRMaj floatingBaseMassMatrix,
                                                                               DMatrixRMaj floatingBaseCoriolisMatrix,
                                                                               DMatrixRMaj floatingBaseContactForceJacobianMatrix,
                                                                               DMatrixRMaj jointAccelerations,
                                                                               DMatrixRMaj contactForcesToPack)
   {
      CommonOps_DDRM.transpose(floatingBaseContactForceJacobianMatrix, matrixTranspose);

      tempObjective.set(floatingBaseCoriolisMatrix);
      CommonOps_DDRM.multAdd(floatingBaseMassMatrix, jointAccelerations, tempObjective);

      pseudoInverseSolver.setA(matrixTranspose);
      pseudoInverseSolver.solve(tempObjective, contactForcesToPack);

      MatrixMissingTools.negate(contactForcesToPack);
   }

   /**
    * The rigid body dynamics are
    * <p>
    * [0; &tau;<sup>T</sup>]<sup>T</sup> = H(q) qDdot + C(q, qDot) + J<sub>c</sub><sup>T</sup> &rho;
    * </p>
    * <p>
    * The actuated portion of the dynamics are then:
    * </p>
    * <p>
    * &tau; = H<sub>body</sub>(q) qDdot + C<sub>body</sub>(q,qDot) + J<sub>body</sub><sup>T</sup> &rho;
    * </p>
    * <p>
    * This method computes the joint torques using the above equations
    * </p>
    *
    * @param bodyMassMatrix                 bottom N rows of the H(q) matrix above, which is H<sub>body</sub>(q). Not modified.
    * @param bodyCoriolisMatrix             bottom N rows of the C(q, qDot) matrix above, which is C<sub>body</sub>(q, qDot). Not modified.
    * @param bodyContactForceJacobianMatrix right N columns of the J matrix above, which is J<sub>body</sub>. Not modified.
    * @param jointAccelerations             joint accelerations. &rho; in the above equations. Not modified.
    * @param contactForces                  contact forces. qDdot in the above equations. Not modified.
    * @param jointTorquesToPack             joint torques. &tau; in the above equations. Modified.
    */
   public static void computeJointTorquesGivenContactForcesAndJointAccelerations(DMatrixRMaj bodyMassMatrix,
                                                                                 DMatrixRMaj bodyCoriolisMatrix,
                                                                                 DMatrixRMaj bodyContactForceJacobianMatrix,
                                                                                 DMatrixRMaj jointAccelerations,
                                                                                 DMatrixRMaj contactForces,
                                                                                 DMatrixRMaj jointTorquesToPack)
   {
      jointTorquesToPack.set(bodyCoriolisMatrix);
      CommonOps_DDRM.multAdd(bodyMassMatrix, jointAccelerations, jointTorquesToPack);
      CommonOps_DDRM.multAddTransA(bodyContactForceJacobianMatrix, contactForces, jointTorquesToPack);
   }

   /**
    * The rigid body dynamics are
    * <p>
    * [0; &tau;<sup>T</sup>]<sup>T</sup> = H(q) qDdot + C(q, qDot) + J<sub>c</sub><sup>T</sup> &rho;
    * </p>
    * The floating base portion of the dynamics are then:
    * </p>
    * <p>
    * 0 = H<sub>base</sub>(q) qDdot + C<sub>base</sub>(q,qDot) + J<sub>base</sub><sup>T</sup> &rho;
    * </p>
    * <p>
    * and the actuated portion of the dynamics are then:
    * </p>
    * <p>
    * &tau; = H<sub>body</sub>(q) qDdot + C<sub>body</sub>(q,qDot) + J<sub>body</sub><sup>T</sup> &rho;
    * </p>
    * <p>
    * This method performs the above calculation and checks whether the evaluation equals zero.
    * </p>
    *
    * @param floatingBaseMassMatrix                 top six rows of the H(q) matrix above, which is H<sub>base</sub>(q). Not modified.
    * @param floatingBaseCoriolisMatrix             top six rows of the C(q, qDot) matrix above, which is C<sub>base</sub>(q, qDot). Not modified.
    * @param floatingBaseContactForceJacobianMatrix first six columns of the J matrix above, which is J<sub>base</sub>. Not modified.
    * @param bodyMassMatrix                         bottom N rows of the H(q) matrix above, which is H<sub>body</sub>(q). Not modified.
    * @param bodyCoriolisMatrix                     bottom N rows of the C(q, qDot) matrix above, which is C<sub>body</sub>(q, qDot). Not modified.
    * @param bodyContactForceJacobianMatrix         right N columns of the J matrix above, which is J<sub>body</sub>. Not modified.
    * @param jointAccelerations                     joint accelerations. &rho; in the above equations. Not modified.
    * @param jointTorques                           joint torques. &tau; in the above equations. Not modified.
    * @param contactForces                          contact forces. qDdot in the above equations. Not modified.
    * @return where the defined dynamics hold.
    */
   public boolean areFloatingBaseRigidBodyDynamicsSatisfied(DMatrixRMaj floatingBaseMassMatrix,
                                                            DMatrixRMaj floatingBaseCoriolisMatrix,
                                                            DMatrixRMaj floatingBaseContactForceJacobianMatrix,
                                                            DMatrixRMaj bodyMassMatrix,
                                                            DMatrixRMaj bodyCoriolisMatrix,
                                                            DMatrixRMaj bodyContactForceJacobianMatrix,
                                                            DMatrixRMaj jointAccelerations,
                                                            DMatrixRMaj jointTorques,
                                                            DMatrixRMaj contactForces)
   {
      if (!areFloatingBaseDynamicsSatisfied(floatingBaseMassMatrix,
                                            floatingBaseCoriolisMatrix,
                                            floatingBaseContactForceJacobianMatrix,
                                            jointAccelerations,
                                            contactForces))
         return false;

      return areActuatedDynamicsSatisfied(bodyMassMatrix, bodyCoriolisMatrix, bodyContactForceJacobianMatrix, jointAccelerations, jointTorques, contactForces);
   }

   /**
    * The rigid body dynamics are
    * <p>
    * [0; &tau;<sup>T</sup>]<sup>T</sup> = H(q) qDdot + C(q, qDot) + J<sub>c</sub><sup>T</sup> &rho;
    * </p>
    * <p>
    * The floating base portion of the dynamics are then:
    * </p>
    * <p>
    * 0 = H<sub>base</sub>(q) qDdot + C<sub>base</sub>(q,qDot) + J<sub>base</sub><sup>T</sup> &rho;
    * </p>
    * <p>
    * This method performs the above calculation and checks whether the evaluation equals zero.
    * </p>
    *
    * @param floatingBaseMassMatrix                 top six rows of the H(q) matrix above, which is H<sub>base</sub>(q). Not modified.
    * @param floatingBaseCoriolisMatrix             top six rows of the C(q, qDot) matrix above, which is C<sub>base</sub>(q, qDot). Not modified.
    * @param floatingBaseContactForceJacobianMatrix first six columns of the J matrix above, which is J<sub>base</sub>. Not modified.
    * @param jointAccelerations                     joint accelerations. &rho; in the above equations. Not modified.
    * @param contactForces                          contact forces. qDdot in the above equations. Not modified.
    * @return whether the defined dynamics hold.
    **/
   public boolean areFloatingBaseDynamicsSatisfied(DMatrixRMaj floatingBaseMassMatrix,
                                                   DMatrixRMaj floatingBaseCoriolisMatrix,
                                                   DMatrixRMaj floatingBaseContactForceJacobianMatrix,
                                                   DMatrixRMaj jointAccelerations,
                                                   DMatrixRMaj contactForces)
   {
      tempObjective.set(floatingBaseCoriolisMatrix);
      CommonOps_DDRM.multAdd(floatingBaseMassMatrix, jointAccelerations, tempObjective);
      CommonOps_DDRM.multAddTransA(floatingBaseContactForceJacobianMatrix, contactForces, tempObjective);

      return equalsZero(tempObjective, tolerance);
   }

   /**
    * The rigid body dynamics are
    * <p>
    * [0; &tau;<sup>T</sup>]<sup>T</sup> = H(q) qDdot + C(q, qDot) + J<sub>c</sub><sup>T</sup> &rho;
    * </p>
    * <p>
    * The actuated portion of the dynamics are then:
    * </p>
    * <p>
    * &tau; = H<sub>body</sub>(q) qDdot + C<sub>body</sub>(q,qDot) + J<sub>body</sub><sup>T</sup> &rho;
    * </p>
    * <p>
    * This method performs the above calculation and checks whether the evaluation equals zero.
    * </p>
    *
    * @param bodyMassMatrix                 bottom N rows of the H(q) matrix above, which is H<sub>body</sub>(q). Not modified.
    * @param bodyCoriolisMatrix             bottom N rows of the C(q, qDot) matrix above, which is C<sub>body</sub>(q, qDot). Not modified.
    * @param bodyContactForceJacobianMatrix right N columns of the J matrix above, which is J<sub>body</sub>. Not modified.
    * @param jointAccelerations             joint accelerations. &rho; in the above equations. Not modified.
    * @param jointTorques                   joint torques. &tau; in the above equations. Not modified.
    * @param contactForces                  contact forces. qDdot in the above equations. Not modified.
    * @return where the defined dynamics hold.
    */
   public boolean areActuatedDynamicsSatisfied(DMatrixRMaj bodyMassMatrix,
                                               DMatrixRMaj bodyCoriolisMatrix,
                                               DMatrixRMaj bodyContactForceJacobianMatrix,
                                               DMatrixRMaj jointAccelerations,
                                               DMatrixRMaj jointTorques,
                                               DMatrixRMaj contactForces)
   {
      tempObjective.set(bodyCoriolisMatrix);
      CommonOps_DDRM.multAdd(bodyMassMatrix, jointAccelerations, tempObjective);
      CommonOps_DDRM.multAddTransA(bodyContactForceJacobianMatrix, contactForces, tempObjective);
      CommonOps_DDRM.subtractEquals(tempObjective, jointTorques);

      return equalsZero(tempObjective, tolerance);
   }

   private static boolean equalsZero(DMatrixRMaj matrix, double tolerance)
   {
      for (int element = 0; element < matrix.getNumElements(); element++)
      {
         if (!MathTools.epsilonEquals(matrix.data[element], 0.0, tolerance))
            return false;
      }

      return true;
   }
}
