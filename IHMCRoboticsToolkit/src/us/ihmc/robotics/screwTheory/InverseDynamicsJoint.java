package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.kinematics.CommonJoint;
import us.ihmc.robotics.nameBasedHashCode.NameBasedHashCodeHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * Base interface that describes the basic API for the joints composing a kinematic chain/tree
 * compatible with the screw theory. Here are a few examples of useful screw tools:
 * <ul>
 * <li>The {@link GeometricJacobian} can compute the Jacobian matrix of a kinematic chain.
 * <li>The {@link TwistCalculator} can compute all the {@code Twist} (angular and linear velocity)
 * of all the {@code RigidBody}s of a kinematic chain/tree.
 * <li>Similar to the {@code TwistCalculator}, the {@link SpatialAccelerationCalculator} can compute
 * all the spatial accelerations of all the {@code RigidBody}s of a kinematic chain/tree.
 * <li>Based on the recursive Newton-Euler algorithm, the {@link InverseDynamicsCalculator} computes
 * the desired joint torques {@code tau} from the joint configurations {@code q}, velocities
 * {@code qd}, desired accelerations {@code qddDesired}, and the list of external {@code Wrench}es
 * applied on the system.
 * <li>Other tools such as {@link CentroidalMomentumMatrix}, {@link ConvectiveTermCalculator},
 * {@link CentroidalMomentumRateTermCalculator}, are useful tools for developing whole-body control
 * framework.
 * </ul>
 * 
 */
public abstract interface InverseDynamicsJoint extends CommonJoint, NameBasedHashCodeHolder
{
   public static int maxDoF = 6;

   /**
    * Returns the the {@code ReferenceFrame} that is attached to the predecessor of this joint,
    * namely the {@code RigidBody} before this joint, and has its origin centered at the joint
    * origin. The pose of the {@code frameBeforeJoint} is independent from this joint motion.
    * 
    * @return the {@code ReferenceFrame} located right before this joint.
    */
   public abstract ReferenceFrame getFrameBeforeJoint();

   /**
    * Returns the the {@code ReferenceFrame} that is attached to the successor of this joint, namely
    * the {@code RigidBody} after this joint, and has its origin centered at the joint origin. The
    * pose of the {@code frameAfterJoint} will change as this joint moves.
    * 
    * @return the {@code ReferenceFrame} located right after this joint.
    */
   public abstract ReferenceFrame getFrameAfterJoint();

   /**
    * Packs the actual, not desired, velocity of this joint in a {@code Twist} (the 3D angular and
    * linear velocities). The reference frames of the resulting {@code Twist} are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code afterJointFrame}.
    * <li>{@code baseFrame} is {@code beforeJointFrame}.
    * <li>{@code expressedInFrame} is {@code afterJointFrame}.
    * </ul>
    * 
    * @param twistToPack the {@code Twist} in which the velocity of this joint is stored. Modified.
    */
   public abstract void getJointTwist(Twist twistToPack);

   /**
    * Packs the {@code Twist} (the 3D angular and linear velocities) of this joint's
    * {@code successor} with respect to this joint's {@code predecessor}. The reference frames of
    * the resulting {@code Twist} are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code successorFrame == successor.getBodyFixedFrame()}.
    * <li>{@code baseFrame} is {@code predecessorFrame == predecessor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} is {@code successorFrame}.
    * </ul>
    * 
    * @param twistToPack the {@code Twist} in which the velocity of this joint's {@code successor}
    *           is stored. Modified.
    */
   public abstract void getSuccessorTwist(Twist twistToPack);

   /**
    * Packs the {@code Twist} (the 3D angular and linear velocities) of this joint's
    * {@code predecessor} with respect to this joint's {@code successor}. The reference frames of
    * the resulting {@code Twist} are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code predecessorFrame == predecessor.getBodyFixedFrame()}.
    * <li>{@code baseFrame} is {@code successorFrame == successor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} is {@code predecessorFrame}.
    * </ul>
    * 
    * @param twistToPack the {@code Twist} in which the velocity of this joint's {@code predecessor}
    *           is stored. Modified.
    */
   public abstract void getPredecessorTwist(Twist twistToPack);

   /**
    * Packs the actual, not desired, acceleration of this joint in a
    * {@code SpatialAccelerationVector} (the 3D angular and linear accelerations). The reference
    * frames of the resulting {@code SpatialAccelerationVector} are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code afterJointFrame}.
    * <li>{@code baseFrame} is {@code beforeJointFrame}.
    * <li>{@code expressedInFrame} is {@code afterJointFrame}.
    * </ul>
    * 
    * @param accelerationToPack the {@code SpatialAccelerationVector} in which the acceleration of
    *           this joint is stored. Modified.
    */
   public abstract void getJointAcceleration(SpatialAccelerationVector accelerationToPack);

   /**
    * Packs the {@code SpatialAccelerationVector} (the 3D angular and linear accelerations) of this
    * joint's {@code successor} with respect to this joint's {@code predecessor}. The reference
    * frames of the resulting {@code SpatialAccelerationVector} are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code successorFrame == successor.getBodyFixedFrame()}.
    * <li>{@code baseFrame} is {@code predecessorFrame == predecessor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} is {@code successorFrame}.
    * </ul>
    * 
    * @param twistToPack the {@code SpatialAccelerationVector} in which the acceleration of this
    *           joint's {@code successor} is stored. Modified.
    */
   public abstract void getSuccessorAcceleration(SpatialAccelerationVector jointAccelerationToPack);

   /**
    * Packs the desired acceleration of this joint in a {@code SpatialAccelerationVector} (the 3D
    * angular and linear accelerations). The reference frames of the resulting
    * {@code SpatialAccelerationVector} are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code afterJointFrame}.
    * <li>{@code baseFrame} is {@code beforeJointFrame}.
    * <li>{@code expressedInFrame} is {@code afterJointFrame}.
    * </ul>
    * 
    * @param accelerationToPack the {@code SpatialAccelerationVector} in which the desired
    *           acceleration of this joint is stored. Modified.
    */
   public abstract void getDesiredJointAcceleration(SpatialAccelerationVector jointAccelerationToPack);

   /**
    * Packs the {@code SpatialAccelerationVector} (the 3D angular and linear accelerations) of this
    * joint's {@code successor} with respect to this joint's {@code predecessor} resulting from the
    * desired acceleration of this joint. The reference frames of the resulting
    * {@code SpatialAccelerationVector} are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code successorFrame == successor.getBodyFixedFrame()}.
    * <li>{@code baseFrame} is {@code predecessorFrame == predecessor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} is {@code successorFrame}.
    * </ul>
    * 
    * @param twistToPack the {@code SpatialAccelerationVector} in which the acceleration of this
    *           joint's {@code successor} resulting from this joint desired acceleration is stored.
    *           Modified.
    */
   public abstract void getDesiredSuccessorAcceleration(SpatialAccelerationVector jointAccelerationToPack);

   /**
    * Packs the {@code SpatialAccelerationVector} (the 3D angular and linear accelerations) of this
    * joint's {@code predecessor} with respect to this joint's {@code successor} resulting from the
    * desired acceleration of this joint. The reference frames of the resulting
    * {@code SpatialAccelerationVector} are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code predecessorFrame == predecessor.getBodyFixedFrame()}.
    * <li>{@code baseFrame} is {@code successorFrame == successor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} is {@code predecessorFrame}.
    * </ul>
    * 
    * @param twistToPack the {@code SpatialAccelerationVector} in which the acceleration of this
    *           joint's {@code predecessor} resulting from this joint desired acceleration is
    *           stored. Modified.
    */
   public abstract void getDesiredPredecessorAcceleration(SpatialAccelerationVector jointAccelerationToPack);

   /**
    * Packs this joint's configuration into a column vector {@code DenseMatrix64F}. Here are a few
    * examples:
    * <ul>
    * <li>For a {@code RevoluteJoint}, the actual joint angle is stored at the
    * {@code rowStart}<sup>th</sup> row.
    * <li>For a {@code SphericalJoint}, the actual joint configuration is a quaternion and is stored
    * from the {@code rowStart}<sup>th</sup> row to the ({@code rowStart + 4})<sup>th</sup> row.
    * </ul>
    * 
    * @param matrixToPack the column vector in which this joint actual configuration is stored.
    *           Modified.
    * @param rowStart row index for the first component of the configuration.
    */
   public abstract void getConfigurationMatrix(DenseMatrix64F matrixToPack, int rowStart);

   /**
    * Packs this joint desired force/torque into a column vector {@code DenseMatrix64F}. Here are a
    * few examples:
    * <ul>
    * <li>For a {@code RevoluteJoint}, the desired joint torque is stored at the 1<sup>st</sup> row.
    * <li>For a {@code PrismaticJoint}, the desired joint force is stored at the 1<sup>st</sup> row.
    * <li>For a {@code SixDoFJoint}, the desired wrench (the 3D torque and 3D force) of this joint
    * is stored from the {@code rowStart}<sup>th</sup> row to the
    * ({@code rowStart + 6})<sup>th</sup> row, starting with the three components of the torque.
    * Note: the joint wrench is the wrench of {@code successorFrame} expressed in
    * {@code successorFrame}.
    * </ul>
    * 
    * @param matrixToPack the column vector in which the desired force/torque of this joint is
    *           stored. Modified.
    */
   public abstract void getTauMatrix(DenseMatrix64F matrixToPack);

   /**
    * Packs this joint actual velocity into a column vector {@code DenseMatrix64F}. Here are a few
    * examples:
    * <ul>
    * <li>For a {@code OneDoFJoint}, the scalar velocity {@code qd} is stored at the
    * {@code rowStart}<sup>th</sup> row.
    * <li>For a {@code SixDoFJoint}, the joint twist is stored from the
    * {@code rowStart}<sup>th</sup> row to the ({@code rowStart + 6})<sup>th</sup> row, starting
    * with the three components of the angular velocity. Note: the joint twist is the twist of the
    * {@code afterJointFrame} with respect to the {@code beforeJointFrame} expressed in the
    * {@code afterJointFrame}.
    * </ul>
    * 
    * @param matrixToPack the column vector in which the velocity of this joint is stored. Modified.
    * @param rowStart row index for the first component of the velocity.
    */
   public abstract void getVelocityMatrix(DenseMatrix64F matrixToPack, int rowStart);

   /**
    * Packs this joint desired acceleration into a column vector {@code DenseMatrix64F}. Here are a
    * few examples:
    * <ul>
    * <li>For a {@code OneDoFJoint}, the scalar acceleration {@code qddDesired} is stored at the
    * {@code rowStart}<sup>th</sup> row.
    * <li>For a {@code SixDoFJoint}, the joint desired spatial acceleration is stored from the
    * {@code rowStart}<sup>th</sup> row to the ({@code rowStart + 6})<sup>th</sup> row, starting
    * with the three components of the angular acceleration.
    * </ul>
    * 
    * @param matrixToPack the column vector in which the acceleration of this joint is stored.
    *           Modified.
    * @param rowStart row index for the first component of the acceleration.
    */
   public abstract void getDesiredAccelerationMatrix(DenseMatrix64F matrixToPack, int rowStart);

   /**
    * Sets the desired acceleration stored in this joint to zero.
    */
   public abstract void setDesiredAccelerationToZero();

   /**
    * Sets the joint current configuration from the given column vector {@code DenseMatrix64F}. Here
    * are a few examples:
    * <ul>
    * <li>For a {@code RevoluteJoint}, the {@code rowStart}<sup>th</sup> row of the given column
    * vector is used to set the current joint angle {@code q}.
    * <li>For a {@code SixDoFJoint}, the 4 rows starting from {@code rowStart} are used to set the
    * current 3D orientation as a quaternion, and the 3 rows starting from ({@code rowStart + 4})
    * are used to set the 3D position.
    * </ul>
    * 
    * @param matrix the column vector from which the configuration of this joint is to be extracted.
    *           Not modified.
    * @param rowStart row index of the first component of this joint configuration.
    */
   public abstract void setConfiguration(DenseMatrix64F matrix, int rowStart);

   /**
    * Sets this joint current velocity from the given column vector {@code DenseMAtrix64F}. Here are
    * a few examples:
    * <ul>
    * <li>For a {@code RevoluteJoint}, the {@code rowStart}<sup>th</sup> row of the given column
    * vector is used to set the joint current angular velocity {@code qd}.
    * <li>For a {@code SixDoFJoint}, the 6 rows starting from {@code rowStart} are use to set the
    * current twist of this joint starting with the angular velocity. Note: the joint twist is the
    * twist of the {@code afterJointFrame} with respect to the {@code beforeJointFrame} expressed in
    * the {@code afterJointFrame}.
    * </ul>
    * 
    * @param jointVelocity the column vector from which the current velocity of this joint is to be
    *           extracted. Not modified.
    * @param rowStart row index of the first component of this joint velocity.
    */
   public abstract void setVelocity(DenseMatrix64F jointVelocity, int rowStart);

   /**
    * Sets this joint desired acceleration from the given column vector {@code DenseMAtrix64F}. Here
    * are a few examples:
    * <ul>
    * <li>For a {@code RevoluteJoint}, the {@code rowStart}<sup>th</sup> row of the given column
    * vector is used to set the joint desired angular acceleration {@code qddDesired}.
    * <li>For a {@code SixDoFJoint}, the 6 rows starting from {@code rowStart} are use to set the
    * desired spatial acceleration of this joint starting with the angular acceleration. Note: the
    * joint spatial acceleration is the acceleration of the {@code afterJointFrame} with respect to
    * the {@code beforeJointFrame} expressed in the {@code afterJointFrame}.
    * </ul>
    * 
    * @param jointDesiredAcceleration the column vector from which the desired acceleration of this
    *           joint is to be extracted. Not modified.
    * @param rowStart row index of the first component of this joint acceleration.
    */
   public abstract void setDesiredAcceleration(DenseMatrix64F jointDesiredAcceleration, int rowStart);

   /**
    * Attach the {@code RigidBody} located after this joint, i.e. the {@code successor}. "After"
    * means that the {@code RigidBody} sits between the joint and the end-effector of the kinematics
    * chain, or that its is the end-effector. This method is to be called <b>only once</b>, at
    * creation of a kinematics chain. It is also in this method, that the final steps of
    * initialization of this joint are made.
    * 
    * @param successor the {@code RigidBody} located after this joint.
    */
   public abstract void setSuccessor(RigidBody successor);

   /**
    * Returns the motion subspace of this joint in the form of {@code GeometricJacobian}. Its number
    * of columns is equal to the number of degrees of freedom of this joint. It is expressed in:
    * <ul>
    * <li>{@code afterJointFrame} for: {@code SixDoFJoint}, {@code SphericalJoint},
    * {@code PlanarJoint}.
    * <li>{@code successorFrame} for: {@code RevoluteJoint} and {@code PrismaticJoint}.
    * </ul>
    * 
    * @return the motion subspace of this joint.
    */
   public abstract GeometricJacobian getMotionSubspace();

   /**
    * Retrieves the unit-twist corresponding to the {@code dofIndex}<sup>th</sup> degree of freedom
    * of this joint.
    * <p>
    * Unit-twist are mostly used to compute a Jacobian.
    * </p>
    * <p>
    * For instance, the unit-twist for a {@link RevoluteJoint} about the the y-axis is equal to [0,
    * 1, 0, 0, 0, 0]<sup>T</sup> with {@code body = joint.getSuccessor()},
    * {@code base = joint.getSuccessor()}, and expressed in the predecessor body-fixed frame.
    * </p>
    * 
    * @param dofIndex index used to specify for which degree of freedom of this joint the unit-twist
    *           should retrieved.
    * @param unitTwistToPack a twist used to stored one of the unit-twists of this joint. Modified.
    */
   public abstract void getUnitTwist(int dofIndex, Twist unitTwistToPack);

   /**
    * Update the motion subspace of this joint. It is only necessary for when the motion subspace
    * depends on this joint configuration. A good example is a four bar linkage, for which the
    * motion subspace depends on the linkage configuration.
    */
   public abstract void updateMotionSubspace();

   /**
    * Returns the {@code RigidBody} that precedes this joint. In other words, the {@code RigidBody}
    * directly connected to this joint that sits between this joint and the root or that is the root
    * of this kinematics chain.
    * 
    * @return the {@code predecessor} of this joint.
    */
   public abstract RigidBody getPredecessor();

   /**
    * Returns the {@code RigidBody} that succeeds this joint. In other words, the {@code RigidBody}
    * directly connected to this joint that sits between this joint and the end-effector or that is
    * the end-effector of this kinematics chain.
    * 
    * @return the {@code successor} of this joint.
    */
   public abstract RigidBody getSuccessor();

   /**
    * Sets the desired force/torque of this joint from the given {@code Wrench} using this joint
    * motion subspace. The given {@code Wrench} is expected to have the following frames:
    * <ul>
    * <li>{@code bodyFrame} should be {@code successorFrame == successor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} should be
    * {@code successorFrame == successor.getBodyFixedFrame()}.
    * </ul>
    * <p>
    * For instance, for a {@code RevoluteJoint} that can rotate around the y-axis, this method will
    * extract the torque around the y-axis from the given {@code Wrench} and use it to update the
    * {@code RevolueJoint}'s desired torque.
    * </p>
    * 
    * @param jointWrench the {@code Wrench} from which the desired force/torque of this joint is to
    *           be extracted. Not modified.
    */
   public abstract void setTorqueFromWrench(Wrench jointWrench);

   /**
    * Returns the reference to the name of this joint.
    * 
    * @return the name of this joint.
    */
   public abstract String getName();

   /**
    * Updates {@code afterJointFrame} of this joint to take into consideration the new joint
    * configuration. Then calls {@link RigidBody#updateFramesRecursively()} which in its turn
    * updates its {@code bodyFixedFrame} and then {@link #updateFramesRecursively()} for all of its
    * {@link InverseDynamicsJoint} child.
    * <p>
    * As a result, this method will update all the reference frames of the subtree starting from
    * this joint.
    * </p>
    */
   public abstract void updateFramesRecursively();

   /**
    * Returns the number of degrees of freedom that this joint has.
    * 
    * @return the number of degrees of freedom for this joint.
    */
   public abstract int getDegreesOfFreedom();

   /**
    * In most cases, this is the same as {@link #getDegreesOfFreedom()}. However, for
    * {@code SixDoFJoint} and {@code SphericalJoint} this method will return
    * {@code getDegreesOfFreedom() + 1}. This is due to the orientation being represented as a
    * quaternion which has 4 components.
    * 
    * @return the size needed to pack this joint configuration into a matrix.
    */
   public abstract int getConfigurationMatrixSize();

   /**
    * Copies the actual state of {@code originalJoint} into this joint without copying the desireds,
    * such as desired acceleration or force/torque.
    * 
    * @param originalJoint the joint to copy the actual (not desired) state of. Not modified.
    */
   public abstract void setJointPositionVelocityAndAcceleration(InverseDynamicsJoint originalJoint);

   /**
    * Copies the desired acceleration of {@code originalJoint} into this joint
    * 
    * @param originalJoint the joint to copy the desired acceleration of. Not modified.
    */
   public abstract void setQddDesired(InverseDynamicsJoint originalJoint);

   public abstract void calculateJointStateChecksum(GenericCRC32 checksum);

   public abstract void calculateJointDesiredChecksum(GenericCRC32 checksum);
}
