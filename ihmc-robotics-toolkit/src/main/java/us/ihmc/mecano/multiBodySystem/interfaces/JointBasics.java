package us.ihmc.mecano.multiBodySystem.interfaces;

import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.robotics.screwTheory.CentroidalMomentumMatrix;
import us.ihmc.robotics.screwTheory.CentroidalMomentumRateTermCalculator;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;

/**
 * Base interface that describes the basic API for the joints composing a kinematic chain/tree
 * compatible with the screw theory. Here are a few examples of useful screw tools:
 * <ul>
 * <li>The {@link GeometricJacobian} can compute the Jacobian matrix of a kinematic chain.
 * <li>The {@link SpatialAccelerationCalculator} can compute all the spatial accelerations of all
 * the {@code RigidBody}s of a kinematic chain/tree.
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
public abstract interface JointBasics
{
   public static int MAX_NUMBER_OF_DOFS = 6;

   /**
    * Returns the the {@code MovingReferenceFrame} that is attached to the predecessor of this joint,
    * namely the {@code RigidBody} before this joint, and has its origin centered at the joint origin.
    * The pose of the {@code frameBeforeJoint} is independent from this joint motion.
    * 
    * @return the {@code MovingReferenceFrame} located right before this joint.
    */
   public abstract MovingReferenceFrame getFrameBeforeJoint();

   /**
    * Returns the the {@code MovingReferenceFrame} that is attached to the successor of this joint,
    * namely the {@code RigidBody} after this joint, and has its origin centered at the joint origin.
    * The pose of the {@code frameAfterJoint} will change as this joint moves.
    * 
    * @return the {@code MovingReferenceFrame} located right after this joint.
    */
   public abstract MovingReferenceFrame getFrameAfterJoint();

   default TwistReadOnly getJointTwist()
   {
      return null;
   }

   default void getJointOffset(RigidBodyTransform transform)
   {
      
   }

   default void getJointConfiguration(RigidBodyTransform transform)
   {
      
   }

   /**
    * Packs the {@code Twist} (the 3D angular and linear velocities) of this joint's {@code successor}
    * with respect to this joint's {@code predecessor}. The reference frames of the resulting
    * {@code Twist} are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code successorFrame == successor.getBodyFixedFrame()}.
    * <li>{@code baseFrame} is {@code predecessorFrame == predecessor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} is {@code successorFrame}.
    * </ul>
    * 
    * @param twistToPack the {@code Twist} in which the velocity of this joint's {@code successor} is
    *           stored. Modified.
    */
   public abstract void getSuccessorTwist(Twist twistToPack);

   /**
    * Packs the {@code Twist} (the 3D angular and linear velocities) of this joint's
    * {@code predecessor} with respect to this joint's {@code successor}. The reference frames of the
    * resulting {@code Twist} are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code predecessorFrame == predecessor.getBodyFixedFrame()}.
    * <li>{@code baseFrame} is {@code successorFrame == successor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} is {@code predecessorFrame}.
    * </ul>
    * 
    * @param twistToPack the {@code Twist} in which the velocity of this joint's {@code predecessor} is
    *           stored. Modified.
    */
   public abstract void getPredecessorTwist(Twist twistToPack);

   default SpatialAccelerationReadOnly getJointAcceleration()
   {
      return null;
   }

   /**
    * Packs the {@code SpatialAccelerationVector} (the 3D angular and linear accelerations) of this
    * joint's {@code successor} with respect to this joint's {@code predecessor}. The reference frames
    * of the resulting {@code SpatialAccelerationVector} are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code successorFrame == successor.getBodyFixedFrame()}.
    * <li>{@code baseFrame} is {@code predecessorFrame == predecessor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} is {@code successorFrame}.
    * </ul>
    * 
    * @param jointAccelerationToPack the {@code SpatialAccelerationVector} in which the acceleration of
    *           this joint's {@code successor} is stored. Modified.
    */
   public abstract void getSuccessorAcceleration(SpatialAcceleration jointAccelerationToPack);

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
    * @param rowStart row index for the first component of the configuration.
    * @param matrixToPack the column vector in which this joint actual configuration is stored.
    *           Modified.
    */
   public abstract void getJointConfiguration(int rowStart, DenseMatrix64F matrixToPack);

   /**
    * Packs this joint desired force/torque into a column vector {@code DenseMatrix64F}. Here are a few
    * examples:
    * <ul>
    * <li>For a {@code RevoluteJoint}, the desired joint torque is stored at the 1<sup>st</sup> row.
    * <li>For a {@code PrismaticJoint}, the desired joint force is stored at the 1<sup>st</sup> row.
    * <li>For a {@code SixDoFJoint}, the desired wrench (the 3D torque and 3D force) of this joint is
    * stored from the {@code rowStart}<sup>th</sup> row to the ({@code rowStart + 6})<sup>th</sup> row,
    * starting with the three components of the torque. Note: the joint wrench is the wrench of
    * {@code successorFrame} expressed in {@code successorFrame}.
    * </ul>
    * 
    * @param rowStart TODO
    * @param matrixToPack the column vector in which the desired force/torque of this joint is stored.
    *           Modified.
    */
   public abstract void getJointTau(int rowStart, DenseMatrix64F matrixToPack);

   /**
    * Packs this joint actual velocity into a column vector {@code DenseMatrix64F}. Here are a few
    * examples:
    * <ul>
    * <li>For a {@code OneDoFJoint}, the scalar velocity {@code qd} is stored at the
    * {@code rowStart}<sup>th</sup> row.
    * <li>For a {@code SixDoFJoint}, the joint twist is stored from the {@code rowStart}<sup>th</sup>
    * row to the ({@code rowStart + 6})<sup>th</sup> row, starting with the three components of the
    * angular velocity. Note: the joint twist is the twist of the {@code afterJointFrame} with respect
    * to the {@code beforeJointFrame} expressed in the {@code afterJointFrame}.
    * </ul>
    * 
    * @param rowStart row index for the first component of the velocity.
    * @param matrixToPack the column vector in which the velocity of this joint is stored. Modified.
    */
   public abstract void getJointVelocity(int rowStart, DenseMatrix64F matrixToPack);

   public abstract void getJointAcceleration(int rowStart, DenseMatrix64F matrixToPack);

   public abstract void setJointAccelerationToZero();

   /**
    * Sets the joint current configuration from the given column vector {@code DenseMatrix64F}. Here
    * are a few examples:
    * <ul>
    * <li>For a {@code RevoluteJoint}, the {@code rowStart}<sup>th</sup> row of the given column vector
    * is used to set the current joint angle {@code q}.
    * <li>For a {@code SixDoFJoint}, the 4 rows starting from {@code rowStart} are used to set the
    * current 3D orientation as a quaternion, and the 3 rows starting from ({@code rowStart + 4}) are
    * used to set the 3D position.
    * </ul>
    * 
    * @param rowStart row index of the first component of this joint configuration.
    * @param matrix the column vector from which the configuration of this joint is to be extracted.
    *           Not modified.
    */
   public abstract void setJointConfiguration(int rowStart, DenseMatrix64F matrix);

   /**
    * Sets the joint current wrench from the given column vector {@code DenseMatrix64F}. Here are a few
    * examples:
    * <ul>
    * <li>For a {@code RevoluteJoint}, the {@code rowStart}<sup>th</sup> row of the given column vector
    * is used to set the joint torque {@code tau}.
    * <li>For a {@code SixDoFJoint}, the 6 rows starting from {@code rowStart} are use to set the
    * current spatial wrench of this joint starting with the torque. Note: the joint wrench is the
    * wrench of the {@code afterJointFrame} with respect to the {@code beforeJointFrame} expressed in
    * the {@code afterJointFrame}.
    * </ul>
    * 
    * @param rowStart row index of the first component of this joint configuration.
    * @param matrixToPack the column vector from which the configuration of this joint is to be
    *           extracted. Not modified.
    */
   public abstract void setJointTau(int rowStart, DenseMatrix64F matrixToPack);

   /**
    * Sets this joint current velocity from the given column vector {@code DenseMAtrix64F}. Here are a
    * few examples:
    * <ul>
    * <li>For a {@code RevoluteJoint}, the {@code rowStart}<sup>th</sup> row of the given column vector
    * is used to set the joint current angular velocity {@code qd}.
    * <li>For a {@code SixDoFJoint}, the 6 rows starting from {@code rowStart} are use to set the
    * current twist of this joint starting with the angular velocity. Note: the joint twist is the
    * twist of the {@code afterJointFrame} with respect to the {@code beforeJointFrame} expressed in
    * the {@code afterJointFrame}.
    * </ul>
    * 
    * @param rowStart row index of the first component of this joint velocity.
    * @param jointVelocity the column vector from which the current velocity of this joint is to be
    *           extracted. Not modified.
    */
   public abstract void setJointVelocity(int rowStart, DenseMatrix64F jointVelocity);

   public abstract void setJointAcceleration(int rowStart, DenseMatrix64F jointDesiredAcceleration);

   /**
    * Attach the {@code RigidBody} located after this joint, i.e. the {@code successor}. "After" means
    * that the {@code RigidBody} sits between the joint and the end-effector of the kinematics chain,
    * or that its is the end-effector. This method is to be called <b>only once</b>, at creation of a
    * kinematics chain. It is also in this method, that the final steps of initialization of this joint
    * are made.
    * 
    * @param successor the {@code RigidBody} located after this joint.
    */
   public abstract void setSuccessor(RigidBodyBasics successor);

   default void getMotionSubspace(DenseMatrix64F matrixToPack)
   {
   }

   default List<TwistReadOnly> getUnitTwists()
   {
      return null;
   }

   /**
    * Update the motion subspace of this joint. It is only necessary for when the motion subspace
    * depends on this joint configuration. A good example is a four bar linkage, for which the motion
    * subspace depends on the linkage configuration.
    */
   public abstract void updateMotionSubspace();

   /**
    * Returns the {@code RigidBody} that precedes this joint. In other words, the {@code RigidBody}
    * directly connected to this joint that sits between this joint and the root or that is the root of
    * this kinematics chain.
    * 
    * @return the {@code predecessor} of this joint.
    */
   public abstract RigidBodyBasics getPredecessor();

   /**
    * Returns the {@code RigidBody} that succeeds this joint. In other words, the {@code RigidBody}
    * directly connected to this joint that sits between this joint and the end-effector or that is the
    * end-effector of this kinematics chain.
    * 
    * @return the {@code successor} of this joint.
    */
   public abstract RigidBodyBasics getSuccessor();

   /**
    * Sets the desired force/torque of this joint from the given {@code Wrench} using this joint motion
    * subspace. The given {@code Wrench} is expected to have the following frames:
    * <ul>
    * <li>{@code bodyFrame} should be {@code successorFrame == successor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} should be {@code successorFrame == successor.getBodyFixedFrame()}.
    * </ul>
    * <p>
    * For instance, for a {@code RevoluteJoint} that can rotate around the y-axis, this method will
    * extract the torque around the y-axis from the given {@code Wrench} and use it to update the
    * {@code RevolueJoint}'s desired torque.
    * </p>
    * 
    * @param jointWrench the {@code Wrench} from which the desired force/torque of this joint is to be
    *           extracted. Not modified.
    */
   public abstract void setJointWrench(Wrench jointWrench);

   /**
    * Returns the reference to the name of this joint.
    * 
    * @return the name of this joint.
    */
   public abstract String getName();

   /**
    * Updates {@code afterJointFrame} of this joint to take into consideration the new joint
    * configuration. Then calls {@link RigidBodyBasics#updateFramesRecursively()} which in its turn
    * updates its {@code bodyFixedFrame} and then {@link #updateFramesRecursively()} for all of its
    * {@link JointBasics} child.
    * <p>
    * As a result, this method will update all the reference frames of the subtree starting from this
    * joint.
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

   default void setJointConfiguration(JointBasics other)
   {
   }

   default void setJointTwist(JointBasics other)
   {
   }

   default void setJointAcceleration(JointBasics other)
   {
   }

   public int hashCode();
}
