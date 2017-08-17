package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.utils.NameBasedHashCodeHolder;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;

/**
 * {@code RigidBody} describes a link which used with {@code InverseDynamicsJoint}s describe a robot
 * that can be used with our screw theory library.
 * <p>
 * This class gathers notably the physical properties of the link and its position in the robot,
 * i.e. its parent and children joints.
 * </p>
 * <p>
 * Here are a few examples of useful screw tools:
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
 * </p>
 */
public class RigidBody implements NameBasedHashCodeHolder
{
   /** This is where the physical properties of this rigid-body are stored. */
   private final RigidBodyInertia inertia;
   /**
    * This is a reference rigidly attached this rigid-body. It's usually centered at the
    * rigid-body's center of mass.
    */
   private final MovingReferenceFrame bodyFixedFrame;
   /**
    * The parent joint is the joint directly connected to a rigid-body and located between the
    * rigid-body and the robot's root.
    * <p>
    * The parent joint is equal to {@code null} when this rigid-body represents the root of the
    * robot.
    * </p>
    */
   private final InverseDynamicsJoint parentJoint;
   /**
    * The children joints are all the joints that are directly connected to a rigid-body and located
    * between the rigid-body and any end-effector of the robot.
    * <p>
    * There is no children joint when this rigid-body is an end-effector.
    * </p>
    */
   private final List<InverseDynamicsJoint> childrenJoints = new ArrayList<>();
   private final List<InverseDynamicsJoint> childrenJointsReadOnly = Collections.unmodifiableList(childrenJoints);
   /**
    * The name of this rigid-body. It is important that this name is unique among all the
    * rigid-bodies of a robot.
    */
   private final String name;

   /**
    * A secondary unique hash code representing this rigid-body that is computed based on
    * {@link #name} and the parent joint name if any.
    * <p>
    * This hash code has the benefit of remaining the same when creating several instances of the
    * same robot, such that it can be used to serialize and deserialize robot information.
    * </p>
    */
   private final long nameBasedHashCode;

   /**
    * Creates a new root rigid-body to which the first joint of a robot kinematic chain can be
    * added.
    * <p>
    * Note that there is only one root body per robot.
    * </p>
    *
    * @param bodyName the name for this rigid-body.
    * @param parentStationaryFrame the parent stationary, i.e. non-moving with respect to world
    *           frame, frame to which this rigid-body will create and attach its body fixed frame.
    *           Most of the time {@code parentStationaryFrame == ReferenceFrame.getWorldFrame()}.
    */
   public RigidBody(String bodyName, ReferenceFrame parentStationaryFrame)
   {
      this(bodyName, new RigidBodyTransform(), parentStationaryFrame);
   }

   /**
    * Creates a new root rigid-body to which the first joint of a robot kinematic chain can be
    * added.
    * <p>
    * Note that there is only one root body per robot.
    * </p>
    *
    * @param bodyName the name for this rigid-body.
    * @param transformToParent provides the pose of this rigid-body's body-fixed-frame with respect
    *           to the {@code parentStationaryFrame}. Not modified.
    * @param parentStationaryFrame the parent stationary, i.e. non-moving with respect to world
    *           frame, frame to which this rigid-body will create and attach its body fixed frame.
    *           Most of the time {@code parentStationaryFrame == ReferenceFrame.getWorldFrame()}.
    */
   public RigidBody(String bodyName, RigidBodyTransform transformToParent, ReferenceFrame parentStationaryFrame)
   {
      if (bodyName == null)
         throw new IllegalArgumentException("Name can not be null");

      nameBasedHashCode = NameBasedHashCodeTools.computeStringHashCode(bodyName);
      name = bodyName;
      inertia = null;
      bodyFixedFrame = MovingReferenceFrame.constructFrameFixedInParent(bodyName + "Frame", parentStationaryFrame, transformToParent);
      parentJoint = null;
   }

   /**
    * Creates a new rigid-body that is setup as the successor of the given {@code parentJoint}.
    *
    * @param bodyName the name for this rigid-body.
    * @param parentJoint the joint directly attached to this rigid-body and located between this
    *           rigid-body and the root body of the robot.
    * @param momentOfInertia the 3D moment of inertia of this rigid-body. Not modified.
    * @param mass the mass of this rigid-body.
    * @param inertiaPose defines the transform from this rigid-body body-fixed-frame to the
    *           {@code parentJoint.getFrameAfterJointFrame()}. The given moment of inertia is
    *           assumed to be expressed in that body-fixed-frame. Also note that the translation
    *           part corresponds to the position of this rigid-body center of mass position
    *           expressed in {@code parentJoint.getFrameAfterJointFrame()}. Not modified.
    */
   public RigidBody(String bodyName, InverseDynamicsJoint parentJoint, Matrix3DReadOnly momentOfInertia, double mass, RigidBodyTransform inertiaPose)
   {
      if (bodyName == null)
         throw new IllegalArgumentException("Name can not be null");

      name = bodyName;
      this.parentJoint = parentJoint;

      nameBasedHashCode = NameBasedHashCodeTools.combineHashCodes(name, parentJoint);
      ReferenceFrame frameAfterJoint = parentJoint.getFrameAfterJoint();
      bodyFixedFrame = MovingReferenceFrame.constructFrameFixedInParent(bodyName + "CoM", frameAfterJoint, inertiaPose);
      inertia = new RigidBodyInertia(bodyFixedFrame, momentOfInertia, mass);
      inertia.getBodyFrame().checkReferenceFrameMatch(inertia.getExpressedInFrame()); // inertia should be expressed in body frame, otherwise it will change
      parentJoint.setSuccessor(this);
   }

   /**
    * Gets the internal reference to the object to contains the information about this rigid-body
    * physical properties.
    *
    * @return the reference to this rigid-body's inertia.
    */
   public RigidBodyInertia getInertia()
   {
      return inertia;
   }

   /**
    * Gets the internal reference to this rigid-body's body-fixed-frame.
    * <p>
    * The main property of the body-fixed-frame is that it is rigidly attached to this rigid-body.
    * </p>
    * <p>
    * Unless it has been changed in runtime, the body-fixed-frame is also centered at this
    * rigid-body's center of mass position.
    * </p>
    * <p>
    * Note that this frame is not particularly interesting for control. It is very often the case
    * that a rigid-body is controlled through the use of a 'control frame' that is also rigidly
    * attached to the rigid-body but positioned at a point of interest and with a more intuitive
    * orientation. For instance, given a rigid-body that represents a hand, the control frame would
    * be located around the grasping location and orientation such that the x-axis is collinear to
    * the main grasping direction.
    * </p>
    *
    * @return the reference frame attached to this rigid-body.
    */
   public MovingReferenceFrame getBodyFixedFrame()
   {
      return bodyFixedFrame;
   }

   /**
    * Gets the internal reference to the parent joint of this rigid-body.
    * <p>
    * The parent joint is the joint directed connected to this rigid-body and located between this
    * and the root body of the robot.
    * </p>
    *
    * @return the reference to the parent joint.
    */
   public InverseDynamicsJoint getParentJoint()
   {
      return parentJoint;
   }

   /**
    * Registers a new child joint to this rigid-body.
    * <p>
    * This method should only be called when building the robot.
    * </p>
    * <p>
    * A child joint is a joint directly connected to this rigid-body and located between this and an
    * end-effector of the robot.
    * </p>
    *
    * @param joint the new child joint to register to this rigid-body.
    */
   public void addChildJoint(InverseDynamicsJoint joint)
   {
      childrenJoints.add(joint);
   }

   /**
    * Gets the unmodifiable list of all the children joints that have been registered to this
    * rigid-body.
    * <p>
    * A child joint is a joint directly connected to this rigid-body and located between this and an
    * end-effector of the robot.
    * </p>
    *
    * @return all the children joints of this rigid-body.
    */
   public List<InverseDynamicsJoint> getChildrenJoints()
   {
      return childrenJointsReadOnly;
   }

   /**
    * Verifies whether this rigid-body has at least one child joint or not.
    * <p>
    * A rigid-body without any children joint is usually referred to as an end-effector.
    * </p>
    *
    * @return {@code true} if this rigid-body has at least one child joint, {@code false} otherwise.
    */
   public boolean hasChildrenJoints()
   {
      return !childrenJoints.isEmpty();
   }

   /**
    * Verifies whether this rigid-body is the root body of a robot.
    * <p>
    * Internally, this verifies if this rigid-body has a parent joint. By definition, the root body
    * of a robot does not have any parent joint.
    * </p>
    *
    * @return {@code true} if this is the root body, {@code false} otherwise.
    */
   public boolean isRootBody()
   {
      return parentJoint == null;
   }

   /**
    * Gets the name of this rigid-body.
    * <p>
    * Each rigid-body of a robot should have a unique name.
    * </p>
    *
    * @return this rigid-body's name.
    */
   public String getName()
   {
      return name;
   }

   /**
    * Packs this rigid-body's center of mass coordinates in the given argument.
    * <p>
    * Note that unless modified in runtime, this method outputs {@code (0, 0, 0)} in the
    * body-fixed-frame. This is due to the fact that by default the body-fixed-frame is centered at
    * this rigid-body's center of mass position.
    * </p>
    *
    * @param comOffsetToPack the {@code FramePoint} in which the center of mass position is stored.
    *           Modified.
    */
   public void getCoMOffset(FramePoint3D comOffsetToPack)
   {
      inertia.getCenterOfMassOffset(comOffsetToPack);
   }

   /**
    * Changes the position of this rigid-body's center of mass.
    * <p>
    * Note that the center of mass position should only be set at construction time and that
    * changing it at runtime might result in undesirable effects as some modules may not anticipate
    * such a change or may not consider the case where the body-fixed-frame is not centered at the
    * center of mass position.
    * </p>
    *
    * @param comOffset the new coordinates of this rigid-body's center of mass position. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the
    *            {@code bodyFixedFrame}.
    */
   public void setCoMOffset(FramePoint3D comOffset)
   {
      inertia.setCenterOfMassOffset(comOffset);
   }

   /**
    * Request all the children joints of this rigid-body to update their reference frame and request
    * their own successor to call this same method.
    * <p>
    * By calling this method on the root body of a robot, it will update the reference frames of all
    * the robot's joints.
    * </p>
    */
   public void updateFramesRecursively()
   {
      bodyFixedFrame.update();

      //      for (InverseDynamicsJoint joint : childrenJoints)
      for (int childIndex = 0; childIndex < childrenJoints.size(); childIndex++)
      {
         childrenJoints.get(childIndex).updateFramesRecursively();
      }
   }

   /**
    * Returns the name of this rigid-body.
    * <p>
    * This should probably include more information such as the mass.
    * </p>
    */
   @Override
   public String toString()
   {
      return name;
   }

   /**
    * Returns the value of a secondary unique hash code representing this rigid-body that is
    * computed based on {@link #name} and the parent joint name if any.
    * <p>
    * This hash code has the benefit of remaining the same when creating several instances of the
    * same robot, such that it can be used to serialize and deserialize robot information.
    * </p>
    */
   @Override
   public long getNameBasedHashCode()
   {
      return nameBasedHashCode;
   }
}
