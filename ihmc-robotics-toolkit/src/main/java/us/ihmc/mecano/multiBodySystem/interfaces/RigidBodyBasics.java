package us.ihmc.mecano.multiBodySystem.interfaces;

import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;

public interface RigidBodyBasics
{

   /**
    * Gets the internal reference to the object to contains the information about this rigid-body
    * physical properties.
    *
    * @return the reference to this rigid-body's inertia.
    */
   SpatialInertiaBasics getInertia();

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
   MovingReferenceFrame getBodyFixedFrame();

   /**
    * Gets the internal reference to the parent joint of this rigid-body.
    * <p>
    * The parent joint is the joint directed connected to this rigid-body and located between this
    * and the root body of the robot.
    * </p>
    *
    * @return the reference to the parent joint.
    */
   JointBasics getParentJoint();

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
   void addChildJoint(JointBasics joint);

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
   List<JointBasics> getChildrenJoints();

   /**
    * Verifies whether this rigid-body has at least one child joint or not.
    * <p>
    * A rigid-body without any children joint is usually referred to as an end-effector.
    * </p>
    *
    * @return {@code true} if this rigid-body has at least one child joint, {@code false} otherwise.
    */
   boolean hasChildrenJoints();

   /**
    * Verifies whether this rigid-body is the root body of a robot.
    * <p>
    * Internally, this verifies if this rigid-body has a parent joint. By definition, the root body
    * of a robot does not have any parent joint.
    * </p>
    *
    * @return {@code true} if this is the root body, {@code false} otherwise.
    */
   boolean isRootBody();

   /**
    * Gets the name of this rigid-body.
    * <p>
    * Each rigid-body of a robot should have a unique name.
    * </p>
    *
    * @return this rigid-body's name.
    */
   String getName();

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
   void getCenterOfMass(FramePoint3D comOffsetToPack);

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
   void setCenterOfMass(FramePoint3D comOffset);

   /**
    * Request all the children joints of this rigid-body to update their reference frame and request
    * their own successor to call this same method.
    * <p>
    * By calling this method on the root body of a robot, it will update the reference frames of all
    * the robot's joints.
    * </p>
    */
   void updateFramesRecursively();

   /**
    * Returns the name of this rigid-body.
    * <p>
    * This should probably include more information such as the mass.
    * </p>
    */
   String toString();

   /**
    * Returns the value of a secondary unique hash code representing this rigid-body that is
    * computed based on {@link #name} and the parent joint name if any.
    * <p>
    * This hash code has the benefit of remaining the same when creating several instances of the
    * same robot, such that it can be used to serialize and deserialize robot information.
    * </p>
    */
   int hashCode();

}