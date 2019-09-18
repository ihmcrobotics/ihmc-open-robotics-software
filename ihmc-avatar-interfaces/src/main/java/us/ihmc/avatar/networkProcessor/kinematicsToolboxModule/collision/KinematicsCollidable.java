package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

/**
 * Defines a collision shape that represents entirely or partially the geometry of a rigid-body.
 * <p>
 * Multiple {@link KinematicsCollidable}s can be used to more accurately represent a rigid-body.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class KinematicsCollidable
{
   /** The rigid-body this collidable represents. */
   private final RigidBodyBasics rigidBody;
   /**
    * Collision identifier for this collidable. Use {@link KinematicsCollidableHelper} to compute
    * collision masks and groups.
    */
   private final int collisionMask;
   /**
    * Collision identifiers of other collidables that this collidable is allowed to collide with. Use
    * {@link KinematicsCollidableHelper} to compute collision masks and groups.
    */
   private final int collisionGroup;
   /**
    * The shape of this collidable. It is strongly recommended to use only {@link Sphere3D} and
    * {@link Capsule3D} as collision evaluations are extremely fast with these shapes.
    */
   private final Shape3DReadOnly shape;
   /**
    * The frame the shape is expressed in. Usually the body-fixed frame of {@code rigidBody} or the
    * frame after it parent joint.
    */
   private final ReferenceFrame shapeFrame;
   /**
    * TODO: This feature is only partially implemented and is meant to define an additional region
    * enveloping the shape that needs to be treated more carefully. At this moment, it used to grow the
    * shape.
    */
   private final double minimumSafeDistance;

   /**
    * Creates a new collidable that represents entirely or partially the geometry of the given
    * {@code rigidBody}.
    * 
    * @param rigidBody      the rigid-body this collidable represents.
    * @param collisionMask  collision identifier for this collidable. Use
    *                       {@link KinematicsCollidableHelper} to compute collision masks and groups.
    * @param collisionGroup collision identifiers of other collidables that this collidable is allowed
    *                       to collide with. Use {@link KinematicsCollidableHelper} to compute
    *                       collision masks and groups.
    * @param shape          the shape of this collidable. It is strongly recommended to use only
    *                       {@link Sphere3D} and {@link Capsule3D} as collision evaluations are
    *                       extremely fast with these shapes.
    * @param shapeFrame     the frame the shape is expressed in. Usually the body-fixed frame of
    *                       {@code rigidBody} or the frame after it parent joint.
    */
   public KinematicsCollidable(RigidBodyBasics rigidBody, int collisionMask, int collisionGroup, Shape3DReadOnly shape, ReferenceFrame shapeFrame)
   {
      this(rigidBody, collisionMask, collisionGroup, shape, shapeFrame, 0.0);
   }

   /**
    * Creates a new collidable that represents entirely or partially the geometry of the given
    * {@code rigidBody}.
    * 
    * @param rigidBody           the rigid-body this collidable represents.
    * @param collisionMask       collision identifier for this collidable. Use
    *                            {@link KinematicsCollidableHelper} to compute collision masks and
    *                            groups.
    * @param collisionGroup      collision identifiers of other collidables that this collidable is
    *                            allowed to collide with. Use {@link KinematicsCollidableHelper} to
    *                            compute collision masks and groups.
    * @param shape               the shape of this collidable. It is strongly recommended to use only
    *                            {@link Sphere3D} and {@link Capsule3D} as collision evaluations are
    *                            extremely fast with these shapes.
    * @param shapeFrame          the frame the shape is expressed in. Usually the body-fixed frame of
    *                            {@code rigidBody} or the frame after it parent joint.
    * @param minimumSafeDistance this feature is only partially implemented and is meant to define an
    *                            additional region enveloping the shape that needs to be treated more
    *                            carefully. At this moment, it used to grow the shape.
    */
   public KinematicsCollidable(RigidBodyBasics rigidBody, int collisionMask, int collisionGroup, Shape3DReadOnly shape, ReferenceFrame shapeFrame,
                               double minimumSafeDistance)
   {
      this.rigidBody = rigidBody;
      this.collisionMask = collisionMask;
      this.collisionGroup = collisionGroup;
      this.shape = shape;
      this.shapeFrame = shapeFrame;
      this.minimumSafeDistance = minimumSafeDistance;
   }

   /**
    * Performs a quick test to check if this collidable and {@code other} are allowed to collide with
    * each other regarding their respective identifiers.
    * 
    * @param other the query.
    * @return {@code true} if the 2 collidables are allowed to collide, {@code false} ortherwise.
    */
   public boolean isCollidableWith(KinematicsCollidable other)
   {
      if ((collisionGroup & other.collisionMask) == 0x00)
         return false;
      if ((other.collisionGroup & collisionMask) == 0x00)
         return false;
      return true;
   }

   /**
    * Performs a collision evaluation between this collidable and {@code other} in order to calculate
    * their closest point, separating/penetration distance, etc.
    * 
    * @param other the query.
    * @return the result of the evaluation.
    */
   public KinematicsCollisionResult evaluateCollision(KinematicsCollidable other)
   {
      KinematicsCollisionResult result = KinematicsCollisionTools.evaluateShape3DShape3DCollision(shape, shapeFrame, other.shape, other.shapeFrame);
      result.setCollidableA(this);
      result.setCollidableB(other);
      return result;
   }

   /**
    * Get the rigid-body this collidable represents.
    * 
    * @return the rigid-body this collidable represents.
    */
   public RigidBodyBasics getRigidBody()
   {
      return rigidBody;
   }

   /**
    * Collision identifier for this collidable used with
    * {@link #isCollidableWith(KinematicsCollidable)}.
    * 
    * @return the mask's value.
    */
   public int getCollisionMask()
   {
      return collisionMask;
   }

   /**
    * Collision identifiers of other collidables that this collidable is allowed to collide with. It is
    * used with {@link #isCollidableWith(KinematicsCollidable)}.
    * 
    * @return the group's value.
    */
   public int getCollisionGroup()
   {
      return collisionGroup;
   }

   /**
    * The shape of this collidable.
    * 
    * @return the shape.
    */
   public Shape3DReadOnly getShape()
   {
      return shape;
   }

   /**
    * The frame the shape is expressed in.
    * 
    * @return the shape frame.
    */
   public ReferenceFrame getShapeFrame()
   {
      return shapeFrame;
   }

   /**
    * Gets the distance from the shape that needs to be treated more carefully.
    * 
    * @return the distance value.
    */
   public double getMinimumSafeDistance()
   {
      return minimumSafeDistance;
   }
}
