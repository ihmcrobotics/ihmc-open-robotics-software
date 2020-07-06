package us.ihmc.robotics.physics;

import us.ihmc.euclid.referenceFrame.FrameBoundingBox3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;

/**
 * Defines a collision shape that represents entirely or partially the geometry of a rigid-body.
 * <p>
 * Multiple {@link Collidable}s can be used to more accurately represent a rigid-body.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class Collidable
{
   /** The rigid-body this collidable represents. */
   private final RigidBodyBasics rigidBody;
   /**
    * Collision identifier for this collidable. Use {@link CollidableHelper} to compute collision masks
    * and groups.
    */
   private final long collisionMask;
   /**
    * Collision identifiers of other collidables that this collidable is allowed to collide with. Use
    * {@link CollidableHelper} to compute collision masks and groups.
    */
   private final long collisionGroup;
   /**
    * The shape of this collidable. It is strongly recommended to use only {@link Sphere3D} and
    * {@link Capsule3D} as collision evaluations are extremely fast with these shapes.
    * <p>
    * Note that the frame in which the shape is expressed has to be rigidly attached to the rigid-body
    * and preferably be the frame after its parent joint as the location of the
    * {@link RigidBodyBasics#getBodyFixedFrame()} depends on the mass property of the body.
    * </p>
    */
   private final FrameShape3DReadOnly shape;
   /**
    * Bounding box for the shape in root frame.
    */
   private final FrameBoundingBox3D boundingBox = new FrameBoundingBox3D();
   /**
    * The root body that is the ancestor of {@code rigidBody}. This is useful to identify the
    * multi-body system this collidable belongs to.
    */
   private final RigidBodyBasics rootBody;

   private int collidableID = 0;

   private FrameShapePosePredictor predictor;

   /**
    * Creates a new collidable that represents entirely or partially the geometry of the given
    * {@code rigidBody}.
    *
    * @param rigidBody      the rigid-body this collidable represents.
    * @param collisionMask  collision identifier for this collidable. Use {@link CollidableHelper} to
    *                       compute collision masks and groups.
    * @param collisionGroup collision identifiers of other collidables that this collidable is allowed
    *                       to collide with. Use {@link CollidableHelper} to compute collision masks
    *                       and groups.
    * @param shape          the shape of this collidable. It is strongly recommended to use only
    *                       {@link Sphere3D} and {@link Capsule3D} as collision evaluations are
    *                       extremely fast with these shapes. Note that the frame in which the shape is
    *                       expressed has to be rigidly attached to the rigid-body and preferably be
    *                       the frame after its parent joint as the location of the
    *                       {@link RigidBodyBasics#getBodyFixedFrame()} depends on the mass property of
    *                       the body.
    * @param shapeFrame     the frame the shape is expressed in. It has to be a frame that is rigidly
    *                       attached to the rigid-body and preferably be the frame after it parent
    *                       joint as the location of the {@link RigidBodyBasics#getBodyFixedFrame()}
    *                       depends on the mass property of the body.
    */
   public Collidable(RigidBodyBasics rigidBody, long collisionMask, long collisionGroup, FrameShape3DReadOnly shape)
   {
      this.rigidBody = rigidBody;
      this.collisionMask = collisionMask;
      this.collisionGroup = collisionGroup;
      this.shape = shape;

      rootBody = rigidBody == null ? null : MultiBodySystemTools.getRootBody(rigidBody);
   }

   public void updateBoundingBox(ReferenceFrame boundingBoxFrame)
   {
      shape.getBoundingBox(boundingBoxFrame, boundingBox);
   }

   /**
    * Performs a quick test to check if this collidable and {@code other} are allowed to collide with
    * each other regarding their respective identifiers.
    *
    * @param other the query.
    * @return {@code true} if the 2 collidables are allowed to collide, {@code false} ortherwise.
    */
   public boolean isCollidableWith(Collidable other)
   {
      if (other == this)
         return false;
      if (collisionGroup == -1 && collisionMask == -1)
      {
         if (!boundingBox.intersectsEpsilon(other.boundingBox, 1.0e-12))
            return false;
         return true;
      }
      else
      {
         if ((collisionGroup & other.collisionMask) == 0x00)
            return false;
         if ((other.collisionGroup & collisionMask) == 0x00)
            return false;
         if (!boundingBox.intersectsEpsilon(other.boundingBox, 1.0e-12))
            return false;
      }
      return true;
   }

   public void setFrameShapePosePredictor(FrameShapePosePredictor predictor)
   {
      this.predictor = predictor;
   }

   /**
    * Performs a collision evaluation between this collidable and {@code other} in order to calculate
    * their closest point, separating/penetration distance, etc.
    *
    * @param other the query.
    * @return the result of the evaluation.
    */
   public CollisionResult evaluateCollision(Collidable other)
   {
      CollisionResult result = new CollisionResult();
      evaluateCollision(other, result);
      return result;
   }

   /**
    * Performs a collision evaluation between this collidable and {@code other} in order to calculate
    * their closest point, separating/penetration distance, etc.
    *
    * @param other        the query. Not modified.
    * @param resultToPack where the result of the evaluation is stored. Modified.
    */
   public void evaluateCollision(Collidable other, CollisionResult resultToPack)
   {
      PhysicsEngineTools.evaluateShape3DShape3DCollision(shape, other.shape, resultToPack.getCollisionData());
      resultToPack.setCollidableA(this);
      resultToPack.setCollidableB(other);
   }

   /**
    * Performs a collision evaluation between this collidable and {@code other} in order to calculate
    * their closest point, separating/penetration distance, etc.
    * <p>
    * Note that this method uses the predicted poses for each collidable one {@code dt} in the future.
    * </p>
    * 
    * @param dt           the integration period to use when predicting the pose of each collidable.
    * @param other        the query. Not modified.
    * @param resultToPack where the result of the evaluation is stored. Modified.
    */
   public void evaluateCollision(double dt, Collidable other, CollisionResult resultToPack)
   {
      PhysicsEngineTools.evaluateShape3DShape3DCollision(predictShape(dt), other.predictShape(dt), resultToPack.getCollisionData());
      resultToPack.setCollidableA(this);
      resultToPack.setCollidableB(other);
   }

   private FrameShape3DReadOnly predictShape(double dt)
   {
      if (predictor == null)
         return shape;

      return predictor.predictShape(shape, rigidBody, dt);
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
    * Collision identifier for this collidable used with {@link #isCollidableWith(Collidable)}.
    *
    * @return the mask's value.
    */
   public long getCollisionMask()
   {
      return collisionMask;
   }

   /**
    * Collision identifiers of other collidables that this collidable is allowed to collide with. It is
    * used with {@link #isCollidableWith(Collidable)}.
    *
    * @return the group's value.
    */
   public long getCollisionGroup()
   {
      return collisionGroup;
   }

   /**
    * The shape of this collidable.
    *
    * @return the shape.
    */
   public FrameShape3DReadOnly getShape()
   {
      return shape;
   }

   public RigidBodyBasics getRootBody()
   {
      return rootBody;
   }

   public boolean isEnvironment()
   {
      return rigidBody == null;
   }

   @Override
   public int hashCode()
   {
      if (collidableID == 0)
      {
         long hash = rigidBody == null ? 0L : rigidBody.hashCode();
         hash = EuclidHashCodeTools.combineHashCode(hash, shape.hashCode());
         hash = EuclidHashCodeTools.combineHashCode(hash, collisionMask);
         hash = EuclidHashCodeTools.combineHashCode(hash, collisionGroup);
         collidableID = EuclidHashCodeTools.toIntHashCode(hash);
      }
      return collidableID;
   }

   @Override
   public String toString()
   {
      String ret = rigidBody != null ? rigidBody.getName() : "static";
      ret += ", shape " + shape.getClass().getSimpleName();
      if (rigidBody != null)
         ret += ", mass " + rigidBody.getInertia().getMass();
      return ret;
   }
}
