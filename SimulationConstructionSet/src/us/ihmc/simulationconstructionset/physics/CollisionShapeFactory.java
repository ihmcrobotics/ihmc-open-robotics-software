package us.ihmc.simulationconstructionset.physics;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.simulationconstructionset.Link;

/**
 * Factory for creating collision shapes
 */
public interface CollisionShapeFactory
{
   /**
    * The default margin which should be used by all implementations.
    */
   public static final double DEFAULT_MARGIN = 0.04;

   /**
    * Specifies the collision margin for the next shape which is to be added.  For small objects this needs to be set to a smaller value
    *
    * @param margin Collision margin
    */
   public abstract void setMargin(double margin);

   /**
    * Creates a box shape.  The box will be centered around the origin (0,0,0) with vertexes spaced at the specified distances from the origin.
    *
    * @param radiusX Radius of box along x-axis
    * @param radiusY Radius of box along y-axis
    * @param radiusZ Radius of box along z-axis
    * @return Description of the shape.
    */
   public abstract CollisionShapeDescription<?> createBox(double radiusX, double radiusY, double radiusZ);

   public abstract CollisionShapeDescription<?> createCylinder(double radius, double height);

   public abstract CollisionShapeDescription<?> createSphere(double radius);

   public abstract CollisionShapeDescription<?> createCapsule(double radius, double objectHeight);

   /**
    * Adds a shape.
    *
    * Also  which shapes a shape can collide against.  By default a shape will collide with all other shapes.
    * A shape will collide with another shape if (shapeGroup & collisionMask) != 0.
    *
    * @param link Link which this shape is attached to.
    * @param shapeToLink Transform from the shape to the Link's local coordinate system.  If null the transform will be set to identity.
    * @param description Description of the collision shape
    * @param collisionGroup Bit field specifying which collision groups the shape belongs to.  Set to 0xFFFFFFFF to belong to all groups
    * @param collisionMask Bit field specifying which groups it can collide against.  Set to 0xFFFFFFFF to collide against all groups
    * @return The resulting collision shape.  Already attached to the provided link.
    */
   public abstract CollisionShape addShape(Link link, RigidBodyTransform shapeToLink, CollisionShapeDescription<?> description, boolean isGround, int collisionGroup, int collisionMask);

   public abstract CollisionShape addShape(CollisionShapeDescription<?> description);

   public abstract void addCollisionMeshDescription(Link link, CollisionMeshDescription collisionMeshDescription);
 }
