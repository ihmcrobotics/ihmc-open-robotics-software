package us.ihmc.simulationconstructionset.physics;

import us.ihmc.simulationconstructionset.Link;
import us.ihmc.robotics.geometry.RigidBodyTransform;

/**
 * Factory for creating collision shapes
 *
 * @author Peter Abeles
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
   public void setMargin(double margin);

   /**
    * Creates a box shape.  The box will be centered around the origin (0,0,0) with vertexes spaced at the specified distances from the origin.
    *
    * @param radiusX Radius of box along x-axis
    * @param radiusY Radius of box along y-axis
    * @param radiusZ Radius of box along z-axis
    * @return Description of the shape.
    */
   public CollisionShapeDescription createBox(double radiusX, double radiusY, double radiusZ);

   // not fully supported yet
   public CollisionShapeDescription createCylinder(double radius, double height);

   // not fully supported yet
   public CollisionShapeDescription createSphere(double radius);


   /**
    * Creates a box shape.
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
   public CollisionShape addShape(Link link, RigidBodyTransform shapeToLink, CollisionShapeDescription description, boolean isGround, int collisionGroup, int collisionMask);

   /**
    * Adds a 3-DOF force sensor at the specified location
    */
   public ScsForceSensor addForceSensor(String name, CollisionShape shape, RigidBodyTransform sensorToShape);
}
