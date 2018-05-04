package us.ihmc.robotics.robotDescription;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.polytope.ConvexPolytope;

public class CollisionMeshDescription implements CollisionMaskHolder
{
   private final Pose3D pose = new Pose3D();
   private final ArrayList<ConvexShapeDescriptionReadOnly> convexShapeDescriptions = new ArrayList<>();
   private boolean isGround = false;
   private int collisionGroup = 0x00;
   private int collisionMask = 0x00;

   private int estimatedNumberOfContactPoints = 24;

   public void setEstimatedNumberOfContactPoints(int estimatedNumberOfContactPoints)
   {
      this.estimatedNumberOfContactPoints = estimatedNumberOfContactPoints;
   }

   public int getEstimatedNumberOfContactPoints()
   {
      return estimatedNumberOfContactPoints;
   }

   public void addConvexShape(ConvexShapeDescriptionReadOnly convexShapeDescription)
   {
      convexShapeDescriptions.add(convexShapeDescription);
   }

   public void addSphere(double radius)
   {
      SphereDescriptionReadOnly sphere = new SphereDescriptionReadOnly(radius, getRigidBodyTransformCopy());
      convexShapeDescriptions.add(sphere);
   }

   public void addCapsule(double radius, LineSegment3D capToCapLineSegment)
   {
      CapsuleDescriptionReadOnly capsule = new CapsuleDescriptionReadOnly(radius, capToCapLineSegment, getRigidBodyTransformCopy());
      convexShapeDescriptions.add(capsule);
   }

   public void addCapsule(double radius, double height, Axis longAxis)
   {
      CapsuleDescriptionReadOnly capsule = new CapsuleDescriptionReadOnly(radius, height, longAxis, getRigidBodyTransformCopy());
      convexShapeDescriptions.add(capsule);
   }

   public void addCapsule(double radius, double height)
   {
      CapsuleDescriptionReadOnly capsule = new CapsuleDescriptionReadOnly(radius, height, getRigidBodyTransformCopy());
      convexShapeDescriptions.add(capsule);
   }

   public void addCubeReferencedAtBottomMiddle(double lengthX, double widthY, double heightZ)
   {
      pose.appendTranslation(0.0, 0.0, heightZ / 2.0);
      CubeDescriptionReadOnly cube = new CubeDescriptionReadOnly(lengthX, widthY, heightZ, getRigidBodyTransformCopy());
      pose.appendTranslation(0.0, 0.0, -heightZ / 2.0);
      convexShapeDescriptions.add(cube);
   }

   public void addCubeReferencedAtCenter(double lengthX, double widthY, double heightZ)
   {
      CubeDescriptionReadOnly cube = new CubeDescriptionReadOnly(lengthX, widthY, heightZ, getRigidBodyTransformCopy());
      convexShapeDescriptions.add(cube);
   }

   public void addCylinderReferencedAtCenter(double radius, double height)
   {
      CylinderDescriptionReadOnly cylinder = new CylinderDescriptionReadOnly(radius, height, getRigidBodyTransformCopy());
      convexShapeDescriptions.add(cylinder);
   }

   public void addCylinderReferencedAtBottomMiddle(double radius, double height)
   {
      pose.appendTranslation(0.0, 0.0, height / 2.0);
      CylinderDescriptionReadOnly cylinder = new CylinderDescriptionReadOnly(radius, height, getRigidBodyTransformCopy());
      convexShapeDescriptions.add(cylinder);
      pose.appendTranslation(0.0, 0.0, -height / 2.0);
   }

   public void addConvexPolytope(ConvexPolytope polytope)
   {
      ConvexPolytopeDescriptionReadOnly polytopeReadOnly = new ConvexPolytopeDescriptionReadOnly(polytope, getRigidBodyTransformCopy());
      convexShapeDescriptions.add(polytopeReadOnly);
   }

   private RigidBodyTransform getRigidBodyTransformCopy()
   {
      return new RigidBodyTransform(pose.getOrientation(), pose.getPosition());
   }

   public void getConvexShapeDescriptions(List<ConvexShapeDescriptionReadOnly> convexShapeDescriptionsToPack)
   {
      convexShapeDescriptionsToPack.addAll(convexShapeDescriptions);
   }

   public boolean getIsGround()
   {
      return isGround;
   }

   @Override
   public int getCollisionGroup()
   {
      return collisionGroup;
   }

   @Override
   public int getCollisionMask()
   {
      return collisionMask;
   }

   public void setIsGround(boolean isGround)
   {
      this.isGround = isGround;
   }

   @Override
   public void setCollisionGroup(int collisionGroup)
   {
      this.collisionGroup = collisionGroup;
   }

   @Override
   public void setCollisionMask(int collisionMask)
   {
      this.collisionMask = collisionMask;
   }

   public void translate(double x, double y, double z)
   {
      pose.appendTranslation(x, y, z);
   }

   public void translate(Tuple3DReadOnly translationVector)
   {
      pose.appendTranslation(translationVector);
   }

   public void transform(RigidBodyTransform transform)
   {
      this.pose.set(transform);
   }

   public void identity()
   {
      pose.setToZero();
   }

   public void rotateEuler(Vector3DReadOnly eulerAngles)
   {
      pose.appendYawRotation(eulerAngles.getZ());
      pose.appendPitchRotation(eulerAngles.getY());
      pose.appendRollRotation(eulerAngles.getZ());
   }

   public void rotate(Orientation3DReadOnly rotation)
   {
      pose.appendRotation(rotation);
   }

   public void rotate(double rotationAngle, Axis axis)
   {
      switch (axis)
      {
      case X:
         pose.appendRollRotation(rotationAngle);
         break;
      case Y:
         pose.appendPitchRotation(rotationAngle);
         break;
      case Z:
         pose.appendYawRotation(rotationAngle);
         break;
      }
   }

   public void scale(double factor)
   {
      throw new RuntimeException("TODO: Implement me");
   }
}
