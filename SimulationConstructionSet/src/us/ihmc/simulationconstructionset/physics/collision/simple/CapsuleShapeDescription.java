package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class CapsuleShapeDescription<T extends CapsuleShapeDescription<T>> implements CollisionShapeDescription<T>
{
   private double radius;
   private LineSegment3D lineSegmentInShapeFrame = new LineSegment3D();
   private LineSegment3D lineSegment = new LineSegment3D();

   private final BoundingBox3D boundingBox = new BoundingBox3D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY,
                                                               Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private boolean boundingBoxNeedsUpdating = true;
   private RigidBodyTransform transform = new RigidBodyTransform();
   private RigidBodyTransform tempTransform = new RigidBodyTransform();

   public CapsuleShapeDescription(double radius, LineSegment3D lineSegment)
   {
      this.radius = radius;
      this.lineSegmentInShapeFrame.set(lineSegment);
      this.lineSegment.set(lineSegment);
      boundingBoxNeedsUpdating = true;
   }

   public CapsuleShapeDescription(double radius, double height)
   {
      if (height < 2.0 * radius)
         throw new RuntimeException("Capsule height must be at least 2.0 * radius!");
      this.radius = radius;
      this.lineSegmentInShapeFrame.set(0.0, 0.0, -height / 2.0 + radius, 0.0, 0.0, height / 2.0 - radius);
      this.lineSegment.set(0.0, 0.0, -height / 2.0 + radius, 0.0, 0.0, height / 2.0 - radius);
      boundingBoxNeedsUpdating = true;
   }

   @Override
   public CapsuleShapeDescription<T> copy()
   {
      CapsuleShapeDescription<T> copy = new CapsuleShapeDescription<T>(radius, lineSegment);
      copy.setTransform(this.transform);
      return copy;
   }

   public double getRadius()
   {
      return radius;
   }

   public void getTransform(RigidBodyTransform transformToPack)
   {
      transformToPack.set(transform);
   }

   public void setTransform(RigidBodyTransform transform)
   {
      this.transform.set(transform);
      this.lineSegment.applyTransform(transform);
   }

   public void getLineSegment(LineSegment3D lineSegmentToPack)
   {
      lineSegmentToPack.set(lineSegment);
   }

   public void getLineSegmentInShapeFrame(LineSegment3D lineSegmentInShapeFrameToPack)
   {
      lineSegmentInShapeFrameToPack.set(lineSegmentInShapeFrame);
   }

   @Override
   public void setFrom(T capsuleShapeDescription)
   {
      this.radius = capsuleShapeDescription.getRadius();
      capsuleShapeDescription.getLineSegment(this.lineSegment);
      capsuleShapeDescription.getLineSegmentInShapeFrame(this.lineSegmentInShapeFrame);
      boundingBoxNeedsUpdating = true;

      capsuleShapeDescription.getTransform(this.transform);
   }

   @Override
   public void applyTransform(RigidBodyTransform transformToWorld)
   {
      transform.preMultiply(transformToWorld);
      lineSegment.applyTransform(transformToWorld);
      boundingBoxNeedsUpdating = true;
   }

   @Override
   public void getBoundingBox(BoundingBox3D boundingBoxToPack)
   {
      if (boundingBoxNeedsUpdating)
      {
         updateBoundingBox();
         boundingBoxNeedsUpdating = false;
      }
      boundingBoxToPack.set(boundingBox);
   }

   private void updateBoundingBox()
   {
      Point3D firstEndpoint = lineSegment.getFirstEndpoint();
      Point3D secondEndpoint = lineSegment.getSecondEndpoint();

      double xMin, yMin, zMin, xMax, yMax, zMax;

      if (firstEndpoint.getX() < secondEndpoint.getX())
      {
         xMin = firstEndpoint.getX();
         xMax = secondEndpoint.getX();
      }
      else
      {
         xMin = secondEndpoint.getX();
         xMax = firstEndpoint.getX();
      }

      if (firstEndpoint.getY() < secondEndpoint.getY())
      {
         yMin = firstEndpoint.getY();
         yMax = secondEndpoint.getY();
      }
      else
      {
         yMin = secondEndpoint.getY();
         yMax = firstEndpoint.getY();
      }

      if (firstEndpoint.getZ() < secondEndpoint.getZ())
      {
         zMin = firstEndpoint.getZ();
         zMax = secondEndpoint.getZ();
      }
      else
      {
         zMin = secondEndpoint.getZ();
         zMax = firstEndpoint.getZ();
      }

      boundingBox.set(xMin - radius, yMin - radius, zMin - radius, xMax + radius, yMax + radius, zMax + radius);
   }

   @Override
   public boolean isPointInside(Point3D pointInWorld)
   {
      return (lineSegment.distanceSquared(pointInWorld) <= radius * radius);
   }

   private final Point3D tempPointForRollingWorldFrame = new Point3D();
   private final Point3D tempPointForRollingShapeFrame = new Point3D();
   private final Vector3D tempVectorForRollingWorldFrame = new Vector3D();
   private final Vector3D tempVectorForRollingShapeFrame = new Vector3D();
   private final Vector3D tempVectorTwoForRollingShapeFrame = new Vector3D();
//   private final LineSegment3d tempLineSegmentForRollingWorldFrame = new LineSegment3d();

   @Override
   public boolean rollContactIfRolling(Vector3D surfaceNormal, Point3D pointToRoll)
   {
      tempTransform.set(transform);
      tempTransform.invert();
//      System.out.println("tempTransform = " + tempTransform);

      tempPointForRollingWorldFrame.set(pointToRoll);
      tempVectorForRollingWorldFrame.set(surfaceNormal);
//      tempLineSegmentForRollingWorldFrame.set(lineSegment);

      tempTransform.transform(tempPointForRollingWorldFrame);
      tempTransform.transform(tempVectorForRollingWorldFrame);
//      tempLineSegmentForRollingWorldFrame.applyTransform(tempTransform);

      boolean isRolling = rollContactIfRollingShapeFrame(tempVectorForRollingWorldFrame, tempPointForRollingWorldFrame);
      transform.transform(tempPointForRollingWorldFrame);
      pointToRoll.set(tempPointForRollingWorldFrame);

      return isRolling;
   }
   
   public boolean rollContactIfRollingShapeFrame(Vector3D surfaceNormalInShapeFrame, Point3D pointToRollInShapeFrame)
   {
      //TODO: This assumes shape frame has the capsule being along the z axis and middle of capsule at 0, 0...
      //TODO: If we don't want to assume that, we should just do everything in world frame then and use the vectors and dot products, etc.
      
//      System.out.println("\nCapsuleRolling:");
//      System.out.println("surfaceNormalInShapeFrame = " + surfaceNormalInShapeFrame);
//      System.out.println("pointToRollInShapeFrame = " + pointToRollInShapeFrame);
//      System.out.println("lineSegmentInShapeFrame = " + lineSegmentInShapeFrame);

      tempVectorForRollingShapeFrame.set(surfaceNormalInShapeFrame);

      double height = 2.0 * (lineSegmentInShapeFrame.getSecondEndpoint().getZ() + radius);
      
//      System.out.println("pointToRollInShapeFrame.getZ() = " + pointToRollInShapeFrame.getZ());
//      System.out.println("height = " + height);
//      System.out.println("radius = " + radius);
//      System.out.println("-height/2.0 + radius = " + (-height/2.0 + radius));
//      System.out.println("height/2.0 - radius = " + (height/2.0 - radius));
      if ((pointToRollInShapeFrame.getZ() > -height/2.0 + radius) && (pointToRollInShapeFrame.getZ() < height/2.0 - radius))
      {
         // Not on the caps. So don't roll along the flat side, only along the curved side...
         tempVectorForRollingShapeFrame.setZ(0.0);
//         System.out.println("Not on the caps... surfaceNormalInShapeFrame = " + tempVectorForRollingShapeFrame);
      }

      lineSegmentInShapeFrame.orthogonalProjection(pointToRollInShapeFrame, tempPointForRollingShapeFrame);
//      System.out.println("after line projection pointToRollInShapeFrame = " + tempPointForRollingShapeFrame);
      double currentRadiusOfPenetration = pointToRollInShapeFrame.distance(tempPointForRollingShapeFrame);

      tempVectorForRollingShapeFrame.normalize();
      tempVectorForRollingShapeFrame.scale(currentRadiusOfPenetration);
//      tempVectorForRolling.scale(radius);

      tempPointForRollingShapeFrame.add(tempVectorForRollingShapeFrame);
      pointToRollInShapeFrame.set(tempPointForRollingShapeFrame);

//      System.out.println("pointToRollInShapeFrame = " + pointToRollInShapeFrame);

      //TODO: Not necessarily true all the time....

      return true;
   }

}
