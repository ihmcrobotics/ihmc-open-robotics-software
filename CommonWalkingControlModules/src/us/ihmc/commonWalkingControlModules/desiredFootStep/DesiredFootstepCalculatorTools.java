package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class DesiredFootstepCalculatorTools
{
   public static double computeMinZPointWithRespectToAnkleInWorldFrame(RotationMatrix footToWorldRotation, ContactablePlaneBody contactableBody)
   {
      List<FramePoint> footPoints = contactableBody.getContactPointsCopy();
      double minZ = Double.POSITIVE_INFINITY;
      FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      Vector3D tempVector = new Vector3D();
      for (FramePoint footPoint : footPoints)
      {
         tempFramePoint.setIncludingFrame(footPoint);
         tempFramePoint.changeFrame(contactableBody.getFrameAfterParentJoint());
         tempVector.set(tempFramePoint.getPoint());
         footToWorldRotation.transform(tempVector);
         if (tempVector.getZ() < minZ)
            minZ = tempVector.getZ();
      }

      return minZ;
   }

   public static double computeMinZPointWithRespectToSoleInWorldFrame(RotationMatrix footToWorldRotation, ContactablePlaneBody contactableBody)
   {
      List<FramePoint> footPoints = contactableBody.getContactPointsCopy();
      double minZ = Double.POSITIVE_INFINITY;
      FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      Vector3D tempVector = new Vector3D();
      for (FramePoint footPoint : footPoints)
      {
         tempFramePoint.setIncludingFrame(footPoint);
         tempFramePoint.changeFrame(contactableBody.getSoleFrame());
         tempVector.set(tempFramePoint.getPoint());
         footToWorldRotation.transform(tempVector);
         if (tempVector.getZ() < minZ)
            minZ = tempVector.getZ();
      }

      return minZ;
   }

   public static FramePoint computeMinZWithRespectToAnkleInWorldFramePoint(RotationMatrix footToWorldRotation, ContactablePlaneBody contactableBody)
   {
      List<FramePoint> footPoints = contactableBody.getContactPointsCopy();
      double minZ = Double.POSITIVE_INFINITY;
      FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      FramePoint minZPoint = new FramePoint(ReferenceFrame.getWorldFrame());
      Vector3D tempVector = new Vector3D();
      for (FramePoint footPoint : footPoints)
      {
         tempFramePoint.setIncludingFrame(footPoint);
         tempFramePoint.changeFrame(contactableBody.getFrameAfterParentJoint());
         tempVector.set(tempFramePoint.getPoint());
         footToWorldRotation.transform(tempVector);

         if (tempVector.getZ() < minZ)
         {
            minZPoint.setIncludingFrame(tempFramePoint);
            minZ = tempVector.getZ();
         }
      }

      return minZPoint;
   }

   public static double computeMaxXWithRespectToAnkleInFrame(RotationMatrix footToWorldRotation, ContactablePlaneBody contactableBody, ReferenceFrame frame)
   {
      RigidBodyTransform worldToDesiredHeadingFrame = frame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      List<FramePoint> footPoints = contactableBody.getContactPointsCopy();
      double maxX = Double.NEGATIVE_INFINITY;
      FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      Vector3D tempVector = new Vector3D();
      for (FramePoint footPoint : footPoints)
      {
         tempFramePoint.setIncludingFrame(footPoint);
         tempFramePoint.changeFrame(contactableBody.getFrameAfterParentJoint());
         tempVector.set(tempFramePoint.getPoint()); // foot point w.r.t. ankle in foot frame
         footToWorldRotation.transform(tempVector); // foot point w.r.t. ankle in world frame
         worldToDesiredHeadingFrame.transform(tempVector); // foot point w.r.t. ankle in desired heading frame
         if (tempVector.getX() > maxX)
            maxX = tempVector.getX();
      }

      return maxX;
   }

   public static FramePoint computeMinZPointInFrame(RigidBodyTransform footToWorldTransform, ContactablePlaneBody contactableBody, ReferenceFrame frame)
   {
      List<FramePoint> footPoints = contactableBody.getContactPointsCopy();

      ReferenceFrame bodyFrame = contactableBody.getFrameAfterParentJoint();

      return computeMinZPointInFrame(footToWorldTransform, footPoints, bodyFrame, frame);
   }

   public static FramePoint computeMinZPointInFrame(RigidBodyTransform footToWorldTransform, List<FramePoint> footPoints, ReferenceFrame bodyFrame,
         ReferenceFrame frame)
   {
      FramePoint minFramePoint = new FramePoint(frame);
      minFramePoint.setZ(Double.POSITIVE_INFINITY);
      FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      boolean pointFound = false;
      for (FramePoint footPoint : footPoints)
      {
         tempFramePoint.setIncludingFrame(footPoint);
         tempFramePoint.changeFrame(bodyFrame);
         tempFramePoint.changeFrameUsingTransform(ReferenceFrame.getWorldFrame(), footToWorldTransform);
         tempFramePoint.changeFrame(frame);

         if (tempFramePoint.getZ() < minFramePoint.getZ())
         {
            minFramePoint.set(tempFramePoint);
            pointFound = true;
         }
      }

      if (!pointFound)
         throw new RuntimeException();

      return minFramePoint;
   }

   public static FramePoint computeMaxXPointInFrame(RigidBodyTransform footToWorldTransform, ContactablePlaneBody contactableBody, ReferenceFrame frame)
   {
      List<FramePoint> footPoints = contactableBody.getContactPointsCopy();
      FramePoint maxFramePoint = new FramePoint(frame);
      maxFramePoint.setX(Double.NEGATIVE_INFINITY);
      FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      boolean pointFound = false;
      for (FramePoint footPoint : footPoints)
      {
         tempFramePoint.setIncludingFrame(footPoint);
         tempFramePoint.changeFrame(contactableBody.getFrameAfterParentJoint());
         tempFramePoint.changeFrameUsingTransform(ReferenceFrame.getWorldFrame(), footToWorldTransform);
         tempFramePoint.changeFrame(frame);

         if (tempFramePoint.getX() > maxFramePoint.getX())
         {
            maxFramePoint.set(tempFramePoint);
            pointFound = true;
         }
      }

      if (!pointFound)
         throw new RuntimeException();

      return maxFramePoint;
   }

   public static List<FramePoint> computeMaximumPointsInDirection(List<FramePoint> framePoints, FrameVector searchDirection, int nPoints)
   {
      if (framePoints.size() < nPoints)
         throw new RuntimeException("Not enough points");
      searchDirection.changeFrame(framePoints.get(0).getReferenceFrame());
      List<FramePoint> ret = new ArrayList<FramePoint>(framePoints);
      Collections.sort(ret, new SearchDirectionFramePointComparator(searchDirection));

      while (ret.size() > nPoints)
      {
         ret.remove(0);
      }

      return ret;
   }

   public static List<Point2D> computeMaximumPointsInDirection(List<Point2D> framePoints, Vector2D searchDirection, int nPoints)
   {
      if (framePoints.size() < nPoints)
         throw new RuntimeException("Not enough points");
      List<Point2D> ret = new ArrayList<Point2D>(framePoints);
      Collections.sort(ret, new SearchDirectionPoint2dComparator(searchDirection));

      while (ret.size() > nPoints)
      {
         ret.remove(0);
      }

      return ret;
   }

   public static int[] findMaximumPointIndexesInDirection(List<FramePoint> framePoints, FrameVector searchDirection, int nPoints)
   {
      List<FramePoint> maximumPoints = computeMaximumPointsInDirection(framePoints, searchDirection, nPoints);

      int[] indexes = new int[nPoints];

      for (int i = 0; i < nPoints; i++)
      {
         indexes[i] = framePoints.indexOf(maximumPoints.get(i));
      }

      return indexes;
   }

   private static class SearchDirectionFramePointComparator implements Comparator<FramePoint>
   {
      private final FrameVector searchDirection;
      private final FrameVector differenceVector = new FrameVector(ReferenceFrame.getWorldFrame());

      public SearchDirectionFramePointComparator(FrameVector searchDirection)
      {
         this.searchDirection = searchDirection;
      }

      public int compare(FramePoint o1, FramePoint o2)
      {
         differenceVector.setIncludingFrame(o1);
         differenceVector.sub(o2);
         double dotProduct = searchDirection.dot(differenceVector);

         return Double.compare(dotProduct, 0.0);
      }
   }

   private static class SearchDirectionPoint2dComparator implements Comparator<Point2D>
   {
      private final Vector2D searchDirection;
      private final Vector2D differenceVector = new Vector2D();

      public SearchDirectionPoint2dComparator(Vector2D searchDirection)
      {
         this.searchDirection = searchDirection;
      }

      public int compare(Point2D o1, Point2D o2)
      {
         differenceVector.set(o1);
         differenceVector.sub(o2);
         double dotProduct = searchDirection.dot(differenceVector);

         return Double.compare(dotProduct, 0.0);
      }
   }
}
