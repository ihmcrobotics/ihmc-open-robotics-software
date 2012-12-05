package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactableBody;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class DesiredFootstepCalculatorTools
{
   public static double computeMinZWithRespectToAnkleInWorldFrame(Matrix3d footToWorldRotation, ContactableBody contactableBody)
   {
      List<FramePoint> footPoints = contactableBody.getContactPoints();
      double minZ = Double.POSITIVE_INFINITY;
      FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      Vector3d tempVector = new Vector3d();
      for (FramePoint footPoint : footPoints)
      {
         tempFramePoint.setAndChangeFrame(footPoint);
         tempFramePoint.changeFrame(contactableBody.getBodyFrame());
         tempVector.set(tempFramePoint.getPoint());
         footToWorldRotation.transform(tempVector);
         if (tempVector.getZ() < minZ)
            minZ = tempVector.getZ();
      }

      return minZ;
   }

   public static double computeMaxXWithRespectToAnkleInFrame(Matrix3d footToWorldRotation, ContactableBody contactableBody, ReferenceFrame frame)
   {
      Transform3D worldToDesiredHeadingFrame = frame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      List<FramePoint> footPoints = contactableBody.getContactPoints();
      double maxX = Double.NEGATIVE_INFINITY;
      FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      Vector3d tempVector = new Vector3d();
      for (FramePoint footPoint : footPoints)
      {
         tempFramePoint.setAndChangeFrame(footPoint);
         tempFramePoint.changeFrame(contactableBody.getBodyFrame());
         tempVector.set(tempFramePoint.getPoint());    // foot point w.r.t. ankle in foot frame
         footToWorldRotation.transform(tempVector);    // foot point w.r.t. ankle in world frame
         worldToDesiredHeadingFrame.transform(tempVector);    // foot point w.r.t. ankle in desired heading frame
         if (tempVector.getX() > maxX)
            maxX = tempVector.getX();
      }

      return maxX;
   }

   public static FramePoint computeMinZPointInFrame(Transform3D footToWorldTransform, ContactableBody contactableBody, ReferenceFrame frame)
   {
      List<FramePoint> footPoints = contactableBody.getContactPoints();
      FramePoint minFramePoint = new FramePoint(frame);
      minFramePoint.setZ(Double.POSITIVE_INFINITY);
      FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      boolean pointFound = false;
      for (FramePoint footPoint : footPoints)
      {
         tempFramePoint.setAndChangeFrame(footPoint);
         tempFramePoint.changeFrame(contactableBody.getBodyFrame());
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

   public static FramePoint computeMaxXPointInFrame(Transform3D footToWorldTransform, ContactableBody contactableBody, ReferenceFrame frame)
   {
      List<FramePoint> footPoints = contactableBody.getContactPoints();
      FramePoint maxFramePoint = new FramePoint(frame);
      maxFramePoint.setX(Double.NEGATIVE_INFINITY);
      FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      boolean pointFound = false;
      for (FramePoint footPoint : footPoints)
      {
         tempFramePoint.setAndChangeFrame(footPoint);
         tempFramePoint.changeFrame(contactableBody.getBodyFrame());
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

   
   public static List<FramePoint> fixTwoPointsAndCopy(List<FramePoint> contactPoints)
   {
      // FIXME: terrible, we need to make it so that ConvexPolygons can consist of fewer than 3 points
      List<FramePoint> contactPointsCopy = new ArrayList<FramePoint>(contactPoints.size() + 2);
      for (FramePoint framePoint : contactPoints)
      {
         contactPointsCopy.add(framePoint);
      }

      if (contactPointsCopy.size() == 2)
      {
         ReferenceFrame referenceFrame = contactPoints.get(0).getReferenceFrame();
         
         FrameVector pointToPoint = new FrameVector(contactPoints.get(1));
         pointToPoint.sub(contactPoints.get(0));
         
         FrameVector z = new FrameVector(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.0);
         z.changeFrame(referenceFrame);
         FrameVector cross = new FrameVector(referenceFrame);
         cross.cross(pointToPoint, z);
         cross.normalize();
         cross.scale(1e-5);
         
         for (int i = 0; i < 2; i++)
         {
            FramePoint extraPoint = new FramePoint(contactPoints.get(i));
            extraPoint.add(cross);
            contactPointsCopy.add(extraPoint);
         }

      }
      return contactPointsCopy;
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
         differenceVector.setAndChangeFrame(o1);
         differenceVector.sub(o2);
         double dotProduct = searchDirection.dot(differenceVector);

         return Double.compare(dotProduct, 0.0);
      }
   }
}
