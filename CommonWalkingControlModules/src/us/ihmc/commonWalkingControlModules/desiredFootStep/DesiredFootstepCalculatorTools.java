package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedFootInterface;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class DesiredFootstepCalculatorTools
{
   public static double computeMinZWithRespectToAnkleInWorldFrame(Matrix3d footToWorldRotation, ReferenceFrame footFrame, BipedFootInterface bipedFoot)
   {
      ArrayList<FramePoint2d> footPoints = bipedFoot.getFootPolygonInSoleFrame().getClockwiseOrderedListOfFramePoints();
      double minZ = Double.POSITIVE_INFINITY;
      FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      Vector3d tempVector = new Vector3d();
      for (FramePoint2d footPoint : footPoints)
      {
         tempFramePoint.setToZero(footPoint.getReferenceFrame());
         tempFramePoint.setXY(footPoint);
         tempFramePoint.changeFrame(footFrame);
         tempVector.set(tempFramePoint.getPoint());
         footToWorldRotation.transform(tempVector);
         if (tempVector.getZ() < minZ)
            minZ = tempVector.getZ();
      }

      return minZ;
   }

   public static double computeMaxXWithRespectToAnkleInFrame(Matrix3d footToWorldRotation, ReferenceFrame footFrame, BipedFootInterface bipedFoot, ReferenceFrame frame)
   {
      Transform3D worldToDesiredHeadingFrame = frame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      ArrayList<FramePoint2d> footPoints = bipedFoot.getFootPolygonInSoleFrame().getClockwiseOrderedListOfFramePoints();
      double maxX = Double.NEGATIVE_INFINITY;
      FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      Vector3d tempVector = new Vector3d();
      for (FramePoint2d footPoint : footPoints)
      {
         tempFramePoint.setToZero(footPoint.getReferenceFrame());
         tempFramePoint.setXY(footPoint);
         tempFramePoint.changeFrame(footFrame);
         tempVector.set(tempFramePoint.getPoint());    // foot point w.r.t. ankle in foot frame
         footToWorldRotation.transform(tempVector);    // foot point w.r.t. ankle in world frame
         worldToDesiredHeadingFrame.transform(tempVector);    // foot point w.r.t. ankle in desired heading frame
         if (tempVector.getX() > maxX)
            maxX = tempVector.getX();
      }

      return maxX;
   }
   
   public static FramePoint computeMinZPointInFrame(Transform3D footToWorldTransform, BipedFootInterface bipedFoot, ReferenceFrame frame)
   {
      ArrayList<FramePoint2d> footPoints = bipedFoot.getFootPolygonInSoleFrame().getClockwiseOrderedListOfFramePoints();
      FramePoint minFramePoint = new FramePoint(frame);
      minFramePoint.setZ(Double.POSITIVE_INFINITY);
      FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      boolean pointFound = false;
      for (FramePoint2d footPoint : footPoints)
      {
         tempFramePoint.setToZero(footPoint.getReferenceFrame());
         tempFramePoint.setXY(footPoint);
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
   
   public static FramePoint computeMaxXPointInFrame(Transform3D footToWorldTransform, BipedFootInterface bipedFoot, ReferenceFrame frame)
   {
      ArrayList<FramePoint2d> footPoints = bipedFoot.getFootPolygonInSoleFrame().getClockwiseOrderedListOfFramePoints();
      FramePoint maxFramePoint = new FramePoint(frame);
      maxFramePoint.setX(Double.NEGATIVE_INFINITY);
      FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      boolean pointFound = false;
      for (FramePoint2d footPoint : footPoints)
      {
         tempFramePoint.setToZero(footPoint.getReferenceFrame());
         tempFramePoint.setXY(footPoint);
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
}
