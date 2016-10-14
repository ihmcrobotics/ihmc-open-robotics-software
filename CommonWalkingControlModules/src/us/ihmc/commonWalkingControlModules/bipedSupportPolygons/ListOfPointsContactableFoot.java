package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ListOfPointsContactableFoot extends ListOfPointsContactablePlaneBody implements ContactableFoot
{
   private final FramePoint2d toeOffContactPoint = new FramePoint2d();

   public ListOfPointsContactableFoot(RigidBody rigidBody, ReferenceFrame soleFrame, List<Point2d> contactPointsInSoleFrame, Point2d toeOffContactPoint)
   {
      super(rigidBody, soleFrame, contactPointsInSoleFrame);

      this.toeOffContactPoint.setIncludingFrame(soleFrame, toeOffContactPoint);
   }

   @Override
   public void getToeOffContactPoint(FramePoint2d contactPointToPack)
   {
      contactPointToPack.setIncludingFrame(toeOffContactPoint);
   }
}
