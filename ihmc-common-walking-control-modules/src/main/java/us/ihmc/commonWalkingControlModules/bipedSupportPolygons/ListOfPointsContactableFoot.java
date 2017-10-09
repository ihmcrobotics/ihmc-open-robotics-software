package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.List;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ListOfPointsContactableFoot extends ListOfPointsContactablePlaneBody implements ContactableFoot
{
   private final FramePoint2D toeOffContactPoint = new FramePoint2D();
   private final FrameLineSegment2d toeOffContactLine = new FrameLineSegment2d();

   public ListOfPointsContactableFoot(RigidBody rigidBody, ReferenceFrame soleFrame, List<Point2D> contactPointsInSoleFrame, Point2D toeOffContactPointInSoleFrame,
         LineSegment2D toeOffContactLineInSoleFrame)
   {
      super(rigidBody, soleFrame, contactPointsInSoleFrame);

      this.toeOffContactPoint.setIncludingFrame(soleFrame, toeOffContactPointInSoleFrame);
      this.toeOffContactLine.setIncludingFrame(soleFrame, toeOffContactLineInSoleFrame);
   }

   @Override
   public void getToeOffContactPoint(FramePoint2D contactPointToPack)
   {
      contactPointToPack.setIncludingFrame(toeOffContactPoint);
   }

   @Override
   public void getToeOffContactLine(FrameLineSegment2d contactLineToPack)
   {
      contactLineToPack.setIncludingFrame(toeOffContactLine);
   }
}
