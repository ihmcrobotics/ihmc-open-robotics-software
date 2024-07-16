package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.List;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public class ListOfPointsContactableFoot extends ListOfPointsContactablePlaneBody implements ContactableFoot
{
   private final FramePoint2D toeOffContactPoint;
   private final FrameLineSegment2D toeOffContactLine;

   public ListOfPointsContactableFoot(RigidBodyBasics rigidBody,
                                      ReferenceFrame soleFrame,
                                      List<? extends Point2DReadOnly> contactPointsInSoleFrame,
                                      Point2DReadOnly toeOffContactPointInSoleFrame,
                                      LineSegment2DReadOnly toeOffContactLineInSoleFrame)
   {
      super(rigidBody, soleFrame, contactPointsInSoleFrame);

      this.toeOffContactPoint = new FramePoint2D(soleFrame, toeOffContactPointInSoleFrame);
      this.toeOffContactLine = new FrameLineSegment2D(soleFrame, toeOffContactLineInSoleFrame);
   }

   @Override
   public void getToeOffContactPoint(FramePoint2D contactPointToPack)
   {
      contactPointToPack.setIncludingFrame(toeOffContactPoint);
   }

   @Override
   public void getToeOffContactLine(FrameLineSegment2D contactLineToPack)
   {
      contactLineToPack.setIncludingFrame(toeOffContactLine);
   }
}
