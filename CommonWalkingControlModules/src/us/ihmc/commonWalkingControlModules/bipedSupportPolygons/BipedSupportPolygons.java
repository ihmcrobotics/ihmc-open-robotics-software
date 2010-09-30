package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2dAndConnectingEdges;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;


/**
 * <p>Title: BipedSupportPolygons </p>
 *
 * <p>Description: Computes and holds on to information about the biped's support polygons that is a function of state only, and not of a particular controller.
 * </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class BipedSupportPolygons
{
   // Reference frames:
   private final ReferenceFrame midFeetZUp, bodyZUp;
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;

   // Polygons:
   private final SideDependentList<FrameConvexPolygon2d> footPolygonsInAnkleZUp = new SideDependentList<FrameConvexPolygon2d>();
   private final SideDependentList<FrameConvexPolygon2d> footPolygonsInBodyZUp = new SideDependentList<FrameConvexPolygon2d>();
   private final SideDependentList<FrameConvexPolygon2d> footPolygonsInMidFeetZUp = new SideDependentList<FrameConvexPolygon2d>();
   private FrameConvexPolygon2d supportPolygonInMidFeetZUp;

   // Connecting edges between the two foot polygons to create the support polygon when in double support; null in single support
   private FrameLineSegment2d connectingEdge1, connectingEdge2;

   // 'Sweet spots', the spots inside each of the footPolygons where capture point placement leads to really good balance. Typically the middle of the foot or so:
   private final SideDependentList<FramePoint2d> sweetSpots = new SideDependentList<FramePoint2d>();

   // Line segment from one sweet spot to the other:
   private FrameLineSegment2d footToFootLineSegmentInMidFeetZUp;

   public BipedSupportPolygons(SideDependentList<ReferenceFrame> ankleZUpFrames, ReferenceFrame midFeetZUpFrame, ReferenceFrame bodyZUpFrame)
   {
	   this.ankleZUpFrames = ankleZUpFrames;
	   this.midFeetZUp = midFeetZUpFrame;
	   this.bodyZUp = bodyZUpFrame;
   }

   public FrameConvexPolygon2d getSupportPolygonInMidFeetZUp()
   {
      return supportPolygonInMidFeetZUp;
   }

   public FrameLineSegment2d getConnectingEdge1()
   {
      return connectingEdge1;
   }

   public FrameLineSegment2d getConnectingEdge2()
   {
      return connectingEdge2;
   }

   public FrameConvexPolygon2d getFootPolygonInAnkleZUp(RobotSide robotSide)
   {
      return footPolygonsInAnkleZUp.get(robotSide);
   }

   public FrameConvexPolygon2d getFootPolygonInBodyZUp(RobotSide robotSide)
   {
      return footPolygonsInBodyZUp.get(robotSide);
   }

   public FrameConvexPolygon2d getFootPolygonInMidFeetZUp(RobotSide robotSide)
   {
      return footPolygonsInMidFeetZUp.get(robotSide);
   }

   public FrameLineSegment2d getFootToFootLineSegmentInMidFeetZUp()
   {
      return footToFootLineSegmentInMidFeetZUp;
   }

   public FramePoint2d getSweetSpotCopy(RobotSide robotSide)
   {
      return new FramePoint2d(sweetSpots.get(robotSide));
   }

   public void update(BipedFootInterface leftFoot, BipedFootInterface rightFoot)
   {
      // Extract the foot polygons in use from the leftFoot and the rightFoot

      SideDependentList<BipedFootInterface> feet = new SideDependentList<BipedFootInterface>(leftFoot, rightFoot);

      for (RobotSide robotSide : RobotSide.values())
      {
         BipedFootInterface foot = feet.get(robotSide);
         FrameConvexPolygon2d footPolygonInFootFrame = foot.getFootPolygonInUse();
         FrameConvexPolygon2d footPolygonInAnkleZUp = footPolygonInFootFrame.changeFrameAndProjectToXYPlaneCopy(ankleZUpFrames.get(robotSide));
         FrameConvexPolygon2d footPolygonsInBodyZUp = footPolygonInAnkleZUp.changeFrameCopy(bodyZUp);
         FrameConvexPolygon2d footPolygonsInMidFeetZUp = footPolygonInAnkleZUp.changeFrameCopy(midFeetZUp);


         this.footPolygonsInAnkleZUp.set(robotSide, footPolygonInAnkleZUp);
         this.footPolygonsInBodyZUp.set(robotSide, footPolygonsInBodyZUp);
         this.footPolygonsInMidFeetZUp.set(robotSide, footPolygonsInMidFeetZUp);
         this.sweetSpots.set(robotSide, footPolygonsInAnkleZUp.get(robotSide).getCentroidCopy()); // Sweet spots are the centroids of the foot polygons.

      }

      // Get the support polygon. If in double support, it is the combined polygon.
      // FIXME: Assumes the individual feet polygons are disjoint for faster computation. Will crash if the feet overlap.
      // If in single support, then the support polygon is just the foot polygon of the supporting foot.
      boolean inDoubleSupport = rightFoot.isSupportingFoot() && leftFoot.isSupportingFoot();
      if (inDoubleSupport)
      {
         FrameConvexPolygon2dAndConnectingEdges supportPolygonAndEdgesInMidFeetZUp =
            FrameConvexPolygon2d.combineDisjointPolygons(footPolygonsInMidFeetZUp.get(RobotSide.LEFT), footPolygonsInMidFeetZUp.get(RobotSide.RIGHT));

         // If feet are overlapping, then supportPolygonAndEdgesInMidFeetZUp = null...
         supportPolygonInMidFeetZUp = supportPolygonAndEdgesInMidFeetZUp.getFrameConvexPolygon2d();

         connectingEdge1 = supportPolygonAndEdgesInMidFeetZUp.getConnectingEdge1();
         connectingEdge2 = supportPolygonAndEdgesInMidFeetZUp.getConnectingEdge2();
      }
      else
      {
         boolean neitherFootIsSupportingFoot = !leftFoot.isSupportingFoot() &&!rightFoot.isSupportingFoot();
         if (neitherFootIsSupportingFoot)
         {
            throw new RuntimeException("neither foot is a supporting foot!");
         }

         RobotSide supportSide = leftFoot.isSupportingFoot() ? RobotSide.LEFT : RobotSide.RIGHT;
         supportPolygonInMidFeetZUp = footPolygonsInMidFeetZUp.get(supportSide);


         connectingEdge1 = null;
         connectingEdge2 = null;
      }

      this.footToFootLineSegmentInMidFeetZUp = new FrameLineSegment2d(sweetSpots.get(RobotSide.LEFT).changeFrameAndProjectToXYPlaneCopy(midFeetZUp),
            sweetSpots.get(RobotSide.RIGHT).changeFrameAndProjectToXYPlaneCopy(midFeetZUp));
   }

   public String toString()
   {
      String ret = "";

      ret = ret + "supportPolygonInMidFeetZUp = " + supportPolygonInMidFeetZUp;


      return ret;
   }
}

