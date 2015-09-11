package us.ihmc.commonWalkingControlModules.captureRegion;

import java.awt.Color;
import java.util.ArrayList;

import us.ihmc.simulationconstructionset.util.ground.steppingStones.SteppingStones;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.ConvexPolygon2dIntersectionSetCalculator;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactPolygon;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class SteppingStonesCaptureRegionIntersectionCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("IntersectionCalculator");

   private final ArrayList<YoFrameConvexPolygon2d> intersectionPolygonsViz;

   @SuppressWarnings("unused")
   private final SteppingStones steppingStones;
   private final ConvexPolygon2dIntersectionSetCalculator convexPolygon2dIntersectionSetCalculator;

   private boolean VISUALIZE = true;

   public SteppingStonesCaptureRegionIntersectionCalculator(SteppingStones steppingStones, YoVariableRegistry yoVariableRegistry,
           YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.steppingStones = steppingStones;
      ArrayList<ConvexPolygon2d> convexPolygons = steppingStones.getShrunkenConvexPolygons();

//    ArrayList<ConvexPolygon2d> convexPolygons = steppingStones.getConvexPolygons();
      convexPolygon2dIntersectionSetCalculator = new ConvexPolygon2dIntersectionSetCalculator(convexPolygons);

      if (yoGraphicsListRegistry == null)
         VISUALIZE = false;

      if (VISUALIZE)
      {
         int numberMaxPolygonViz = 30;    // 10;
         int maxNumberVerticesPerPolygon = 20;

         intersectionPolygonsViz = new ArrayList<YoFrameConvexPolygon2d>(numberMaxPolygonViz);

//       Color[] colorsToUse = new Color[]{Color.RED, Color.WHITE, Color.BLUE, Color.GREEN, Color.BLACK};

         ArtifactList artifactList = new ArtifactList("SteppingStones");
         for (int i = 0; i < numberMaxPolygonViz; i++)
         {
            YoFrameConvexPolygon2d polygonViz = new YoFrameConvexPolygon2d("intersectionViz_" + i, "", ReferenceFrame.getWorldFrame(),
                                                   maxNumberVerticesPerPolygon, registry);
            intersectionPolygonsViz.add(polygonViz);

            YoArtifactPolygon dynamicGraphicYoPolygonArtifact = new YoArtifactPolygon("Intersection_" + i, polygonViz, Color.BLACK, true);
            artifactList.add(dynamicGraphicYoPolygonArtifact);

//          YoboticsBipedPlotter.registerDynamicGraphicPolygon("Intersection_" + i, colorsToUse[i % colorsToUse.length], polygonViz, true);
//            YoboticsBipedPlotter.registerDynamicGraphicPolygon("Intersection_" + i, Color.BLACK, polygonViz, true);
         }

         yoGraphicsListRegistry.registerArtifactList(artifactList);

      }
      else
      {
         intersectionPolygonsViz = null;
      }

      if (yoVariableRegistry != null)
      {
         yoVariableRegistry.addChild(registry);
      }
   }

   public ArrayList<ConvexPolygon2d> findIntersectionsBetweenSteppingStonesAndCaptureRegion(FrameConvexPolygon2d captureRegion)
   {
      captureRegion.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

//    captureRegion = captureRegion.changeFrameCopy(YoboticsBipedReferenceFrames.getWorldFrame());

      ConvexPolygon2d captureRegionPolygon = captureRegion.getConvexPolygon2dCopy();
      ArrayList<ConvexPolygon2d> intersectingPolygonList = convexPolygon2dIntersectionSetCalculator.findIntersectionPolygonList(captureRegionPolygon);

//    ArrayList<ConvexPolygon2d> intersectingPolygonList = convexPolygon2dIntersectionSetCalculator.findTentativeListOfPolygonsIntersectingTargetPolygon(captureRegionPolygon);


      if (VISUALIZE)
      {
         int numberIntersecting = 0;
         if (intersectingPolygonList != null)
            numberIntersecting = intersectingPolygonList.size();

         for (int i = 0; i < intersectionPolygonsViz.size(); i++)
         {
            if (i < numberIntersecting)
            {
               intersectionPolygonsViz.get(i).setConvexPolygon2d(intersectingPolygonList.get(i));

               // intersectionPolygonsViz.get(i).setConvexPolygon2d(captureRegionPolygon);
            }
            else
            {
               intersectionPolygonsViz.get(i).setConvexPolygon2d(null);

               // intersectionPolygonsViz.get(i).setConvexPolygon2d(captureRegionPolygon);
            }
         }
      }

      return intersectingPolygonList;
   }

// public Footstep findFootstepInsideSteppingStoneAndCaptureRegionIntersection(Footstep originalFootstep, FrameConvexPolygon2d captureRegion)
// {
//    captureRegion = captureRegion.changeFrameCopy(YoboticsBipedReferenceFrames.getWorldFrame());
//    FramePoint originalPosition = originalFootstep.footstepPosition;
//
//    originalPosition.checkReferenceFrameMatch(captureRegion.getReferenceFrame());
//
//    Point2d originalPosition2d = new Point2d(originalPosition.getX(), originalPosition.getY());
//    ConvexPolygon2d captureRegionPolygon = captureRegion.getConvexPolygon2dCopy();
//
//    // If the point is already inside the Capture Region and inside the stepping stones, then just return the original point.
//    if (captureRegionPolygon.isPointInside(originalPosition2d))
//    {
//       if (convexPolygon2dIntersectionSetCalculator.isPointInside(originalPosition2d))
//          return originalFootstep;
//    }
//
//    Point2d nearestPoint = convexPolygon2dIntersectionSetCalculator.getNearestPoint(originalPosition2d);
//    FramePoint nearestFramePoint = new FramePoint(YoboticsBipedReferenceFrames.getWorldFrame(), nearestPoint.x, nearestPoint.y, originalFootstep.footstepPosition.getZ());
//
//    return new Footstep(originalFootstep.footstepSide, nearestFramePoint, originalFootstep.footstepYaw);
//
// }
}
