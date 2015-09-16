package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.awt.Color;
import java.util.List;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameLineSegment2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.GlobalTimer;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPolygon;

/**
 * <p>Title: OldBipedSupportPolygons </p>
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

/*
 * FIXME: not rewindable! Generate garbage. Use BipedSupportPolygons instead.
 */
public class OldBipedSupportPolygons
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static boolean VISUALIZE = true;

   private final YoVariableRegistry registry = new YoVariableRegistry("BipedSupportPolygons");

   // Reference frames:
   private final ReferenceFrame midFeetZUp;
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;

   // Polygons:
   private final SideDependentList<FrameConvexPolygon2d> footPolygonsInAnkleZUp = new SideDependentList<FrameConvexPolygon2d>();
   private final SideDependentList<FrameConvexPolygon2d> footPolygonsInMidFeetZUp = new SideDependentList<FrameConvexPolygon2d>();
   private final FrameConvexPolygon2d supportPolygonInMidFeetZUp = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d supportPolygonInWorld = new FrameConvexPolygon2d();
   private final YoFrameConvexPolygon2d supportPolygonViz;
   private final YoFrameLineSegment2d footToFootSegmentViz;

   // Connecting edges between the two foot polygons to create the support polygon when in double support; null in single support
   private final FrameLineSegment2d connectingEdge1, connectingEdge2;

   // 'Sweet spots', the spots inside each of the footPolygons where capture point placement leads to really good balance. Typically the middle of the foot or so:
   private final SideDependentList<FramePoint2d> sweetSpotsInAnkleZUp = new SideDependentList<FramePoint2d>();
   private final SideDependentList<FramePoint2d> sweetSpotsInMidFeetZUp = new SideDependentList<FramePoint2d>();

   // Line segment from one sweet spot to the other:
   private final FrameLineSegment2d footToFootLineSegmentInMidFeetZUp;
   private final FrameLineSegment2d footToFootLineSegmentInWorld;

   // In order to deal with intersecting polygons, it is much harder to calculate the connecting edges
   // So let's not use the connecting edges unles we need
   private boolean useConnectingEdges;

   private final GlobalTimer timer = new GlobalTimer(getClass().getSimpleName() + "Timer", registry);

   public OldBipedSupportPolygons(SideDependentList<ReferenceFrame> ankleZUpFrames, ReferenceFrame midFeetZUpFrame, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry, boolean useConnectingEdges)
   {
      this.ankleZUpFrames = ankleZUpFrames;
      this.midFeetZUp = midFeetZUpFrame;
      this.useConnectingEdges = useConnectingEdges;

      supportPolygonViz = new YoFrameConvexPolygon2d("combinedPolygon", "", worldFrame, 30, registry);
      footToFootSegmentViz = new YoFrameLineSegment2d("footToFoot", "", worldFrame, registry);

      ArtifactList artifactList = new ArtifactList("Biped Support Polygon");

      YoArtifactPolygon dynamicGraphicYoPolygonArtifact = new YoArtifactPolygon("Combined Polygon", supportPolygonViz, Color.pink, false);
      artifactList.add(dynamicGraphicYoPolygonArtifact);

      for (RobotSide robotSide : RobotSide.values)
      {
         footPolygonsInAnkleZUp.put(robotSide, new FrameConvexPolygon2d());
         footPolygonsInMidFeetZUp.put(robotSide, new FrameConvexPolygon2d());
         sweetSpotsInAnkleZUp.put(robotSide, new FramePoint2d());
         sweetSpotsInMidFeetZUp.put(robotSide, new FramePoint2d());
      }

      footToFootLineSegmentInMidFeetZUp = new FrameLineSegment2d(midFeetZUp);
      footToFootLineSegmentInWorld = new FrameLineSegment2d(worldFrame);
      connectingEdge1 = new FrameLineSegment2d(midFeetZUp);
      connectingEdge2 = new FrameLineSegment2d(midFeetZUp);

      if (yoGraphicsListRegistry != null)
      {
         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }

      parentRegistry.addChild(registry);
   }

   public FrameConvexPolygon2d getSupportPolygonInMidFeetZUp()
   {
      return supportPolygonInMidFeetZUp;
   }

   public FrameConvexPolygon2d getSupportPolygonInWorld()
   {
      return supportPolygonInWorld;
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

   public FrameConvexPolygon2d getFootPolygonInMidFeetZUp(RobotSide robotSide)
   {
      return footPolygonsInMidFeetZUp.get(robotSide);
   }

   public SideDependentList<FrameConvexPolygon2d> getFootPolygonsInMidFeetZUp()
   {
      return footPolygonsInMidFeetZUp;
   }

   public FrameLineSegment2d getFootToFootLineSegmentInMidFeetZUp()
   {
      return footToFootLineSegmentInMidFeetZUp;
   }

   public FramePoint2d getSweetSpotCopy(RobotSide robotSide)
   {
      return new FramePoint2d(sweetSpotsInAnkleZUp.get(robotSide));
   }

   public FramePoint2d getSweetSpot(RobotSide robotSide)
   {
      return sweetSpotsInAnkleZUp.get(robotSide);
   }

   public void update(SideDependentList<List<FramePoint>> contactPoints)
   {
      timer.startTimer();
      boolean inDoubleSupport = true;
      boolean neitherFootIsSupportingFoot = true;
      RobotSide supportSide = null;

      connectingEdge1.setToNaN();
      connectingEdge2.setToNaN();

      for (RobotSide robotSide : RobotSide.values)
      {
         List<FramePoint> contactPointsForSide = contactPoints.get(robotSide);
         boolean isSupportFoot = contactPointsForSide.size() > 0;
         if (isSupportFoot)
         {
            supportSide = robotSide;
            neitherFootIsSupportingFoot = false;

            footPolygonsInAnkleZUp.get(robotSide).setIncludingFrameByProjectionOntoXYPlaneAndUpdate(ankleZUpFrames.get(robotSide), contactPointsForSide);
            footPolygonsInMidFeetZUp.get(robotSide).setIncludingFrameByProjectionOntoXYPlaneAndUpdate(midFeetZUp, contactPointsForSide);

            sweetSpotsInAnkleZUp.get(robotSide).setIncludingFrame(footPolygonsInAnkleZUp.get(robotSide).getCentroid()); // Sweet spots are the centroids of the foot polygons.
            sweetSpotsInMidFeetZUp.get(robotSide).setIncludingFrame(footPolygonsInMidFeetZUp.get(robotSide).getCentroid()); // Sweet spots are the centroids of the foot polygons.
         }
         else
         {
            inDoubleSupport = false;
         }
      }

      updateSupportPolygon(inDoubleSupport, neitherFootIsSupportingFoot, supportSide);

      timer.stopTimer();

      if (VISUALIZE)
         visualize();
   }

   private void updateSupportPolygon(boolean inDoubleSupport, boolean neitherFootIsSupportingFoot, RobotSide supportSide)
   {
      // Get the support polygon. If in double support, it is the combined polygon.
      // FIXME: Assumes the individual feet polygons are disjoint for faster computation. Will crash if the feet overlap.
      // If in single support, then the support polygon is just the foot polygon of the supporting foot.
      if (neitherFootIsSupportingFoot)
         throw new RuntimeException("neither foot is a supporting foot!");

      boolean useDumbCombination = !useConnectingEdges;

      if (inDoubleSupport)
      {
         if (useConnectingEdges)
         {
            useDumbCombination = !ConvexPolygonTools.combineDisjointPolygons(footPolygonsInMidFeetZUp.get(RobotSide.LEFT),
                  footPolygonsInMidFeetZUp.get(RobotSide.RIGHT), supportPolygonInMidFeetZUp, connectingEdge1, connectingEdge2);
            if (useDumbCombination)
               System.err.println("Feet polygons overlap!!!");
         }

         if (useDumbCombination)
         {
            supportPolygonInMidFeetZUp.setIncludingFrameAndUpdate(footPolygonsInMidFeetZUp.get(RobotSide.LEFT), footPolygonsInMidFeetZUp.get(RobotSide.RIGHT));
         }
      }
      else
      {
         supportPolygonInMidFeetZUp.setIncludingFrameAndUpdate(footPolygonsInMidFeetZUp.get(supportSide));
      }

      footToFootLineSegmentInMidFeetZUp.setFirstEndPoint(sweetSpotsInMidFeetZUp.get(RobotSide.LEFT));
      footToFootLineSegmentInMidFeetZUp.setSecondEndPoint(sweetSpotsInMidFeetZUp.get(RobotSide.RIGHT));

      supportPolygonInWorld.setIncludingFrameAndUpdate(supportPolygonInMidFeetZUp);
      supportPolygonInWorld.changeFrame(worldFrame);

      footToFootLineSegmentInWorld.setAndChangeFrame(footToFootLineSegmentInMidFeetZUp);
      footToFootLineSegmentInWorld.changeFrame(worldFrame);
   }

   public String toString()
   {
      return "supportPolygonInMidFeetZUp = " + supportPolygonInMidFeetZUp;
   }

   private void visualize()
   {
      supportPolygonViz.setFrameConvexPolygon2d(supportPolygonInWorld);
      footToFootSegmentViz.setFrameLineSegment2d(footToFootLineSegmentInWorld);
   }
}
