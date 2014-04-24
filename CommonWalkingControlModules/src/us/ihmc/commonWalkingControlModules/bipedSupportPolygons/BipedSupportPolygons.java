package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.awt.Color;
import java.util.List;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2dAndConnectingEdges;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.plotting.DynamicGraphicYoPolygonArtifact;
import com.yobotics.simulationconstructionset.time.GlobalTimer;
import com.yobotics.simulationconstructionset.util.graphics.ArtifactList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameConvexPolygon2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameLineSegment2d;

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

/*
 * FIXME: not rewindable!
 */
public class BipedSupportPolygons
{
   private static boolean VISUALIZE = true;

   private final YoVariableRegistry registry = new YoVariableRegistry("BipedSupportPolygons");

   // Reference frames:
   private final ReferenceFrame midFeetZUp;
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;

   // Polygons:
   private final SideDependentList<FrameConvexPolygon2d> footPolygonsInAnkleZUp = new SideDependentList<FrameConvexPolygon2d>();
   private final SideDependentList<FrameConvexPolygon2d> footPolygonsInMidFeetZUp = new SideDependentList<FrameConvexPolygon2d>();
   private final FrameConvexPolygon2d supportPolygonInMidFeetZUp = new FrameConvexPolygon2d();
   private final YoFrameConvexPolygon2d supportPolygonViz;
   private final YoFrameLineSegment2d footToFootSegmentViz;

   // Connecting edges between the two foot polygons to create the support polygon when in double support; null in single support
   private FrameLineSegment2d connectingEdge1, connectingEdge2;

   // 'Sweet spots', the spots inside each of the footPolygons where capture point placement leads to really good balance. Typically the middle of the foot or so:
   private final SideDependentList<FramePoint2d> sweetSpots = new SideDependentList<FramePoint2d>();

   // Line segment from one sweet spot to the other:
   private FrameLineSegment2d footToFootLineSegmentInMidFeetZUp;

   // In order to deal with intersecting polygons, it is much harder to calculate the connecting edges
   // So let's not use the connecting edges unles we need
   private boolean useConnectingEdges;

   private final GlobalTimer timer = new GlobalTimer(getClass().getSimpleName() + "Timer", registry);

   public BipedSupportPolygons(SideDependentList<ReferenceFrame> ankleZUpFrames, ReferenceFrame midFeetZUpFrame, YoVariableRegistry parentRegistry,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, boolean useConnectingEdges)
   {
      this.ankleZUpFrames = ankleZUpFrames;
      this.midFeetZUp = midFeetZUpFrame;
      this.useConnectingEdges = useConnectingEdges;

      supportPolygonViz = new YoFrameConvexPolygon2d("combinedPolygon", "", ReferenceFrame.getWorldFrame(), 30, registry);
      footToFootSegmentViz = new YoFrameLineSegment2d("footToFoot", "", ReferenceFrame.getWorldFrame(), registry);

      DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList("VTP Calculator");
      ArtifactList artifactList = new ArtifactList("VTP Calculator");

      DynamicGraphicYoPolygonArtifact dynamicGraphicYoPolygonArtifact = new DynamicGraphicYoPolygonArtifact("Combined Polygon", supportPolygonViz, Color.pink,
            false);
      artifactList.add(dynamicGraphicYoPolygonArtifact);

      for (RobotSide robotSide : RobotSide.values)
      {
         footPolygonsInAnkleZUp.put(robotSide, new FrameConvexPolygon2d());
         footPolygonsInMidFeetZUp.put(robotSide, new FrameConvexPolygon2d());
      }
      
      if (dynamicGraphicObjectsListRegistry != null)
      {
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);
         dynamicGraphicObjectsListRegistry.registerArtifactList(artifactList);
      }

      parentRegistry.addChild(registry);
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
      return new FramePoint2d(sweetSpots.get(robotSide));
   }

   public void update(SideDependentList<List<FramePoint>> contactPoints)
   {
      update(contactPoints, true);
   }
   
   public void update(SideDependentList<List<FramePoint>> contactPoints, boolean recycleMemory)
   {      
      timer.startTimer();
      boolean inDoubleSupport = true;
      boolean neitherFootIsSupportingFoot = true;
      RobotSide supportSide = null;
      for (RobotSide robotSide : RobotSide.values)
      {
         List<FramePoint> contactPointsForSide = contactPoints.get(robotSide);
         boolean isSupportFoot = contactPointsForSide.size() > 0;
         if (isSupportFoot)
         {
            supportSide = robotSide;
            neitherFootIsSupportingFoot = false;

            footPolygonsInAnkleZUp.get(robotSide).setIncludingFrameByProjectionOntoXYPlane(ankleZUpFrames.get(robotSide), contactPointsForSide);
            footPolygonsInMidFeetZUp.get(robotSide).setIncludingFrameByProjectionOntoXYPlane(midFeetZUp, contactPointsForSide);

            sweetSpots.set(robotSide, footPolygonsInAnkleZUp.get(robotSide).getCentroidCopy()); // Sweet spots are the centroids of the foot polygons.
         }
         else
         {
            inDoubleSupport = false;
         }
      }

      // Get the support polygon. If in double support, it is the combined polygon.
      // FIXME: Assumes the individual feet polygons are disjoint for faster computation. Will crash if the feet overlap.
      // If in single support, then the support polygon is just the foot polygon of the supporting foot.
      if (inDoubleSupport)
      {
         if (useConnectingEdges)
         {
            FrameConvexPolygon2dAndConnectingEdges supportPolygonAndEdgesInMidFeetZUp = FrameConvexPolygon2d.combineDisjointPolygons(
                  footPolygonsInMidFeetZUp.get(RobotSide.LEFT), footPolygonsInMidFeetZUp.get(RobotSide.RIGHT));

            if (supportPolygonAndEdgesInMidFeetZUp == null)
               System.err.println("Feet polygons overlap. Crashing!!!");

            // If feet are overlapping, then supportPolygonAndEdgesInMidFeetZUp = null...
            supportPolygonInMidFeetZUp.setIncludingFrame(supportPolygonAndEdgesInMidFeetZUp.getFrameConvexPolygon2d());

            connectingEdge1 = supportPolygonAndEdgesInMidFeetZUp.getConnectingEdge1();
            connectingEdge2 = supportPolygonAndEdgesInMidFeetZUp.getConnectingEdge2();
         }
         else
         {
            supportPolygonInMidFeetZUp.setIncludingFrame(footPolygonsInMidFeetZUp.get(RobotSide.LEFT), footPolygonsInMidFeetZUp.get(RobotSide.RIGHT));
         }

      }
      else
      {
         if (neitherFootIsSupportingFoot)
         {
            throw new RuntimeException("neither foot is a supporting foot!");
         }

         supportPolygonInMidFeetZUp.setIncludingFrame(footPolygonsInMidFeetZUp.get(supportSide));

         connectingEdge1 = null;
         connectingEdge2 = null;
      }

      this.footToFootLineSegmentInMidFeetZUp = new FrameLineSegment2d(sweetSpots.get(RobotSide.LEFT).changeFrameAndProjectToXYPlaneCopy(midFeetZUp), sweetSpots
            .get(RobotSide.RIGHT).changeFrameAndProjectToXYPlaneCopy(midFeetZUp));

      timer.stopTimer();

      if (VISUALIZE)
      {
         visualize();
      }
   }

   public String toString()
   {
      return "supportPolygonInMidFeetZUp = " + supportPolygonInMidFeetZUp;
   }

   private void visualize()
   {
      supportPolygonViz.setFrameConvexPolygon2d(supportPolygonInMidFeetZUp.changeFrameCopy(ReferenceFrame.getWorldFrame()));
      footToFootSegmentViz.setFrameLineSegment2d(footToFootLineSegmentInMidFeetZUp.changeFrameCopy(ReferenceFrame.getWorldFrame()));
   }
}
