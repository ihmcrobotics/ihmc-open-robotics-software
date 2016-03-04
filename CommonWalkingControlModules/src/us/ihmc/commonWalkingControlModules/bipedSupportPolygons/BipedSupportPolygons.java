package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.awt.Color;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameLineSegment2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.GlobalTimer;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPolygon;

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
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static boolean VISUALIZE = true;

   private final YoVariableRegistry registry = new YoVariableRegistry("BipedSupportPolygons");

   // Reference frames:
   private final ReferenceFrame midFeetZUp;
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;
   private final SideDependentList<ReferenceFrame> soleZUpFrames;

   // Polygons:
   private final SideDependentList<FrameConvexPolygon2d> footPolygonsInWorldFrame = new SideDependentList<FrameConvexPolygon2d>();
   private final SideDependentList<FrameConvexPolygon2d> footPolygonsInSoleFrame = new SideDependentList<FrameConvexPolygon2d>();
   private final SideDependentList<FrameConvexPolygon2d> footPolygonsInSoleZUpFrame = new SideDependentList<FrameConvexPolygon2d>();
   private final SideDependentList<FrameConvexPolygon2d> footPolygonsInAnkleZUp = new SideDependentList<FrameConvexPolygon2d>();
   private final SideDependentList<FrameConvexPolygon2d> footPolygonsInMidFeetZUp = new SideDependentList<FrameConvexPolygon2d>();
   private final FrameConvexPolygon2d supportPolygonInMidFeetZUp = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d supportPolygonInWorld = new FrameConvexPolygon2d();
   private final YoFrameConvexPolygon2d supportPolygonViz;
   private final YoFrameLineSegment2d footToFootSegmentViz;

   // 'Sweet spots', the spots inside each of the footPolygons where capture point placement leads to really good balance. Typically the middle of the foot or so:
   private final SideDependentList<YoFramePoint2d> sweetSpotsInAnkleZUp = new SideDependentList<YoFramePoint2d>();
   private final SideDependentList<YoFramePoint2d> sweetSpotsInMidFeetZUp = new SideDependentList<YoFramePoint2d>();

   // Line segment from one sweet spot to the other:
   private final FrameLineSegment2d footToFootLineSegmentInMidFeetZUp;
   private final FrameLineSegment2d footToFootLineSegmentInWorld;

   private final GlobalTimer timer = new GlobalTimer(getClass().getSimpleName() + "Timer", registry);

   public BipedSupportPolygons(SideDependentList<ReferenceFrame> ankleZUpFrames, ReferenceFrame midFeetZUpFrame,
         SideDependentList<ReferenceFrame> soleZUpFrames, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.ankleZUpFrames = ankleZUpFrames;
      this.midFeetZUp = midFeetZUpFrame;
      this.soleZUpFrames = soleZUpFrames;

      supportPolygonViz = new YoFrameConvexPolygon2d("combinedPolygon", "", worldFrame, 30, registry);
      footToFootSegmentViz = new YoFrameLineSegment2d("footToFoot", "", worldFrame, registry);

      ArtifactList artifactList = new ArtifactList("Biped Support Polygon");

      YoArtifactPolygon dynamicGraphicYoPolygonArtifact = new YoArtifactPolygon("Combined Polygon", supportPolygonViz, Color.pink, false);
      artifactList.add(dynamicGraphicYoPolygonArtifact);

      for (RobotSide robotSide : RobotSide.values)
      {
         footPolygonsInWorldFrame.put(robotSide, new FrameConvexPolygon2d());
         footPolygonsInSoleFrame.put(robotSide, new FrameConvexPolygon2d());
         footPolygonsInSoleZUpFrame.put(robotSide, new FrameConvexPolygon2d());
         footPolygonsInAnkleZUp.put(robotSide, new FrameConvexPolygon2d());
         footPolygonsInMidFeetZUp.put(robotSide, new FrameConvexPolygon2d());
         String robotSidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         sweetSpotsInAnkleZUp.put(robotSide, new YoFramePoint2d(robotSidePrefix + "SweetSpotInAnkleZUp", ankleZUpFrames.get(robotSide), registry));
         sweetSpotsInMidFeetZUp.put(robotSide, new YoFramePoint2d(robotSidePrefix + "SweetSpotsInMidFeetZUp", midFeetZUpFrame, registry));
      }

      footToFootLineSegmentInMidFeetZUp = new FrameLineSegment2d(midFeetZUp);
      footToFootLineSegmentInWorld = new FrameLineSegment2d(worldFrame);

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

   public FrameConvexPolygon2d getFootPolygonInAnkleZUp(RobotSide robotSide)
   {
      return footPolygonsInAnkleZUp.get(robotSide);
   }

   public FrameConvexPolygon2d getFootPolygonInSoleFrame(RobotSide robotSide)
   {
      return footPolygonsInSoleFrame.get(robotSide);
   }

   public FrameConvexPolygon2d getFootPolygonInSoleZUpFrame(RobotSide robotSide)
   {
      return footPolygonsInSoleZUpFrame.get(robotSide);
   }

   public FrameConvexPolygon2d getFootPolygonInWorldFrame(RobotSide robotSide)
   {
      return footPolygonsInWorldFrame.get(robotSide);
   }

   public FrameConvexPolygon2d getFootPolygonInMidFeetZUp(RobotSide robotSide)
   {
      return footPolygonsInMidFeetZUp.get(robotSide);
   }

   public SideDependentList<FrameConvexPolygon2d> getFootPolygonsInMidFeetZUp()
   {
      return footPolygonsInMidFeetZUp;
   }

   public SideDependentList<FrameConvexPolygon2d> getFootPolygonsInWorldFrame()
   {
      return footPolygonsInWorldFrame;
   }

   public FrameLineSegment2d getFootToFootLineSegmentInMidFeetZUp()
   {
      return footToFootLineSegmentInMidFeetZUp;
   }

   private final FramePoint tempFramePoint = new FramePoint();

   public void updateUsingContactStates(SideDependentList<YoPlaneContactState> contactStates)
   {
      timer.startTimer();
      boolean inDoubleSupport = true;
      boolean neitherFootIsSupportingFoot = true;
      RobotSide supportSide = null;

      for (RobotSide robotSide : RobotSide.values)
      {
         YoPlaneContactState contactState = contactStates.get(robotSide);

         FrameConvexPolygon2d footPolygonInWorldFrame = footPolygonsInWorldFrame.get(robotSide);
         FrameConvexPolygon2d footPolygonInSoleFrame = footPolygonsInSoleFrame.get(robotSide);
         FrameConvexPolygon2d footPolygonInSoleZUpFrame = footPolygonsInSoleZUpFrame.get(robotSide);
         FrameConvexPolygon2d footPolygonInAnkleZUp = footPolygonsInAnkleZUp.get(robotSide);
         FrameConvexPolygon2d footPolygonInMidFeetZUp = footPolygonsInMidFeetZUp.get(robotSide);

         footPolygonInWorldFrame.clearAndUpdate(worldFrame);
         footPolygonInSoleFrame.clearAndUpdate(contactState.getPlaneFrame());
         footPolygonInSoleZUpFrame.clearAndUpdate(soleZUpFrames.get(robotSide));
         footPolygonInAnkleZUp.clearAndUpdate(ankleZUpFrames.get(robotSide));
         footPolygonInMidFeetZUp.clearAndUpdate(midFeetZUp);

         if (contactState.inContact())
         {
            supportSide = robotSide;
            neitherFootIsSupportingFoot = false;

            for (int i = 0; i < contactState.getTotalNumberOfContactPoints(); i++)
            {
               ContactPointInterface contactPoint = contactState.getContactPoints().get(i);
               if (!contactPoint.isInContact())
                  continue;

               contactPoint.getPosition(tempFramePoint);
               footPolygonInWorldFrame.addVertexByProjectionOntoXYPlane(tempFramePoint);
               footPolygonInSoleFrame.addVertexByProjectionOntoXYPlane(tempFramePoint);
               footPolygonInSoleZUpFrame.addVertexByProjectionOntoXYPlane(tempFramePoint);
               footPolygonInAnkleZUp.addVertexByProjectionOntoXYPlane(tempFramePoint);
               footPolygonInMidFeetZUp.addVertexByProjectionOntoXYPlane(tempFramePoint);
            }

            footPolygonInWorldFrame.update();
            footPolygonInSoleFrame.update();
            footPolygonInSoleZUpFrame.update();
            footPolygonInAnkleZUp.update();
            footPolygonInMidFeetZUp.update();

            sweetSpotsInAnkleZUp.get(robotSide).set(footPolygonInAnkleZUp.getCentroid()); // Sweet spots are the centroids of the foot polygons.
            sweetSpotsInMidFeetZUp.get(robotSide).set(footPolygonInMidFeetZUp.getCentroid()); // Sweet spots are the centroids of the foot polygons.
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

   private final FramePoint2d tempFramePoint2d = new FramePoint2d();

   private void updateSupportPolygon(boolean inDoubleSupport, boolean neitherFootIsSupportingFoot, RobotSide supportSide)
   {
      // Get the support polygon. If in double support, it is the combined polygon.
      // FIXME: Assumes the individual feet polygons are disjoint for faster computation. Will crash if the feet overlap.
      // If in single support, then the support polygon is just the foot polygon of the supporting foot.
      if (neitherFootIsSupportingFoot)
         throw new RuntimeException("neither foot is a supporting foot!");

      if (inDoubleSupport)
      {
         supportPolygonInMidFeetZUp.setIncludingFrameAndUpdate(footPolygonsInMidFeetZUp.get(RobotSide.LEFT), footPolygonsInMidFeetZUp.get(RobotSide.RIGHT));
      }
      else
      {
         supportPolygonInMidFeetZUp.setIncludingFrameAndUpdate(footPolygonsInMidFeetZUp.get(supportSide));
      }

      sweetSpotsInMidFeetZUp.get(RobotSide.LEFT).getFrameTuple2dIncludingFrame(tempFramePoint2d);
      footToFootLineSegmentInMidFeetZUp.setFirstEndPoint(tempFramePoint2d);

      sweetSpotsInMidFeetZUp.get(RobotSide.RIGHT).getFrameTuple2dIncludingFrame(tempFramePoint2d);
      footToFootLineSegmentInMidFeetZUp.setSecondEndPoint(tempFramePoint2d);

      supportPolygonInWorld.setIncludingFrameAndUpdate(supportPolygonInMidFeetZUp);
      supportPolygonInWorld.changeFrame(worldFrame);

      footToFootLineSegmentInWorld.setAndChangeFrame(footToFootLineSegmentInMidFeetZUp);
      footToFootLineSegmentInWorld.changeFrame(worldFrame);
   }

   public ReferenceFrame getMidFeetZUpFrame()
   {
      return midFeetZUp;
   }

   public SideDependentList<ReferenceFrame> getAnkleZUpFrames()
   {
      return ankleZUpFrames;
   }

   public SideDependentList<ReferenceFrame> getSoleZUpFrames()
   {
      return soleZUpFrames;
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
