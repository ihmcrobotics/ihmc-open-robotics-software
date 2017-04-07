package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import static us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer.defaultFeetColors;

import java.awt.Color;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;

/*
 * FIXME: not rewindable!
 */
public class BipedSupportPolygons
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static boolean VISUALIZE = true;
   private static final int maxNumberOfContactPointsPerFoot = 6;

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
   private final SideDependentList<YoFrameConvexPolygon2d> footPolygonsViz = new SideDependentList<>();

   private final ExecutionTimer timer = new ExecutionTimer(getClass().getSimpleName() + "Timer", registry);

   public BipedSupportPolygons(SideDependentList<ReferenceFrame> ankleZUpFrames, ReferenceFrame midFeetZUpFrame,
         SideDependentList<ReferenceFrame> soleZUpFrames, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.ankleZUpFrames = ankleZUpFrames;
      this.midFeetZUp = midFeetZUpFrame;
      this.soleZUpFrames = soleZUpFrames;

      supportPolygonViz = new YoFrameConvexPolygon2d("combinedPolygon", "", worldFrame, 2 * maxNumberOfContactPointsPerFoot, registry);

      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoArtifactPolygon supportPolygonArtifact = new YoArtifactPolygon("Combined Polygon", supportPolygonViz, Color.pink, false);
      artifactList.add(supportPolygonArtifact);

      for (RobotSide robotSide : RobotSide.values)
      {
         footPolygonsInWorldFrame.put(robotSide, new FrameConvexPolygon2d());
         footPolygonsInSoleFrame.put(robotSide, new FrameConvexPolygon2d());
         footPolygonsInSoleZUpFrame.put(robotSide, new FrameConvexPolygon2d());
         footPolygonsInAnkleZUp.put(robotSide, new FrameConvexPolygon2d());
         footPolygonsInMidFeetZUp.put(robotSide, new FrameConvexPolygon2d());
         String robotSidePrefix = robotSide.getCamelCaseNameForStartOfExpression();

         YoFrameConvexPolygon2d footPolygonViz = new YoFrameConvexPolygon2d(robotSidePrefix + "FootPolygon", "", worldFrame, maxNumberOfContactPointsPerFoot, registry);
         footPolygonsViz.put(robotSide, footPolygonViz);
         YoArtifactPolygon footPolygonArtifact = new YoArtifactPolygon(robotSide.getCamelCaseNameForMiddleOfExpression() + " Foot Polygon", footPolygonViz, defaultFeetColors.get(robotSide), false);
         artifactList.add(footPolygonArtifact);
      }

      if (yoGraphicsListRegistry != null)
      {
         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }

      parentRegistry.addChild(registry);
   }

   private final FramePoint tempFramePoint = new FramePoint();

   public void updateUsingContactStates(SideDependentList<? extends PlaneContactState> contactStates)
   {
      timer.startMeasurement();
      boolean inDoubleSupport = true;
      boolean neitherFootIsSupportingFoot = true;
      RobotSide supportSide = null;

      for (RobotSide robotSide : RobotSide.values)
      {
         PlaneContactState contactState = contactStates.get(robotSide);

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
         }
         else
         {
            inDoubleSupport = false;
         }
      }

      updateSupportPolygon(inDoubleSupport, neitherFootIsSupportingFoot, supportSide);

      timer.stopMeasurement();

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

      if (inDoubleSupport)
      {
         supportPolygonInMidFeetZUp.setIncludingFrameAndUpdate(footPolygonsInMidFeetZUp.get(RobotSide.LEFT), footPolygonsInMidFeetZUp.get(RobotSide.RIGHT));
      }
      else
      {
         supportPolygonInMidFeetZUp.setIncludingFrameAndUpdate(footPolygonsInMidFeetZUp.get(supportSide));
      }

      supportPolygonInWorld.setIncludingFrameAndUpdate(supportPolygonInMidFeetZUp);
      supportPolygonInWorld.changeFrameAndProjectToXYPlane(worldFrame);
   }

   private void visualize()
   {
      supportPolygonViz.setFrameConvexPolygon2d(supportPolygonInWorld);

      for (RobotSide robotSide : RobotSide.values)
      {
         YoFrameConvexPolygon2d footPolygonViz = footPolygonsViz.get(robotSide);
         FrameConvexPolygon2d footPolygon = footPolygonsInWorldFrame.get(robotSide);
         if (footPolygon.isEmpty())
            footPolygonViz.hide();
         else
            footPolygonViz.setFrameConvexPolygon2d(footPolygon);
      }
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

   public String toString()
   {
      return "supportPolygonInMidFeetZUp = " + supportPolygonInMidFeetZUp;
   }
}
