package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import static us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer.defaultFeetColors;

import java.awt.Color;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;

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
   private final SideDependentList<FrameConvexPolygon2D> footPolygonsInWorldFrame = new SideDependentList<FrameConvexPolygon2D>();
   private final SideDependentList<FrameConvexPolygon2D> footPolygonsInSoleFrame = new SideDependentList<FrameConvexPolygon2D>();
   private final SideDependentList<FrameConvexPolygon2D> footPolygonsInSoleZUpFrame = new SideDependentList<FrameConvexPolygon2D>();
   private final SideDependentList<FrameConvexPolygon2D> footPolygonsInAnkleZUp = new SideDependentList<FrameConvexPolygon2D>();
   private final SideDependentList<FrameConvexPolygon2D> footPolygonsInMidFeetZUp = new SideDependentList<FrameConvexPolygon2D>();
   private final FrameConvexPolygon2D supportPolygonInMidFeetZUp = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D supportPolygonInWorld = new FrameConvexPolygon2D();

   private final YoFrameConvexPolygon2D supportPolygonViz;
   private final SideDependentList<YoFrameConvexPolygon2D> footPolygonsViz = new SideDependentList<>();

   private final ExecutionTimer timer = new ExecutionTimer(getClass().getSimpleName() + "Timer", registry);

   public BipedSupportPolygons(SideDependentList<ReferenceFrame> ankleZUpFrames, ReferenceFrame midFeetZUpFrame,
         SideDependentList<ReferenceFrame> soleZUpFrames, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.ankleZUpFrames = ankleZUpFrames;
      this.midFeetZUp = midFeetZUpFrame;
      this.soleZUpFrames = soleZUpFrames;

      supportPolygonViz = new YoFrameConvexPolygon2D("combinedPolygon", "", worldFrame, 2 * maxNumberOfContactPointsPerFoot, registry);

      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoArtifactPolygon supportPolygonArtifact = new YoArtifactPolygon("Combined Polygon", supportPolygonViz, Color.pink, false);
      artifactList.add(supportPolygonArtifact);

      for (RobotSide robotSide : RobotSide.values)
      {
         footPolygonsInWorldFrame.put(robotSide, new FrameConvexPolygon2D());
         footPolygonsInSoleFrame.put(robotSide, new FrameConvexPolygon2D());
         footPolygonsInSoleZUpFrame.put(robotSide, new FrameConvexPolygon2D());
         footPolygonsInAnkleZUp.put(robotSide, new FrameConvexPolygon2D());
         footPolygonsInMidFeetZUp.put(robotSide, new FrameConvexPolygon2D());
         String robotSidePrefix = robotSide.getCamelCaseNameForStartOfExpression();

         YoFrameConvexPolygon2D footPolygonViz = new YoFrameConvexPolygon2D(robotSidePrefix + "FootPolygon", "", worldFrame, maxNumberOfContactPointsPerFoot, registry);
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

   private final FramePoint3D tempFramePoint = new FramePoint3D();

   public void updateUsingContactStates(SideDependentList<? extends PlaneContactState> contactStates)
   {
      timer.startMeasurement();
      boolean inDoubleSupport = true;
      boolean neitherFootIsSupportingFoot = true;
      RobotSide supportSide = null;

      for (RobotSide robotSide : RobotSide.values)
      {
         PlaneContactState contactState = contactStates.get(robotSide);

         FrameConvexPolygon2D footPolygonInWorldFrame = footPolygonsInWorldFrame.get(robotSide);
         FrameConvexPolygon2D footPolygonInSoleFrame = footPolygonsInSoleFrame.get(robotSide);
         FrameConvexPolygon2D footPolygonInSoleZUpFrame = footPolygonsInSoleZUpFrame.get(robotSide);
         FrameConvexPolygon2D footPolygonInAnkleZUp = footPolygonsInAnkleZUp.get(robotSide);
         FrameConvexPolygon2D footPolygonInMidFeetZUp = footPolygonsInMidFeetZUp.get(robotSide);

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
               footPolygonInWorldFrame.addVertexMatchingFrame(tempFramePoint);
               footPolygonInSoleFrame.addVertexMatchingFrame(tempFramePoint);
               footPolygonInSoleZUpFrame.addVertexMatchingFrame(tempFramePoint);
               footPolygonInAnkleZUp.addVertexMatchingFrame(tempFramePoint);
               footPolygonInMidFeetZUp.addVertexMatchingFrame(tempFramePoint);
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
         supportPolygonInMidFeetZUp.setIncludingFrame(footPolygonsInMidFeetZUp.get(RobotSide.LEFT), footPolygonsInMidFeetZUp.get(RobotSide.RIGHT));
      }
      else
      {
         supportPolygonInMidFeetZUp.setIncludingFrame(footPolygonsInMidFeetZUp.get(supportSide));
      }

      supportPolygonInWorld.setIncludingFrame(supportPolygonInMidFeetZUp);
      supportPolygonInWorld.changeFrameAndProjectToXYPlane(worldFrame);
   }

   private void visualize()
   {
      supportPolygonViz.set(supportPolygonInWorld);

      for (RobotSide robotSide : RobotSide.values)
      {
         YoFrameConvexPolygon2D footPolygonViz = footPolygonsViz.get(robotSide);
         FrameConvexPolygon2D footPolygon = footPolygonsInWorldFrame.get(robotSide);
         if (footPolygon.isEmpty())
            footPolygonViz.clear();
         else
            footPolygonViz.set(footPolygon);
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

   public FrameConvexPolygon2D getSupportPolygonInMidFeetZUp()
   {
      return supportPolygonInMidFeetZUp;
   }

   public FrameConvexPolygon2D getSupportPolygonInWorld()
   {
      return supportPolygonInWorld;
   }

   public FrameConvexPolygon2D getFootPolygonInAnkleZUp(RobotSide robotSide)
   {
      return footPolygonsInAnkleZUp.get(robotSide);
   }

   public FrameConvexPolygon2D getFootPolygonInSoleFrame(RobotSide robotSide)
   {
      return footPolygonsInSoleFrame.get(robotSide);
   }

   public FrameConvexPolygon2D getFootPolygonInSoleZUpFrame(RobotSide robotSide)
   {
      return footPolygonsInSoleZUpFrame.get(robotSide);
   }

   public FrameConvexPolygon2D getFootPolygonInWorldFrame(RobotSide robotSide)
   {
      return footPolygonsInWorldFrame.get(robotSide);
   }

   public FrameConvexPolygon2D getFootPolygonInMidFeetZUp(RobotSide robotSide)
   {
      return footPolygonsInMidFeetZUp.get(robotSide);
   }

   public SideDependentList<FrameConvexPolygon2D> getFootPolygonsInMidFeetZUp()
   {
      return footPolygonsInMidFeetZUp;
   }

   public SideDependentList<FrameConvexPolygon2D> getFootPolygonsInWorldFrame()
   {
      return footPolygonsInWorldFrame;
   }

   public String toString()
   {
      return "supportPolygonInMidFeetZUp = " + supportPolygonInMidFeetZUp;
   }
}
