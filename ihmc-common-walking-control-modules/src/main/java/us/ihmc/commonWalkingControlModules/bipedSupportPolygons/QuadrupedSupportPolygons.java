package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.awt.Color;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class QuadrupedSupportPolygons
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final Color defaultFrontLeftColor = new Color(0.85f, 0.35f, 0.65f, 1.0f);
   private static final Color defaultHindRightColor = new Color(0.85f, 0.35f, 0.65f, 1.0f);
   private static final Color defaultFrontRightColor = new Color(0.15f, 0.8f, 0.15f, 1.0f);
   private static final Color defaultHindLeftColor = new Color(0.15f, 0.8f, 0.15f, 1.0f);
   private static final QuadrantDependentList<Color> defaultFeetColors = new QuadrantDependentList<>(defaultFrontLeftColor, defaultFrontRightColor,
                                                                                                     defaultHindLeftColor, defaultHindRightColor);

   private static boolean VISUALIZE = true;

   private final YoRegistry registry = new YoRegistry("QuadrupedSupportPolygons");

   // Reference frames:
   private final ReferenceFrame centerOfFeetZUpFrame;
   private final QuadrantDependentList<MovingReferenceFrame> soleZUpFrames;

   // Polygons:
   private final QuadrantDependentList<FrameConvexPolygon2D> footPolygonsInWorldFrame = new QuadrantDependentList<>();
   private final QuadrantDependentList<FrameConvexPolygon2D> footPolygonsInSoleFrame = new QuadrantDependentList<>();
   private final QuadrantDependentList<FrameConvexPolygon2D> footPolygonsInSoleZUpFrame = new QuadrantDependentList<>();
   private final QuadrantDependentList<FrameConvexPolygon2D> footPolygonsInMidFeetZUp = new QuadrantDependentList<>();
   private final FrameConvexPolygon2D supportPolygonInMidFeetZUp = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D supportPolygonInWorld = new FrameConvexPolygon2D();

   private final YoFrameConvexPolygon2D supportPolygonViz;
   private final YoFramePoint2D supportPolygonCentroid;
   private final QuadrantDependentList<YoFrameConvexPolygon2D> footPolygonsViz = new QuadrantDependentList<>();

   public QuadrupedSupportPolygons(ReferenceFrame centerOfFeetZUpFrame, QuadrantDependentList<YoPlaneContactState> contactStates,
                                   QuadrantDependentList<MovingReferenceFrame> soleZUpFrames, YoRegistry parentRegistry,
                                   YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.centerOfFeetZUpFrame = centerOfFeetZUpFrame;
      this.soleZUpFrames = soleZUpFrames;

      supportPolygonViz = new YoFrameConvexPolygon2D("combinedPolygon", "", worldFrame, 4 * contactStates.get(RobotQuadrant.FRONT_LEFT).getTotalNumberOfContactPoints(),
                                                     registry);
      supportPolygonCentroid = new YoFramePoint2D("supportPolygonCentroid", worldFrame, registry);

      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoArtifactPolygon supportPolygonArtifact = new YoArtifactPolygon("Combined Polygon", supportPolygonViz, Color.pink, false);
      artifactList.add(supportPolygonArtifact);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footPolygonsInWorldFrame.put(robotQuadrant, new FrameConvexPolygon2D());
         footPolygonsInSoleFrame.put(robotQuadrant, new FrameConvexPolygon2D());
         footPolygonsInSoleZUpFrame.put(robotQuadrant, new FrameConvexPolygon2D());
         footPolygonsInMidFeetZUp.put(robotQuadrant, new FrameConvexPolygon2D());
         String robotSidePrefix = robotQuadrant.getCamelCaseNameForStartOfExpression();

         YoFrameConvexPolygon2D footPolygonViz = new YoFrameConvexPolygon2D(robotSidePrefix + "FootPolygon", "", worldFrame,
                                                                            contactStates.get(robotQuadrant).getTotalNumberOfContactPoints(), registry);
         footPolygonsViz.put(robotQuadrant, footPolygonViz);
         YoArtifactPolygon footPolygonArtifact = new YoArtifactPolygon(robotQuadrant.getCamelCaseNameForMiddleOfExpression() + " Foot Polygon", footPolygonViz,
                                                                       defaultFeetColors.get(robotQuadrant), false);
         artifactList.add(footPolygonArtifact);
      }

      if (yoGraphicsListRegistry != null)
      {
         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }

      parentRegistry.addChild(registry);
   }

   public void updateUsingContactStates(QuadrantDependentList<? extends PlaneContactState> contactStates)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         PlaneContactState contactState = contactStates.get(robotQuadrant);

         FrameConvexPolygon2D footPolygonInWorldFrame = footPolygonsInWorldFrame.get(robotQuadrant);
         FrameConvexPolygon2D footPolygonInSoleFrame = footPolygonsInSoleFrame.get(robotQuadrant);
         FrameConvexPolygon2D footPolygonInSoleZUpFrame = footPolygonsInSoleZUpFrame.get(robotQuadrant);
         FrameConvexPolygon2D footPolygonInMidFeetZUp = footPolygonsInMidFeetZUp.get(robotQuadrant);

         footPolygonInWorldFrame.clearAndUpdate(worldFrame);
         footPolygonInSoleFrame.clearAndUpdate(contactState.getPlaneFrame());
         footPolygonInSoleZUpFrame.clearAndUpdate(soleZUpFrames.get(robotQuadrant));
         footPolygonInMidFeetZUp.clearAndUpdate(centerOfFeetZUpFrame);

         if (contactState.inContact())
         {
            for (int i = 0; i < contactState.getTotalNumberOfContactPoints(); i++)
            {
               ContactPointBasics contactPoint = contactState.getContactPoints().get(i);
               if (!contactPoint.isInContact())
                  continue;

               footPolygonInWorldFrame.addVertexMatchingFrame(contactPoint);
               footPolygonInSoleFrame.addVertexMatchingFrame(contactPoint);
               footPolygonInSoleZUpFrame.addVertexMatchingFrame(contactPoint);
               footPolygonInMidFeetZUp.addVertexMatchingFrame(contactPoint);
            }

            footPolygonInWorldFrame.update();
            footPolygonInSoleFrame.update();
            footPolygonInSoleZUpFrame.update();
            footPolygonInMidFeetZUp.update();
         }
      }

      updateSupportPolygon(contactStates);
      supportPolygonCentroid.set(supportPolygonInWorld.getCentroid());

      if (VISUALIZE)
         visualize();
   }

   private void updateSupportPolygon(QuadrantDependentList<? extends PlaneContactState> contactStates)
   {
      // Get the support polygon.
      supportPolygonInMidFeetZUp.clearAndUpdate();
      supportPolygonInMidFeetZUp.setReferenceFrame(centerOfFeetZUpFrame);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (!contactStates.get(robotQuadrant).inContact())
            continue;
         supportPolygonInMidFeetZUp.addVertices(footPolygonsInMidFeetZUp.get(robotQuadrant));
      }
      supportPolygonInMidFeetZUp.update();

      supportPolygonInWorld.setIncludingFrame(supportPolygonInMidFeetZUp);
      supportPolygonInWorld.changeFrameAndProjectToXYPlane(worldFrame);
   }

   private void visualize()
   {
      supportPolygonViz.set(supportPolygonInWorld);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         YoFrameConvexPolygon2D footPolygonViz = footPolygonsViz.get(robotQuadrant);
         FrameConvexPolygon2D footPolygon = footPolygonsInWorldFrame.get(robotQuadrant);
         if (footPolygon.isEmpty())
            footPolygonViz.clear();
         else
            footPolygonViz.set(footPolygon);
      }
   }

   public ReferenceFrame getMidFeetZUpFrame()
   {
      return centerOfFeetZUpFrame;
   }

   public QuadrantDependentList<MovingReferenceFrame> getSoleZUpFrames()
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

   public FrameConvexPolygon2D getFootPolygonInSoleFrame(RobotQuadrant robotQuadrant)
   {
      return footPolygonsInSoleFrame.get(robotQuadrant);
   }

   public FrameConvexPolygon2D getFootPolygonInSoleZUpFrame(RobotQuadrant robotQuadrant)
   {
      return footPolygonsInSoleZUpFrame.get(robotQuadrant);
   }

   public FrameConvexPolygon2D getFootPolygonInWorldFrame(RobotQuadrant robotQuadrant)
   {
      return footPolygonsInWorldFrame.get(robotQuadrant);
   }

   public QuadrantDependentList<FrameConvexPolygon2D> getFootPolygonsInWorldFrame()
   {
      return footPolygonsInWorldFrame;
   }

   public String toString()
   {
      return "supportPolygonInMidFeetZUp = " + supportPolygonInMidFeetZUp;
   }
}
