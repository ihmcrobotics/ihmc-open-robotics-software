package us.ihmc.commonWalkingControlModules.capturePoint;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactPointInterface;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ICPControlPolygons
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final Color combinedColor = Color.red;
   private static final Color leftFootColor = new Color(250, 128 , 114);
   private static final Color rightFootColor = new Color(255, 160, 122);
   private static final SideDependentList<Color> feetColors = new SideDependentList<>(leftFootColor, rightFootColor);

   private static boolean VISUALIZE = false;
   private static final int maxNumberOfContactPointsPerFoot = 6;

   private final YoVariableRegistry registry = new YoVariableRegistry("ICPControlPolygons");

   // Reference frames:
   private final ReferenceFrame midFeetZUp;

   // Polygons:
   private final SideDependentList<FrameConvexPolygon2d> footControlPolygonsInWorldFrame = new SideDependentList<>();
   private final SideDependentList<FrameConvexPolygon2d> footControlPolygonsInMidFeetZUp = new SideDependentList<>();

   private final FrameConvexPolygon2d controlPolygonInMidFeetZUp = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d controlPolygonInWorld = new FrameConvexPolygon2d();

   private final YoFrameConvexPolygon2d controlPolygonViz;
   private final SideDependentList<YoFrameConvexPolygon2d> controlFootPolygonsViz = new SideDependentList<>();

   private final ICPControlPlane icpControlPlane;

   public ICPControlPolygons(ICPControlPlane icpControlPlane, ReferenceFrame midFeetZUpFrame, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.icpControlPlane = icpControlPlane;
      this.midFeetZUp = midFeetZUpFrame;

      controlPolygonViz = new YoFrameConvexPolygon2d("combinedPolygon", "", worldFrame, 2 * maxNumberOfContactPointsPerFoot, registry);

      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoArtifactPolygon controlPolygonArtifact = new YoArtifactPolygon("Combined Control Polygon", controlPolygonViz, combinedColor, false);
      artifactList.add(controlPolygonArtifact);

      for (RobotSide robotSide : RobotSide.values)
      {
         footControlPolygonsInWorldFrame.put(robotSide, new FrameConvexPolygon2d());
         footControlPolygonsInMidFeetZUp.put(robotSide, new FrameConvexPolygon2d());
         String robotSidePrefix = robotSide.getCamelCaseNameForStartOfExpression();

         YoFrameConvexPolygon2d controlFootPolygonViz = new YoFrameConvexPolygon2d(robotSidePrefix + "controlFootPolygon", "", worldFrame, maxNumberOfContactPointsPerFoot, registry);
         controlFootPolygonsViz.put(robotSide, controlFootPolygonViz);
         YoArtifactPolygon footPolygonArtifact = new YoArtifactPolygon(robotSide.getCamelCaseNameForMiddleOfExpression() + " Control Foot Polygon", controlFootPolygonViz, feetColors.get(robotSide), false);
         artifactList.add(footPolygonArtifact);
      }

      artifactList.setVisible(VISUALIZE);

      if (yoGraphicsListRegistry != null)
      {
         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }

      parentRegistry.addChild(registry);
   }

   private final FramePoint3D tempFramePoint = new FramePoint3D();

   public void updateUsingContactStates(SideDependentList<? extends PlaneContactState> contactStates)
   {
      boolean inDoubleSupport = true;
      boolean neitherFootIsSupportingFoot = true;
      RobotSide supportSide = null;

      for (RobotSide robotSide : RobotSide.values)
      {
         PlaneContactState contactState = contactStates.get(robotSide);

         FrameConvexPolygon2d footPolygonInWorldFrame = footControlPolygonsInWorldFrame.get(robotSide);
         FrameConvexPolygon2d footPolygonInMidFeetZUp = footControlPolygonsInMidFeetZUp.get(robotSide);

         footPolygonInWorldFrame.clearAndUpdate(worldFrame);
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

               icpControlPlane.projectPointOntoControlPlane(worldFrame, contactPoint.getPosition(), tempFramePoint);
               footPolygonInWorldFrame.addVertexByProjectionOntoXYPlane(tempFramePoint);
               footPolygonInMidFeetZUp.addVertexByProjectionOntoXYPlane(tempFramePoint);
            }

            footPolygonInWorldFrame.update();
            footPolygonInMidFeetZUp.update();
         }
         else
         {
            inDoubleSupport = false;
         }
      }

      updateSupportPolygon(inDoubleSupport, neitherFootIsSupportingFoot, supportSide);

      updateVisualize();
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
         controlPolygonInMidFeetZUp.setIncludingFrameAndUpdate(footControlPolygonsInMidFeetZUp.get(RobotSide.LEFT), footControlPolygonsInMidFeetZUp.get(RobotSide.RIGHT));
      }
      else
      {
         controlPolygonInMidFeetZUp.setIncludingFrameAndUpdate(footControlPolygonsInMidFeetZUp.get(supportSide));
      }

      controlPolygonInWorld.setIncludingFrameAndUpdate(controlPolygonInMidFeetZUp);
      controlPolygonInWorld.changeFrameAndProjectToXYPlane(worldFrame);
   }

   private void updateVisualize()
   {
      controlPolygonViz.setFrameConvexPolygon2d(controlPolygonInWorld);

      for (RobotSide robotSide : RobotSide.values)
      {
         YoFrameConvexPolygon2d footPolygonViz = controlFootPolygonsViz.get(robotSide);
         FrameConvexPolygon2d footPolygon = footControlPolygonsInWorldFrame.get(robotSide);
         if (footPolygon.isEmpty())
            footPolygonViz.hide();
         else
            footPolygonViz.setFrameConvexPolygon2d(footPolygon);
      }
   }

   public FrameConvexPolygon2d getFootControlPolygonInWorldFrame(RobotSide robotSide)
   {
      return footControlPolygonsInWorldFrame.get(robotSide);
   }

   public ICPControlPlane getIcpControlPlane()
   {
      return icpControlPlane;
   }
}
