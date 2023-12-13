package us.ihmc.commonWalkingControlModules.capturePoint;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactPointBasics;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ICPControlPolygons implements SCS2YoGraphicHolder
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final Color combinedColor = Color.red;
   private static final Color leftFootColor = new Color(250, 128, 114);
   private static final Color rightFootColor = new Color(255, 160, 122);
   private static final SideDependentList<Color> feetColors = new SideDependentList<>(leftFootColor, rightFootColor);

   private static boolean VISUALIZE = false;
   private static final int maxNumberOfContactPointsPerFoot = 6;

   private final YoRegistry registry = new YoRegistry("ICPControlPolygons");

   // Polygons:
   private final SideDependentList<FixedFrameConvexPolygon2DBasics> footControlPolygonsInWorldFrame = new SideDependentList<>();

   private final FixedFrameConvexPolygon2DBasics controlPolygonInWorld;

   private final YoFrameConvexPolygon2D controlPolygonViz;
   private final SideDependentList<YoFrameConvexPolygon2D> controlFootPolygonsViz = new SideDependentList<>();

   private final ICPControlPlane icpControlPlane;

   private final FramePoint3D tempProjectedContactPosition = new FramePoint3D();
   private final FramePoint3D tempContactPosition = new FramePoint3D();

   public ICPControlPolygons(ICPControlPlane icpControlPlane, YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.icpControlPlane = icpControlPlane;

      controlPolygonViz = new YoFrameConvexPolygon2D("combinedPolygon", "", worldFrame, 2 * maxNumberOfContactPointsPerFoot, registry);

      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoArtifactPolygon controlPolygonArtifact = new YoArtifactPolygon("Combined Control Polygon", controlPolygonViz, combinedColor, false);
      artifactList.add(controlPolygonArtifact);

      controlPolygonInWorld = new FrameConvexPolygon2D(worldFrame);

      for (RobotSide robotSide : RobotSide.values)
      {
         footControlPolygonsInWorldFrame.put(robotSide, new FrameConvexPolygon2D(worldFrame));
         String robotSidePrefix = robotSide.getCamelCaseNameForStartOfExpression();

         if (VISUALIZE)
         {
            YoFrameConvexPolygon2D controlFootPolygonViz = new YoFrameConvexPolygon2D(robotSidePrefix + "controlFootPolygon",
                                                                                      "",
                                                                                      worldFrame,
                                                                                      maxNumberOfContactPointsPerFoot,
                                                                                      registry);
            controlFootPolygonsViz.put(robotSide, controlFootPolygonViz);
            YoArtifactPolygon footPolygonArtifact = new YoArtifactPolygon(robotSide.getCamelCaseNameForMiddleOfExpression() + " Control Foot Polygon",
                                                                          controlFootPolygonViz,
                                                                          feetColors.get(robotSide),
                                                                          false);
            artifactList.add(footPolygonArtifact);
         }
      }

      if (VISUALIZE && yoGraphicsListRegistry != null)
      {
         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }

      parentRegistry.addChild(registry);
   }

   public void updateUsingContactStates(SideDependentList<? extends PlaneContactState> contactStates)
   {
      boolean inDoubleSupport = true;
      boolean neitherFootIsSupportingFoot = true;
      RobotSide supportSide = null;

      for (RobotSide robotSide : RobotSide.values)
      {
         PlaneContactState contactState = contactStates.get(robotSide);

         FixedFrameConvexPolygon2DBasics footPolygonInWorldFrame = footControlPolygonsInWorldFrame.get(robotSide);

         footPolygonInWorldFrame.clearAndUpdate();

         if (contactState.inContact())
         {
            supportSide = robotSide;
            neitherFootIsSupportingFoot = false;

            for (int i = 0; i < contactState.getTotalNumberOfContactPoints(); i++)
            {
               ContactPointBasics contactPoint = contactState.getContactPoints().get(i);
               if (!contactPoint.isInContact())
                  continue;

               icpControlPlane.projectPointOntoControlPlane(worldFrame, contactPoint, tempProjectedContactPosition);
               footPolygonInWorldFrame.addVertexMatchingFrame(tempProjectedContactPosition);
            }

            footPolygonInWorldFrame.update();
         }
         else
         {
            inDoubleSupport = false;
         }
      }

      updateSupportPolygon(inDoubleSupport, neitherFootIsSupportingFoot, supportSide);

      updateVisualize();
   }

   public void updateUsingContactStateCommand(SideDependentList<PlaneContactStateCommand> contactStateCommands)
   {
      boolean inDoubleSupport = true;
      boolean neitherFootIsSupportingFoot = true;
      RobotSide supportSide = null;

      for (RobotSide robotSide : RobotSide.values)
      {
         PlaneContactStateCommand contactStateCommand = contactStateCommands.get(robotSide);

         FixedFrameConvexPolygon2DBasics footPolygonInWorldFrame = footControlPolygonsInWorldFrame.get(robotSide);

         footPolygonInWorldFrame.clearAndUpdate();

         if (!contactStateCommand.isEmpty())
         {
            supportSide = robotSide;
            neitherFootIsSupportingFoot = false;

            for (int i = 0; i < contactStateCommand.getNumberOfContactPoints(); i++)
            {
               tempContactPosition.setIncludingFrame(contactStateCommand.getContactPoint(i));
               icpControlPlane.projectPointOntoControlPlane(worldFrame, tempContactPosition, tempProjectedContactPosition);
               footPolygonInWorldFrame.addVertexMatchingFrame(tempProjectedContactPosition);
            }

            footPolygonInWorldFrame.update();
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
         controlPolygonInWorld.set(footControlPolygonsInWorldFrame.get(RobotSide.LEFT), footControlPolygonsInWorldFrame.get(RobotSide.RIGHT));
      }
      else
      {
         controlPolygonInWorld.set(footControlPolygonsInWorldFrame.get(supportSide));
      }
   }

   private void updateVisualize()
   {
      controlPolygonViz.set(controlPolygonInWorld);

      if (VISUALIZE)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            YoFrameConvexPolygon2D footPolygonViz = controlFootPolygonsViz.get(robotSide);
            FixedFrameConvexPolygon2DBasics footPolygon = footControlPolygonsInWorldFrame.get(robotSide);
            if (footPolygon.isEmpty())
               footPolygonViz.clear();
            else
               footPolygonViz.set(footPolygon);
         }
      }
   }

   public FrameConvexPolygon2DReadOnly getFootControlPolygonInWorldFrame(RobotSide robotSide)
   {
      return footControlPolygonsInWorldFrame.get(robotSide);
   }

   public FrameConvexPolygon2DReadOnly getControlPolygonInWorldFrame()
   {
      return controlPolygonInWorld;
   }

   public ICPControlPlane getIcpControlPlane()
   {
      return icpControlPlane;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      if (!VISUALIZE)
         return null;

      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPolygon2D("Combined Control Polygon",
                                                                      controlPolygonViz,
                                                                      ColorDefinitions.argb(combinedColor.getRGB())));

      for (RobotSide robotSide : RobotSide.values)
      {
         group.addChild(YoGraphicDefinitionFactory.newYoGraphicPolygon2D(robotSide.getPascalCaseName() + " Control Foot Polygon",
                                                                         controlFootPolygonsViz.get(robotSide),
                                                                         ColorDefinitions.argb(feetColors.get(robotSide).getRGB())));
      }
      return group;
   }
}
