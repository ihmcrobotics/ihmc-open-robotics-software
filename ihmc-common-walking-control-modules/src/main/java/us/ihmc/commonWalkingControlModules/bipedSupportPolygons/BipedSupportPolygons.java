package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import static us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer.defaultFeetColors;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
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

   // Polygons:
   private final SideDependentList<FixedFrameConvexPolygon2DBasics> footPolygonsInWorldFrame = new SideDependentList<>();
   private final SideDependentList<FixedFrameConvexPolygon2DBasics> footPolygonsInSoleFrame = new SideDependentList<>();
   private final SideDependentList<FixedFrameConvexPolygon2DBasics> footPolygonsInSoleZUpFrame = new SideDependentList<>();
   private final SideDependentList<FixedFrameConvexPolygon2DBasics> footPolygonsInMidFeetZUp = new SideDependentList<>();
   private final FixedFrameConvexPolygon2DBasics supportPolygonInMidFeetZUp;
   private final FixedFrameConvexPolygon2DBasics supportPolygonInWorld;

   private final YoFrameConvexPolygon2D supportPolygonViz;
   private final SideDependentList<YoFrameConvexPolygon2D> footPolygonsViz = new SideDependentList<>();

   private final FramePoint3D tempContactPosition = new FramePoint3D();

   public BipedSupportPolygons(CommonHumanoidReferenceFrames referenceFrames, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(referenceFrames.getMidFeetZUpFrame(), referenceFrames.getSoleZUpFrames(), referenceFrames.getSoleFrames(), parentRegistry, yoGraphicsListRegistry);
   }

   public BipedSupportPolygons(ReferenceFrame midFeetZUpFrame, SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                               SideDependentList<? extends ReferenceFrame> soleFrames, YoVariableRegistry parentRegistry,
                               YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      supportPolygonViz = new YoFrameConvexPolygon2D("combinedPolygon", "", worldFrame, 2 * maxNumberOfContactPointsPerFoot, registry);

      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoArtifactPolygon supportPolygonArtifact = new YoArtifactPolygon("Combined Polygon", supportPolygonViz, Color.pink, false);
      artifactList.add(supportPolygonArtifact);

      supportPolygonInMidFeetZUp = new FrameConvexPolygon2D(midFeetZUpFrame);
      supportPolygonInWorld = new FrameConvexPolygon2D(worldFrame);

      for (RobotSide robotSide : RobotSide.values)
      {
         footPolygonsInWorldFrame.put(robotSide, new FrameConvexPolygon2D(worldFrame));
         footPolygonsInSoleFrame.put(robotSide, new FrameConvexPolygon2D(soleFrames.get(robotSide)));
         footPolygonsInSoleZUpFrame.put(robotSide, new FrameConvexPolygon2D(soleZUpFrames.get(robotSide)));
         footPolygonsInMidFeetZUp.put(robotSide, new FrameConvexPolygon2D(midFeetZUpFrame));
         String robotSidePrefix = robotSide.getCamelCaseNameForStartOfExpression();

         YoFrameConvexPolygon2D footPolygonViz = new YoFrameConvexPolygon2D(robotSidePrefix + "FootPolygon", "", worldFrame, maxNumberOfContactPointsPerFoot,
                                                                            registry);
         footPolygonsViz.put(robotSide, footPolygonViz);
         YoArtifactPolygon footPolygonArtifact = new YoArtifactPolygon(robotSide.getCamelCaseNameForMiddleOfExpression() + " Foot Polygon", footPolygonViz,
                                                                       defaultFeetColors.get(robotSide), false);
         artifactList.add(footPolygonArtifact);
      }

      if (yoGraphicsListRegistry != null)
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

         FixedFrameConvexPolygon2DBasics footPolygonInWorldFrame = footPolygonsInWorldFrame.get(robotSide);
         FixedFrameConvexPolygon2DBasics footPolygonInSoleFrame = footPolygonsInSoleFrame.get(robotSide);
         FixedFrameConvexPolygon2DBasics footPolygonInSoleZUpFrame = footPolygonsInSoleZUpFrame.get(robotSide);
         FixedFrameConvexPolygon2DBasics footPolygonInMidFeetZUp = footPolygonsInMidFeetZUp.get(robotSide);

         footPolygonInWorldFrame.clearAndUpdate();
         footPolygonInSoleFrame.clearAndUpdate();
         footPolygonInSoleZUpFrame.clearAndUpdate();
         footPolygonInMidFeetZUp.clearAndUpdate();

         if (contactState.inContact())
         {
            supportSide = robotSide;
            neitherFootIsSupportingFoot = false;

            for (int i = 0; i < contactState.getTotalNumberOfContactPoints(); i++)
            {
               ContactPointInterface contactPoint = contactState.getContactPoints().get(i);
               if (!contactPoint.isInContact())
                  continue;

               FramePoint3DReadOnly position = contactPoint.getPosition();
               footPolygonInWorldFrame.addVertexMatchingFrame(position);
               footPolygonInSoleFrame.addVertexMatchingFrame(position);
               footPolygonInSoleZUpFrame.addVertexMatchingFrame(position);
               footPolygonInMidFeetZUp.addVertexMatchingFrame(position);
            }

            footPolygonInWorldFrame.update();
            footPolygonInSoleFrame.update();
            footPolygonInSoleZUpFrame.update();
            footPolygonInMidFeetZUp.update();
         }
         else
         {
            inDoubleSupport = false;
         }
      }

      updateSupportPolygon(inDoubleSupport, neitherFootIsSupportingFoot, supportSide);

      if (VISUALIZE)
         visualize();
   }

   public void updateUsingContactStateCommand(SideDependentList<PlaneContactStateCommand> contactStateCommands)
   {
      boolean inDoubleSupport = true;
      boolean neitherFootIsSupportingFoot = true;
      RobotSide supportSide = null;

      for (RobotSide robotSide : RobotSide.values)
      {
         PlaneContactStateCommand contactStateCommand = contactStateCommands.get(robotSide);

         FixedFrameConvexPolygon2DBasics footPolygonInWorldFrame = footPolygonsInWorldFrame.get(robotSide);
         FixedFrameConvexPolygon2DBasics footPolygonInSoleFrame = footPolygonsInSoleFrame.get(robotSide);
         FixedFrameConvexPolygon2DBasics footPolygonInSoleZUpFrame = footPolygonsInSoleZUpFrame.get(robotSide);
         FixedFrameConvexPolygon2DBasics footPolygonInMidFeetZUp = footPolygonsInMidFeetZUp.get(robotSide);

         footPolygonInWorldFrame.clearAndUpdate();
         footPolygonInSoleFrame.clearAndUpdate();
         footPolygonInSoleZUpFrame.clearAndUpdate();
         footPolygonInMidFeetZUp.clearAndUpdate();

         if (!contactStateCommand.isEmpty())
         {
            supportSide = robotSide;
            neitherFootIsSupportingFoot = false;

            for (int i = 0; i < contactStateCommand.getNumberOfContactPoints(); i++)
            {
               tempContactPosition.setIncludingFrame(contactStateCommand.getContactPoint(i));
               footPolygonInWorldFrame.addVertexMatchingFrame(tempContactPosition);
               footPolygonInSoleFrame.addVertexMatchingFrame(tempContactPosition);
               footPolygonInSoleZUpFrame.addVertexMatchingFrame(tempContactPosition);
               footPolygonInMidFeetZUp.addVertexMatchingFrame(tempContactPosition);
            }

            footPolygonInWorldFrame.update();
            footPolygonInSoleFrame.update();
            footPolygonInSoleZUpFrame.update();
            footPolygonInMidFeetZUp.update();
         }
         else
         {
            inDoubleSupport = false;
         }
      }

      updateSupportPolygon(inDoubleSupport, neitherFootIsSupportingFoot, supportSide);

      if (VISUALIZE)
         visualize();
   }

   private void updateSupportPolygon(boolean inDoubleSupport, boolean neitherFootIsSupportingFoot, RobotSide supportSide)
   {
      if (inDoubleSupport)
      {
         // If in double support, it is the combined polygon.
         supportPolygonInMidFeetZUp.set(footPolygonsInMidFeetZUp.get(RobotSide.LEFT), footPolygonsInMidFeetZUp.get(RobotSide.RIGHT));
      }
      else if (neitherFootIsSupportingFoot)
      {
         // When no foot is in contact the support polygon is empty.
         supportPolygonInMidFeetZUp.clearAndUpdate();
      }
      else
      {
         // If in single support, then the support polygon is just the foot polygon of the supporting foot.
         supportPolygonInMidFeetZUp.set(footPolygonsInMidFeetZUp.get(supportSide));
      }

      supportPolygonInWorld.setMatchingFrame(supportPolygonInMidFeetZUp, true);
   }

   private void visualize()
   {
      supportPolygonViz.set(supportPolygonInWorld);

      for (RobotSide robotSide : RobotSide.values)
      {
         YoFrameConvexPolygon2D footPolygonViz = footPolygonsViz.get(robotSide);
         FixedFrameConvexPolygon2DBasics footPolygon = footPolygonsInWorldFrame.get(robotSide);
         if (footPolygon.isEmpty())
            footPolygonViz.clear();
         else
            footPolygonViz.set(footPolygon);
      }
   }

   public FrameConvexPolygon2DReadOnly getSupportPolygonInMidFeetZUp()
   {
      return supportPolygonInMidFeetZUp;
   }

   public FrameConvexPolygon2DReadOnly getSupportPolygonInWorld()
   {
      return supportPolygonInWorld;
   }

   public FrameConvexPolygon2DReadOnly getFootPolygonInSoleFrame(RobotSide robotSide)
   {
      return footPolygonsInSoleFrame.get(robotSide);
   }

   public FrameConvexPolygon2DReadOnly getFootPolygonInSoleZUpFrame(RobotSide robotSide)
   {
      return footPolygonsInSoleZUpFrame.get(robotSide);
   }

   public FrameConvexPolygon2DReadOnly getFootPolygonInWorldFrame(RobotSide robotSide)
   {
      return footPolygonsInWorldFrame.get(robotSide);
   }

   @Override
   public String toString()
   {
      return "supportPolygonInMidFeetZUp = " + supportPolygonInMidFeetZUp;
   }
}
