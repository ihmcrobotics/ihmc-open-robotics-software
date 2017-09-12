package us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser;

import java.awt.Color;

import us.ihmc.quadrupedRobotics.geometry.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactOval;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.CommonQuadrupedReferenceFrames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedPathPreview
{
   private int iterations = 30;
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final FramePoint2D circleCenter2d = new FramePoint2D();

   private final CommonQuadrupedReferenceFrames referenceFrames;

   private SwingTargetGenerator swingTargetGenerator;
   private QuadrupedSupportPolygon fourFootSupportPolygon = new QuadrupedSupportPolygon();
   private final QuadrupedSupportPolygon updatedSupportPolygon = new QuadrupedSupportPolygon();
   private final QuadrupedSupportPolygon tempCommonSupportPolygon = new QuadrupedSupportPolygon();
   private final QuadrupedSupportPolygon tempPolygon = new QuadrupedSupportPolygon();

   private final YoDouble inscribedCircleRadius = new YoDouble("inscribedCircleRadius", registry);
   private final YoFramePoint[] circleCenters = new YoFramePoint[iterations]; //new YoFramePoint("circleCenter", ReferenceFrame.getWorldFrame(), registry);
   private final YoArtifactOval[] inscribedCircles = new YoArtifactOval[iterations];//new YoArtifactCircle("inscribedCircle", circleCenter, inscribedCircleRadius, Color.BLACK);

   private final YoFrameConvexPolygon2d[] commonSupportPolygons = new YoFrameConvexPolygon2d[iterations];
   private final YoArtifactPolygon[] commonSupportArtifactPolygons = new YoArtifactPolygon[iterations];

   private final YoFrameConvexPolygon2d[] tripleSupportPolygons = new YoFrameConvexPolygon2d[iterations * 2];
   private final YoArtifactPolygon[] tripleSupportArtifactPolygons = new YoArtifactPolygon[iterations * 2];

   private final FramePoint3D footLocation = new FramePoint3D(ReferenceFrame.getWorldFrame());

   private final FramePoint3D desiredPosition = new FramePoint3D(ReferenceFrame.getWorldFrame());
//   private final QuadrupedSupportPolygon emptyPolygon = new QuadrupedSupportPolygon();

   public QuadrupedPathPreview(SwingTargetGenerator swingTargetGenerator, CommonQuadrupedReferenceFrames referenceFrames, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      inscribedCircleRadius.set(0.04);

      this.referenceFrames = referenceFrames;
      for (int i = 0; i < commonSupportPolygons.length; i++)
      {
         String polygonName = "commonTriangle" + i;
         YoFrameConvexPolygon2d yoFrameConvexPolygon2d = new YoFrameConvexPolygon2d(polygonName, "", ReferenceFrame.getWorldFrame(), 3, registry);
         commonSupportPolygons[i] = yoFrameConvexPolygon2d;

         float saturation = 0.5f;
         float brightness = 0.5f;
         float hue = (float) (0.1 * i);
         commonSupportArtifactPolygons[i] = new YoArtifactPolygon(polygonName, yoFrameConvexPolygon2d, Color.getHSBColor(hue, saturation, brightness), false);

         yoGraphicsListRegistry.registerArtifact(polygonName, commonSupportArtifactPolygons[i]);

         circleCenters[i] = new YoFramePoint("circleCenter" + i, ReferenceFrame.getWorldFrame(), registry);
         inscribedCircles[i] = new YoArtifactOval("inscribedCircle" + i, circleCenters[i], inscribedCircleRadius, Color.BLACK);
         yoGraphicsListRegistry.registerArtifact(polygonName, inscribedCircles[i]);
      }

      for (int i = 0; i < tripleSupportPolygons.length; i++)
      {
         String polygonName = "tripleSupport" + i;
         YoFrameConvexPolygon2d yoFrameConvexPolygon2d = new YoFrameConvexPolygon2d(polygonName, "", ReferenceFrame.getWorldFrame(), 3, registry);
         tripleSupportPolygons[i] = yoFrameConvexPolygon2d;

         float saturation = 0.5f;
         float brightness = 0.5f;
         float hue = (float) (0.1 * i);

         tripleSupportArtifactPolygons[i] = new YoArtifactPolygon(polygonName, yoFrameConvexPolygon2d, Color.getHSBColor(hue, saturation, brightness), false);
         yoGraphicsListRegistry.registerArtifact(polygonName, tripleSupportArtifactPolygons[i]);
      }
      this.swingTargetGenerator = swingTargetGenerator;

      parentRegistry.addChild(registry);
   }

   public void update(RobotQuadrant swingLeg, FrameVector3D desiredBodyVelocity, double yawRate)
   {
      updateFeetLocations();
      updatedSupportPolygon.set(fourFootSupportPolygon);

      for (int i = 0; i < commonSupportPolygons.length; i++)
      {
         swingTargetGenerator.getSwingTarget(updatedSupportPolygon, swingLeg, desiredBodyVelocity, desiredPosition, yawRate);

         //get swing leg support
         QuadrupedSupportPolygon swingLegSupportPolygon = new QuadrupedSupportPolygon();
         updatedSupportPolygon.getAndRemoveFootstep(swingLegSupportPolygon, swingLeg);
         drawSupportPolygon(swingLegSupportPolygon, tripleSupportPolygons[i * 2]);

         //get next step in future support
         RobotQuadrant nextRegularGaitSwingQuadrant = swingLeg.getNextRegularGaitSwingQuadrant();
         QuadrupedSupportPolygon nextSwingLegSupportPolygon = new QuadrupedSupportPolygon();
         updatedSupportPolygon.getAndReplaceFootstep(nextSwingLegSupportPolygon, swingLeg, desiredPosition);
         nextSwingLegSupportPolygon.removeFootstep(nextRegularGaitSwingQuadrant);
         drawSupportPolygon(nextSwingLegSupportPolygon, tripleSupportPolygons[i * 2 + 1]);

         //if there's a common draw it
         swingLegSupportPolygon.getShrunkenCommonTriangle2d(nextSwingLegSupportPolygon, tempCommonSupportPolygon, tempPolygon, swingLeg, 0.02, 0.02, 0.02);
         drawSupportPolygon(tempCommonSupportPolygon, commonSupportPolygons[i]);

         tempCommonSupportPolygon.getCenterOfCircleOfRadiusInCornerOfTriangleAndCheckNotLargerThanInCircle(swingLeg, inscribedCircleRadius.getDoubleValue(), circleCenter2d);
         YoFramePoint circleCenter = circleCenters[i];
         circleCenter.set(circleCenter2d, 0.0);

         updatedSupportPolygon.setFootstep(swingLeg, desiredPosition);
         swingLeg = swingLeg.getNextRegularGaitSwingQuadrant();
      }
   }

   private void drawSupportPolygon(QuadrupedSupportPolygon supportPolygon, YoFrameConvexPolygon2d yoPolygon)
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         FramePoint3D footstep = supportPolygon.getFootstep(quadrant);
         if (footstep != null)
         {
            polygon.addVertex(footstep.getX(), footstep.getY());
         }
      }
      polygon.update();
      yoPolygon.setConvexPolygon2d(polygon);
   }

   private void updateFeetLocations()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ReferenceFrame footFrame = referenceFrames.getFootFrame(robotQuadrant);
         footLocation.setToZero(footFrame);
         footLocation.changeFrame(ReferenceFrame.getWorldFrame());
         fourFootSupportPolygon.setFootstep(robotQuadrant, footLocation);
      }
   }
}
