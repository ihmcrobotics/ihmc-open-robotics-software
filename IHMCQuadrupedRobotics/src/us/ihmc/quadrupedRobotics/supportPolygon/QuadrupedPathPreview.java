package us.ihmc.quadrupedRobotics.supportPolygon;

import java.awt.Color;

import us.ihmc.quadrupedRobotics.footstepChooser.SwingTargetGenerator;
import us.ihmc.quadrupedRobotics.referenceFrames.CommonQuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactCircle;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPolygon;

public class QuadrupedPathPreview
{
   private int iterations = 30;
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final FramePoint2d circleCenter2d = new FramePoint2d();

   private final CommonQuadrupedReferenceFrames referenceFrames;

   private SwingTargetGenerator swingTargetGenerator;
   private QuadrupedSupportPolygon fourFootSupportPolygon = new QuadrupedSupportPolygon();
   private final QuadrupedSupportPolygon updatedSupportPolygon = new QuadrupedSupportPolygon();

   private final DoubleYoVariable inscribedCircleRadius = new DoubleYoVariable("inscribedCircleRadius", registry);
   private final YoFramePoint[] circleCenters = new YoFramePoint[iterations]; //new YoFramePoint("circleCenter", ReferenceFrame.getWorldFrame(), registry);
   private final YoArtifactCircle[] inscribedCircles = new YoArtifactCircle[iterations];//new YoArtifactCircle("inscribedCircle", circleCenter, inscribedCircleRadius, Color.BLACK);

   private final YoFrameConvexPolygon2d[] commonSupportPolygons = new YoFrameConvexPolygon2d[iterations];
   private final YoArtifactPolygon[] commonSupportArtifactPolygons = new YoArtifactPolygon[iterations];

   private final YoFrameConvexPolygon2d[] tripleSupportPolygons = new YoFrameConvexPolygon2d[iterations * 2];
   private final YoArtifactPolygon[] tripleSupportArtifactPolygons = new YoArtifactPolygon[iterations * 2];

   private final FramePoint footLocation = new FramePoint(ReferenceFrame.getWorldFrame());

   private final FramePoint desiredPosition = new FramePoint(ReferenceFrame.getWorldFrame());
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
         inscribedCircles[i] = new YoArtifactCircle("inscribedCircle" + i, circleCenters[i], inscribedCircleRadius, Color.BLACK);
         yoGraphicsListRegistry.registerArtifact(polygonName, inscribedCircles[i]);
      }

      for (int i = 0; i < tripleSupportPolygons.length; i++)
      {
         String polygonName = "trippleSupport" + i;
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

   public void update(RobotQuadrant swingLeg, FrameVector desiredBodyVelocity, double yawRate)
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
         QuadrupedSupportPolygon shrunkenCommonSupportPolygon = new QuadrupedSupportPolygon();
         swingLegSupportPolygon.getShrunkenCommonTriangle2d(nextSwingLegSupportPolygon, shrunkenCommonSupportPolygon, swingLeg, 0.02, 0.02, 0.02);
         drawSupportPolygon(shrunkenCommonSupportPolygon, commonSupportPolygons[i]);

         shrunkenCommonSupportPolygon.getCenterOfCircleOfRadiusInCornerOfTriangle(swingLeg, inscribedCircleRadius.getDoubleValue(), circleCenter2d);
         YoFramePoint circleCenter = circleCenters[i];
         circleCenter.setXY(circleCenter2d);

         updatedSupportPolygon.setFootstep(swingLeg, desiredPosition);
         swingLeg = swingLeg.getNextRegularGaitSwingQuadrant();
      }
   }

   private void drawSupportPolygon(QuadrupedSupportPolygon supportPolygon, YoFrameConvexPolygon2d yoPolygon)
   {
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         FramePoint footstep = supportPolygon.getFootstep(quadrant);
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
