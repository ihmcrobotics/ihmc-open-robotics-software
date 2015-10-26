package us.ihmc.quadrupedRobotics.supportPolygon;

import java.awt.Color;

import us.ihmc.quadrupedRobotics.footstepChooser.SwingTargetGenerator;
import us.ihmc.quadrupedRobotics.referenceFrames.CommonQuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPolygon;

public class QuadrupedPathPreview
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final CommonQuadrupedReferenceFrames referenceFrames;

   private SwingTargetGenerator swingTargetGenerator;
   private QuadrupedSupportPolygon fourFootSupportPolygon = new QuadrupedSupportPolygon();
   private final QuadrupedSupportPolygon updatedSupportPolygon = new QuadrupedSupportPolygon();

   private final YoFrameConvexPolygon2d[] nextTenTripleSupportPolygons = new YoFrameConvexPolygon2d[10];
   private final YoArtifactPolygon[] nextTenTripleSupportArtifactPolygons = new YoArtifactPolygon[10];
      
   private final FramePoint footLocation = new FramePoint(ReferenceFrame.getWorldFrame());

   private final FramePoint desiredPosition = new FramePoint(ReferenceFrame.getWorldFrame());

   public QuadrupedPathPreview(SwingTargetGenerator swingTargetGenerator, CommonQuadrupedReferenceFrames referenceFrames,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.referenceFrames = referenceFrames;
      for (int i = 0; i < nextTenTripleSupportPolygons.length; i++)
      {
         String polygonName = "upcommingTriplePolygon" + i;
         YoFrameConvexPolygon2d yoFrameConvexPolygon2d = new YoFrameConvexPolygon2d(polygonName, "", ReferenceFrame.getWorldFrame(), 3, registry);
         nextTenTripleSupportPolygons[i] = yoFrameConvexPolygon2d;
         float saturation = 0.5f;
         float brightness = 0.5f;
         float hue = (float)(0.1 * i);
         nextTenTripleSupportArtifactPolygons[i] = new YoArtifactPolygon(polygonName, yoFrameConvexPolygon2d, Color.getHSBColor(hue, saturation, brightness), false);
         
         yoGraphicsListRegistry.registerArtifact(polygonName, nextTenTripleSupportArtifactPolygons[i]);
      }
      this.swingTargetGenerator = swingTargetGenerator;
   }

   public void update(RobotQuadrant swingLeg, FrameVector desiredBodyVelocity, double yawRate)
   {
      updateFeetLocations();
      updatedSupportPolygon.set(fourFootSupportPolygon);

      for (int i = 0; i < nextTenTripleSupportPolygons.length; i++)
      {
         
         swingTargetGenerator.getSwingTarget(swingLeg, desiredBodyVelocity, desiredPosition, yawRate);

         //get swing leg support
         QuadrupedSupportPolygon swingLegSupportPolygon = updatedSupportPolygon.deleteLegCopy(swingLeg);

         //get next step in future support
         RobotQuadrant nextRegularGaitSwingQuadrant = swingLeg.getNextRegularGaitSwingQuadrant();
         QuadrupedSupportPolygon nextSwingLegSupportPolygon = updatedSupportPolygon.replaceFootstepCopy(swingLeg, desiredPosition);
         nextSwingLegSupportPolygon.deleteLeg(nextRegularGaitSwingQuadrant);

         //if there's a common draw it
         QuadrupedSupportPolygon shrunkenCommonSupportPolygon = swingLegSupportPolygon.getShrunkenCommonSupportPolygon(nextSwingLegSupportPolygon, swingLeg,
               0.02, 0.02, 0.02);
         if (shrunkenCommonSupportPolygon != null)
         {
            drawSupportPolygon(shrunkenCommonSupportPolygon, i);
         }

         swingLeg = swingLeg.getNextRegularGaitSwingQuadrant();

         updatedSupportPolygon.setFootstep(swingLeg, desiredPosition);
      }
   }

   private void drawSupportPolygon(QuadrupedSupportPolygon supportPolygon, int iteration)
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
      nextTenTripleSupportPolygons[iteration].setConvexPolygon2d(polygon);
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
