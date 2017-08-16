package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import java.awt.Color;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.math.frames.YoFrameLineSegment2d;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;

public class SimpleDesiredHeadingControlModuleVisualizer
{
   private final YoFrameLineSegment2d desiredHeadingLine;
   private final YoFrameLineSegment2d finalHeadingLine;

   private final ProcessedSensorsInterface processedSensors;

   public SimpleDesiredHeadingControlModuleVisualizer(ProcessedSensorsInterface processedSensors, YoVariableRegistry registry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.processedSensors = processedSensors;

      desiredHeadingLine = new YoFrameLineSegment2d("desiredHeadingLine", "", ReferenceFrame.getWorldFrame(), registry);
      finalHeadingLine = new YoFrameLineSegment2d("finalHeadingLine", "", ReferenceFrame.getWorldFrame(), registry);

      if (yoGraphicsListRegistry != null)
      {
         ArtifactList artifactList = new ArtifactList("Simple Desired Heading");

         YoArtifactLineSegment2d yoFrameLineSegment2dArtifact = new YoArtifactLineSegment2d("Desired Heading Line", desiredHeadingLine, Color.MAGENTA);
         artifactList.add(yoFrameLineSegment2dArtifact);

         yoFrameLineSegment2dArtifact = new YoArtifactLineSegment2d("Final Heading Line", finalHeadingLine, Color.ORANGE);
         artifactList.add(yoFrameLineSegment2dArtifact);

         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }
   }

   public void updateDesiredHeading(double desiredHeading, double desiredHeadingFinal)
   {
      updatedDesiredHeadingLine(desiredHeading);
      updatedFinalHeadingLine(desiredHeadingFinal);
   }

   private void updatedDesiredHeadingLine(double desiredHeading)
   {
      FramePoint2D endpoint1 = new FramePoint2D(processedSensors.getCenterOfMassGroundProjectionInFrame(ReferenceFrame.getWorldFrame()));
      FramePoint2D endpoint2 = new FramePoint2D(endpoint1);
      double length = 1.0;
      endpoint2.setX(endpoint2.getX() + length * Math.cos(desiredHeading));
      endpoint2.setY(endpoint2.getY() + length * Math.sin(desiredHeading));

      FrameLineSegment2d frameLineSegment2d = new FrameLineSegment2d(endpoint1, endpoint2);
      desiredHeadingLine.setFrameLineSegment2d(frameLineSegment2d);
   }

   private void updatedFinalHeadingLine(double desiredHeadingFinal)
   {
      FramePoint2D endpoint1 = new FramePoint2D(processedSensors.getCenterOfMassGroundProjectionInFrame(ReferenceFrame.getWorldFrame()));
      FramePoint2D endpoint2 = new FramePoint2D(endpoint1);
      double length = 1.0;
      endpoint2.setX(endpoint2.getX() + length * Math.cos(desiredHeadingFinal));
      endpoint2.setY(endpoint2.getY() + length * Math.sin(desiredHeadingFinal));

      FrameLineSegment2d frameLineSegment2d = new FrameLineSegment2d(endpoint1, endpoint2);
      finalHeadingLine.setFrameLineSegment2d(frameLineSegment2d);
   }
}
