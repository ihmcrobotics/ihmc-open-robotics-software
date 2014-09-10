package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

   import java.awt.Color;

import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.yoUtilities.math.frames.YoFrameLineSegment2d;

import com.yobotics.simulationconstructionset.plotting.YoFrameLineSegment2dArtifact;

   public class SimpleDesiredHeadingControlModuleVisualizer
   {     
      private final YoFrameLineSegment2d desiredHeadingLine;
      private final YoFrameLineSegment2d finalHeadingLine;

      private final ProcessedSensorsInterface processedSensors;

      public SimpleDesiredHeadingControlModuleVisualizer(ProcessedSensorsInterface processedSensors, YoVariableRegistry registry, YoGraphicsListRegistry dynamicGraphicObjectsListRegistry)
      {
         this.processedSensors = processedSensors;

         desiredHeadingLine = new YoFrameLineSegment2d("desiredHeadingLine", "", ReferenceFrame.getWorldFrame(), registry);
         finalHeadingLine = new YoFrameLineSegment2d("finalHeadingLine", "", ReferenceFrame.getWorldFrame(), registry);

         if (dynamicGraphicObjectsListRegistry != null)
         {
            ArtifactList artifactList = new ArtifactList("Simple Desired Heading");

            YoFrameLineSegment2dArtifact yoFrameLineSegment2dArtifact = new YoFrameLineSegment2dArtifact("Desired Heading Line", desiredHeadingLine,
                  Color.MAGENTA);
            artifactList.add(yoFrameLineSegment2dArtifact);

            yoFrameLineSegment2dArtifact = new YoFrameLineSegment2dArtifact("Final Heading Line", finalHeadingLine, Color.ORANGE);
            artifactList.add(yoFrameLineSegment2dArtifact);

            dynamicGraphicObjectsListRegistry.registerArtifactList(artifactList);
         }
      }

      public void updateDesiredHeading(double desiredHeading, double desiredHeadingFinal)
      {
         updatedDesiredHeadingLine(desiredHeading);
         updatedFinalHeadingLine(desiredHeadingFinal);
      }

      private void updatedDesiredHeadingLine(double desiredHeading)
      {
         FramePoint2d endpoint1 = processedSensors.getCenterOfMassGroundProjectionInFrame(ReferenceFrame.getWorldFrame()).toFramePoint2d();
         FramePoint2d endpoint2 = new FramePoint2d(endpoint1);
         double length = 1.0;
         endpoint2.setX(endpoint2.getX() + length * Math.cos(desiredHeading));
         endpoint2.setY(endpoint2.getY() + length * Math.sin(desiredHeading));

         FrameLineSegment2d frameLineSegment2d = new FrameLineSegment2d(endpoint1, endpoint2);
         desiredHeadingLine.setFrameLineSegment2d(frameLineSegment2d);
      }

      private void updatedFinalHeadingLine(double desiredHeadingFinal)
      {
         FramePoint2d endpoint1 = processedSensors.getCenterOfMassGroundProjectionInFrame(ReferenceFrame.getWorldFrame()).toFramePoint2d();
         FramePoint2d endpoint2 = new FramePoint2d(endpoint1);
         double length = 1.0;
         endpoint2.setX(endpoint2.getX() + length * Math.cos(desiredHeadingFinal));
         endpoint2.setY(endpoint2.getY() + length * Math.sin(desiredHeadingFinal));

         FrameLineSegment2d frameLineSegment2d = new FrameLineSegment2d(endpoint1, endpoint2);
         finalHeadingLine.setFrameLineSegment2d(frameLineSegment2d);
      }
}
