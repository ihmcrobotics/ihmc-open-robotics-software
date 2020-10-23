package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import java.awt.Color;

import us.ihmc.euclid.referenceFrame.FrameLine3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DBasics;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class EdgeVisualizer
{
   private static final double LineVizWidth = 0.1;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FrameLine3DBasics tempLineOfRotationInWorld = new FrameLine3D();

   private final YoBoolean visualize;
   private final YoFramePoint2D linePointA;
   private final YoFramePoint2D linePointB;

   public EdgeVisualizer(String prefix, Color color, YoRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      visualize = new YoBoolean(prefix + "_Visualize", registry);
      linePointA = new YoFramePoint2D(prefix + "_FootRotationPointA", worldFrame, registry);
      linePointB = new YoFramePoint2D(prefix + "_FootRotationPointB", worldFrame, registry);

      Artifact lineArtifact = new YoArtifactLineSegment2d(prefix + "_LineOfRotation", linePointA, linePointB, color, 0.005, 0.01);
      graphicsListRegistry.registerArtifact(getClass().getSimpleName(), lineArtifact);
   }

   public void visualize(boolean visualize)
   {
      this.visualize.set(visualize);
   }

   public void reset()
   {
      linePointA.setToNaN();
      linePointB.setToNaN();
   }

   public void updateGraphics(FrameLine2DReadOnly lineOfRotation)
   {
      if (visualize.getBooleanValue())
      {
         tempLineOfRotationInWorld.setToZero(lineOfRotation.getReferenceFrame());
         tempLineOfRotationInWorld.set(lineOfRotation);
         tempLineOfRotationInWorld.changeFrame(ReferenceFrame.getWorldFrame());

         linePointA.set(tempLineOfRotationInWorld.getDirection());
         linePointA.scale(-0.5 * LineVizWidth);
         linePointA.add(tempLineOfRotationInWorld.getPointX(), tempLineOfRotationInWorld.getPointY());

         linePointB.set(tempLineOfRotationInWorld.getDirection());
         linePointB.scale(0.5 * LineVizWidth);
         linePointB.add(tempLineOfRotationInWorld.getPointX(), tempLineOfRotationInWorld.getPointY());
      }
      else
      {
         reset();
      }
   }
}
