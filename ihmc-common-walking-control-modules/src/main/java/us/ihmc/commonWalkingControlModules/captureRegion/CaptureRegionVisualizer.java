package us.ihmc.commonWalkingControlModules.captureRegion;

import java.awt.Color;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CaptureRegionVisualizer
{
   private static final String caption = "CaptureRegion";
   private static final Color color = Color.GREEN;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final YoFrameConvexPolygon2d yoCaptureRegionPolygon;
   private final FrameConvexPolygon2D captureRegionPolygon = new FrameConvexPolygon2D();
   private final OneStepCaptureRegionCalculator captureRegionCalculator;

   public CaptureRegionVisualizer(OneStepCaptureRegionCalculator captureRegionCalculator, YoGraphicsListRegistry yoGraphicsListRegistry,
                                  YoVariableRegistry parentRegistry)
   {
      this(captureRegionCalculator, "", yoGraphicsListRegistry, parentRegistry);
   }

   public CaptureRegionVisualizer(OneStepCaptureRegionCalculator captureRegionCalculator, String suffix, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoVariableRegistry parentRegistry)
   {
      this.captureRegionCalculator = captureRegionCalculator;

      yoCaptureRegionPolygon = new YoFrameConvexPolygon2d(caption, suffix, worldFrame, 30, registry);

      YoArtifactPolygon polygonArtifact = new YoArtifactPolygon(caption + suffix, yoCaptureRegionPolygon, color, false);
      yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), polygonArtifact);

      parentRegistry.addChild(registry);
   }

   public void hide()
   {
      yoCaptureRegionPolygon.clear();
   }

   public void update()
   {
      captureRegionPolygon.setIncludingFrame(captureRegionCalculator.getCaptureRegion());
      captureRegionPolygon.changeFrameAndProjectToXYPlane(worldFrame);

      if (yoCaptureRegionPolygon != null)
      {
         try
         {
            yoCaptureRegionPolygon.set(captureRegionPolygon);
         }
         catch (Exception e)
         {
            System.out.println(e);
         }
      }
   }
}
