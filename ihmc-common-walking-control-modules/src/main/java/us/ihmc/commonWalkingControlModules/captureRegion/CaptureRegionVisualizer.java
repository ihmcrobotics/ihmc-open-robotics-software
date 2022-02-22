package us.ihmc.commonWalkingControlModules.captureRegion;

import java.awt.Color;
import java.util.function.IntFunction;
import java.util.function.Supplier;

import us.ihmc.communication.net.ObjectProducer;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CaptureRegionVisualizer
{
   private static final String caption = "CaptureRegion";
   private static final Color color = Color.GREEN;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final YoFrameConvexPolygon2D yoCaptureRegionPolygon;
   private final FrameConvexPolygon2D captureRegionPolygon = new FrameConvexPolygon2D();
   private final Supplier<FrameConvexPolygon2DReadOnly> captureRegionProvider;

   public CaptureRegionVisualizer(Supplier<FrameConvexPolygon2DReadOnly> captureRegionProvider, YoGraphicsListRegistry yoGraphicsListRegistry,
                                  YoRegistry parentRegistry)
   {
      this(captureRegionProvider, "", yoGraphicsListRegistry, parentRegistry);
   }

   public CaptureRegionVisualizer(Supplier<FrameConvexPolygon2DReadOnly> captureRegionProvider, String suffix, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoRegistry parentRegistry)
   {
      this.captureRegionProvider = captureRegionProvider;

      yoCaptureRegionPolygon = new YoFrameConvexPolygon2D(caption, suffix, worldFrame, 30, registry);

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
      captureRegionPolygon.setIncludingFrame(captureRegionProvider.get());
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
