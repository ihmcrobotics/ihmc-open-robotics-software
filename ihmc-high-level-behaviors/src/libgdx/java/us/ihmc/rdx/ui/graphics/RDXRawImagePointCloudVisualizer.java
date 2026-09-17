package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.commons.thread.Notification;
import us.ihmc.perception.RawImage;
import us.ihmc.rdx.AbstractRDXPointCloudRenderer.ColoringMethod;
import us.ihmc.rdx.imgui.ImGuiExpandCollapseRenderer;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.robotics.time.TimeTools;

import java.time.Instant;
import java.util.Arrays;
import java.util.Deque;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

/**
 * Visualizer for visualizing point cloud from depth and color images.
 * Attempts to compensate for de-synchronization of color and depth
 * using the image acquisition times.
 */
public class RDXRawImagePointCloudVisualizer extends RDXVisualizer
{
   // The first element in the history is the newest, and the last is the oldest
   private final LinkedList<RawImage> depthImageHistory = new LinkedList<>();
   private final LinkedList<RawImage> colorImageHistory = new LinkedList<>();
   private final Notification newImageNotification = new Notification();

   private RDXRawImagePointCloudRenderer pointCloudRenderer;
   private int maxPoints = 0;
   private String[] availableColoringMethods = new String[0];

   private final ImInt coloringMethod = new ImInt(ColoringMethod.COLOR_IMAGE.ordinal());
   private final float[] defaultColor = new float[] {1.0f, 1.0f, 1.0f};
   private final ImFloat pointScale = new ImFloat(1.5f);

   private final  ImGuiPlot deSyncPlot = new ImGuiPlot("Depth Color De-Sync", 100, 230, 22);
   private double colorDepthDeSync = Double.NaN;

   private final ImGuiExpandCollapseRenderer expandCollapseRenderer = new ImGuiExpandCollapseRenderer();
   private boolean showExtraOptions = false;

   /**
    * The duration of history kept in the image histories.
    * Affects how much delay difference the visualizer can compensate for.
    * For example, if the color images have one second of delay compared to depth,
    * at least one second of history is required to sync the color and depth.
    */
   private final ImFloat historyLength = new ImFloat(0.3f);

   /**
    * Maximum de-synchronization allowed between color and depth images,
    * after attempting to compensate using the image history.
    * If the newest depth image and the oldest color image are
    * de-synchronized by more than  {@code maxDeSync} seconds,
    * the visualizer will default to rendering depth with gradient colors.
    * This value should not be more than half the history length.
    */
   private final ImFloat maxDeSync = new ImFloat(0.1f);

   private final boolean enableColorImageRendering;
   private boolean switchBackToColor = false;

   public RDXRawImagePointCloudVisualizer(String title)
   {
      this(title, true);
   }

   public RDXRawImagePointCloudVisualizer(String title, boolean enableColorImageRendering)
   {
      super(title);
      this.enableColorImageRendering = enableColorImageRendering;

      setSceneLevels(RDXSceneLevel.MODEL);
   }

   public void setDepthImage(RawImage depthImage)
   {
      RawImage image = depthImage.get();

      synchronized (depthImageHistory)
      {
         if (image != null)
            depthImageHistory.addFirst(depthImage);
      }

      newImageNotification.set();
   }

   public void setColorImage(RawImage colorImage)
   {
      RawImage image = colorImage.get();

      synchronized (colorImageHistory)
      {
         if (image != null)
            colorImageHistory.addFirst(colorImage);
      }

      newImageNotification.set();
   }

   @Override
   public void update()
   {
      super.update();

      // Ensure we got something new to render
      if (!newImageNotification.poll())
         return;

      // Ensure we got at least one depth image
      if (depthImageHistory.isEmpty())
         return;

      RawImage depthImage = null;
      RawImage colorImage = null;
      boolean foundMatchingColorImage = false;

      // We try to match the color and depth images according to acquisition time
      if (enableColorImageRendering && !colorImageHistory.isEmpty() && (coloringMethod.get() == ColoringMethod.COLOR_IMAGE.ordinal() || switchBackToColor))
      {
         depthImage = depthImageHistory.getFirst();
         colorImage = colorImageHistory.getFirst();
         colorDepthDeSync = Math.abs(TimeTools.secondsBetween(colorImage.getAcquisitionTime(), depthImage.getAcquisitionTime()));

         if (depthImage.getAcquisitionTime().isBefore(colorImage.getAcquisitionTime()))
         {
            synchronized (colorImageHistory)
            {  // Newest depth image is older than the newest color image, so we find an older color image
               colorImage = findClosest(depthImage, colorImageHistory);
            }
         }
         else
         {
            synchronized (depthImageHistory)
            {  // Newest depth image is newer than the newest color image, so we find an older depth image
               depthImage = findClosest(colorImage, depthImageHistory);
            }
         }

         // If the closest images are too far apart, we default to rendering the newest depth image without color
         double secondsBetweenImages = Math.abs(TimeTools.secondsBetween(depthImage.getAcquisitionTime(), colorImage.getAcquisitionTime()));
         foundMatchingColorImage = secondsBetweenImages < maxDeSync.get();
      }

      // If no matching color image was found, use the newest depth image and set coloring method to gradient
      if (!foundMatchingColorImage)
      {
         depthImage = depthImageHistory.getFirst();
         colorImage = null;

         if (!switchBackToColor && coloringMethod.get() == ColoringMethod.COLOR_IMAGE.ordinal())
         {
            coloringMethod.set(ColoringMethod.GRADIENT_WORLD_Z.ordinal());
            switchBackToColor = true;
         }
      }
      else if (switchBackToColor)
      {
         coloringMethod.set(ColoringMethod.COLOR_IMAGE.ordinal());
         switchBackToColor = false;
      }

      // Ensure the renderer is initialized with a large enough max size
      int size = depthImage.getWidth() * depthImage.getHeight();
      if (maxPoints < size)
      {
         maxPoints = size;

         if (pointCloudRenderer != null)
            pointCloudRenderer.dispose();

         boolean renderColorizedDepth = depthImage.getOpenCVType() == opencv_core.CV_8UC3;
         pointCloudRenderer = new RDXRawImagePointCloudRenderer(enableColorImageRendering, renderColorizedDepth);
         pointCloudRenderer.create(maxPoints);
         availableColoringMethods = Arrays.stream(pointCloudRenderer.getAvailableColoringMethods()).map(Enum::name).toArray(String[]::new);
      }

      // Update the render settings
      pointCloudRenderer.setPointScale(pointScale.get() / depthImage.getFocalLengthX());
      pointCloudRenderer.setDefaultPointColor(defaultColor[0], defaultColor[1], defaultColor[2], 1.0f);
      pointCloudRenderer.setColoringMethod(ColoringMethod.values()[coloringMethod.get()]);

      // Update the mesh
      if (colorImage == null)
         pointCloudRenderer.updateMesh(depthImage);
      else
         pointCloudRenderer.updateMesh(depthImage, colorImage);

      // Remove images that are too old from history
      synchronized (depthImageHistory)
      {
         clearExpiredHistory(depthImageHistory, historyLength.get());
      }
      synchronized (colorImageHistory)
      {
         clearExpiredHistory(colorImageHistory, historyLength.get());
      }
   }

   private RawImage findClosest(RawImage targetImage, List<RawImage> imageHistory)
   {
      List<Instant> imageAcquisitionTimes = imageHistory.stream().map(RawImage::getAcquisitionTime).toList();
      int closestColorImageIndex = TimeTools.findClosestTimeIndex(targetImage.getAcquisitionTime(), imageAcquisitionTimes);
      return imageHistory.get(closestColorImageIndex);
   }

   private static void clearExpiredHistory(Deque<RawImage> imageHistory, double maxDuration)
   {
      while (!imageHistory.isEmpty()
             && TimeTools.secondsBetween(imageHistory.getLast().getAcquisitionTime(), imageHistory.getFirst().getAcquisitionTime()) > maxDuration)
      {
         imageHistory.removeLast().release();
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (enableColorImageRendering)
      {
         // Render the de-sync plot
         deSyncPlot.setYScale(0, historyLength.get());
         deSyncPlot.setWidth((int) (0.65f * ImGui.getWindowWidth()));
         ImGui.pushStyleColor(ImGuiCol.PlotLines, ImGuiTools.greenRedGradientColor(colorDepthDeSync, 0.0f, historyLength.get()));
         deSyncPlot.render(colorDepthDeSync);
         ImGui.popStyleColor();
      }

      // Render the renderer options
      if (ImGui.combo("Coloring Method", coloringMethod, availableColoringMethods))
         switchBackToColor = false;
      ImGui.colorEdit3("Default Color", defaultColor);
      ImGui.sliderFloat("Point Scale", pointScale.getData(), 0.0f, 2.0f);

      // Render extra options (if expanded)
      if (enableColorImageRendering && expandCollapseRenderer.render(showExtraOptions))
         showExtraOptions = !showExtraOptions;
      ImGui.sameLine();
      ImGui.text((showExtraOptions ? "Hide" : "Show") + " Extra Options");
      if (showExtraOptions)
      {
         if (ImGui.sliderFloat("History Length (S)", historyLength.getData(), 0.0f, 3.0f))
         {
            if (maxDeSync.get() > 0.5f * historyLength.get())
               maxDeSync.set(0.5f * historyLength.get());
         }
         ImGuiTools.previousWidgetTooltip("Affects how much de-sync can be corrected.");

         ImGui.sliderFloat("Max De-Sync (S)", maxDeSync.getData(), 0.0f, 0.5f * historyLength.get());
         ImGuiTools.previousWidgetTooltip("Amount of de-sync allowed after correction.");
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (pointCloudRenderer != null && sceneLevelCheck(sceneLevels))
         pointCloudRenderer.getRenderables(renderables, pool);
   }

   @Override
   public void destroy()
   {
      super.destroy();

      synchronized (depthImageHistory)
      {
         depthImageHistory.forEach(RawImage::release);
      }
      synchronized (colorImageHistory)
      {
         colorImageHistory.forEach(RawImage::release);
      }

      if (pointCloudRenderer != null)
         pointCloudRenderer.dispose();
   }
}
