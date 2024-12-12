package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import us.ihmc.commons.thread.Notification;
import us.ihmc.perception.RawImage;
import us.ihmc.rdx.AbstractRDXPointCloudRenderer.ColoringMethod;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.robotics.time.TimeTools;

import java.time.Duration;
import java.time.Instant;
import java.util.Arrays;
import java.util.Deque;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Set;

public class RDXRawImagePointCloudVisualizer extends RDXVisualizer
{
   // The first element in the history is the newest, and the last is the oldest
   private final Deque<RawImage> depthImageHistory = new LinkedList<>();
   private final Deque<RawImage> colorImageHistory = new LinkedList<>();
   private Duration maxHistoryDuration = Duration.ofSeconds(1L);
   private final Notification newImageNotification = new Notification();

   private RDXRawImagePointCloudRenderer pointCloudRenderer;
   private int maxPoints = 0;
   private String[] availableColoringMethods = new String[0];

   private final ImInt coloringMethod = new ImInt();
   private final float[] defaultColor = new float[] {1.0f, 1.0f, 1.0f};
   private final ImFloat maxImageAgeDifference = new ImFloat(1.0f);

   public RDXRawImagePointCloudVisualizer(String title)
   {
      super(title);
   }

   public void setDepthImage(RawImage depthImage)
   {
      RawImage image = depthImage.get();
      if (image != null)
         depthImageHistory.addFirst(depthImage);

      newImageNotification.set();
   }

   public void setColorImage(RawImage colorImage)
   {
      RawImage image = colorImage.get();
      if (image != null)
         colorImageHistory.addFirst(colorImage);

      newImageNotification.set();
   }

   private static void updateHistory(Deque<RawImage> imageHistory, Duration maxDuration)
   {
      while (Duration.between(imageHistory.getLast().getAcquisitionTime(), imageHistory.getFirst().getAcquisitionTime()).compareTo(maxDuration) > 0)
      {
         imageHistory.removeLast().release();
      }
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

      RawImage depthImage;
      RawImage colorImage;

      if (colorImageHistory.isEmpty())
      {
         depthImage = depthImageHistory.getFirst();
         colorImage = null;
      }
      else
      {
         depthImage = depthImageHistory.getFirst();
         colorImage = colorImageHistory.getFirst();

         if (depthImage.getAcquisitionTime().isBefore(colorImage.getAcquisitionTime()))
            colorImage = findClosest(colorImageHistory, depthImage.getAcquisitionTime());
         else
            depthImage = findClosest(depthImageHistory, colorImage.getAcquisitionTime());

         if (!isWithinDifferenceTolerance(depthImage.getAcquisitionTime(), colorImage.getAcquisitionTime(), maxImageAgeDifference.get()))
            colorImage = null;
      }

      int size = depthImage.getWidth() * depthImage.getHeight();
      if (maxPoints < size)
      {
         maxPoints = size;

         if (pointCloudRenderer != null)
            pointCloudRenderer.dispose();
         pointCloudRenderer = new RDXRawImagePointCloudRenderer(true);
         pointCloudRenderer.create(maxPoints);
         availableColoringMethods = Arrays.stream(pointCloudRenderer.getAvailableColoringMethods()).map(Enum::name).toArray(String[]::new);
      }

      if (colorImage == null)
      {
         pointCloudRenderer.updateMesh(depthImage);
      }
      else
      {
         pointCloudRenderer.updateMesh(depthImage, colorImage);
      }

      updateHistory(depthImageHistory, maxHistoryDuration);
      updateHistory(colorImageHistory, maxHistoryDuration);
   }

   private RawImage findClosest(Deque<RawImage> imageHistory, Instant targetInstant)
   {
      Iterator<RawImage> historyIterator = imageHistory.iterator();
      RawImage closestAfter = null;
      RawImage closestBefore = null;
      while (historyIterator.hasNext())
      {
         closestAfter = closestBefore;
         closestBefore = historyIterator.next();
         if (closestBefore.getAcquisitionTime().isBefore(targetInstant))
            break;
      }

      if (closestAfter == null)
         return closestBefore;

      double secondsBetweenTargetAndAfter = Math.abs(TimeTools.secondsBetween(closestAfter.getAcquisitionTime(), targetInstant));
      double secondsBetweenTargetAndBefore = Math.abs(TimeTools.secondsBetween(closestBefore.getAcquisitionTime(), targetInstant));

      return secondsBetweenTargetAndAfter < secondsBetweenTargetAndBefore ? closestAfter : closestBefore;
   }

   private boolean isWithinDifferenceTolerance(Instant instantA, Instant instantB, double maxDifference)
   {
      return Math.abs(TimeTools.secondsBetween(instantA, instantB)) < maxDifference;
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (ImGui.combo("Coloring Method", coloringMethod, availableColoringMethods))
         pointCloudRenderer.setColoringMethod(ColoringMethod.values()[coloringMethod.get()]);
      if (ImGui.colorEdit3("Default Color", defaultColor))
         pointCloudRenderer.setDefaultPointColor(new Color(defaultColor[0], defaultColor[1], defaultColor[2], 1.0f));

      ImGui.sliderFloat("Max Image Age Difference", maxImageAgeDifference.getData(), 0.0f, 30.0f);

   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (pointCloudRenderer != null)
         pointCloudRenderer.getRenderables(renderables, pool);
   }

   @Override
   public void destroy()
   {
      super.destroy();

      depthImageHistory.forEach(RawImage::release);
      colorImageHistory.forEach(RawImage::release);

      if (pointCloudRenderer != null)
         pointCloudRenderer.dispose();
   }
}
