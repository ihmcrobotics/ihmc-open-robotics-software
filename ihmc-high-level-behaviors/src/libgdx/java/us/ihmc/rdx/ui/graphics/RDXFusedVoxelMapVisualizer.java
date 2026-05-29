package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImFloat;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDAFusedVoxelMapExtractor;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.robotics.time.TimeTools;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

/**
 * Fuses depth and colour data from both ZED X Mini cameras into a confidence-weighted coloured voxel map.
 */
public class RDXFusedVoxelMapVisualizer extends RDXVisualizer
{
   private final LinkedList<RawImage> steppingDepthHistory      = new LinkedList<>();
   private final LinkedList<RawImage> steppingColorHistory      = new LinkedList<>();
   private final LinkedList<RawImage> experimentalDepthHistory  = new LinkedList<>();
   private final LinkedList<RawImage> experimentalColorHistory  = new LinkedList<>();
   private final Notification newImageNotification = new Notification();

   private CUDAFusedVoxelMapExtractor extractor;

   private RDXColoredWorldPointCloudRenderer renderer;
   private int currentMaxPoints = 0;

   private final ImFloat steppingMinRange     = new ImFloat(0.30f);
   private final ImFloat steppingMaxRange     = new ImFloat(10.0f);
   private final ImFloat experimentalMinRange  = new ImFloat(0.10f);
   private final ImFloat experimentalMaxRange  = new ImFloat(8.0f);

   // stepping (belly, 4 mm): sweet-spot ~3 m, σ = 2 m
   private static final float STEPPING_OPTIMAL_M    = 3.0f;
   private static final float STEPPING_SIGMA_M      = 2.0f;
   // experimental (head, 2.2 mm): sweet-spot ~1.8 m, σ = 1.2 m
   private static final float EXPERIMENTAL_OPTIMAL_M = 1.8f;
   private static final float EXPERIMENTAL_SIGMA_M   = 1.2f;

   private final ImFloat voxelSize = new ImFloat(0.05f);
   private static final int VOXEL_KEY_OFFSET = 1 << 19;

   private final ImFloat historyLengthS = new ImFloat(1.0f);

   public RDXFusedVoxelMapVisualizer(String title)
   {
      super(title);
      setSceneLevels(RDXSceneLevel.MODEL);
   }

   public void setSteppingDepthImage(RawImage depthImage)
   {
      if (depthImage.get() != null)
      {
         synchronized (steppingDepthHistory) { steppingDepthHistory.addFirst(depthImage); }
         newImageNotification.set();
      }
   }

   public void setSteppingColorImage(RawImage colorImage)
   {
      if (colorImage.get() != null)
         synchronized (steppingColorHistory) { steppingColorHistory.addFirst(colorImage); }
   }

   public void setExperimentalDepthImage(RawImage depthImage)
   {
      if (depthImage.get() != null)
      {
         synchronized (experimentalDepthHistory) { experimentalDepthHistory.addFirst(depthImage); }
         newImageNotification.set();
      }
   }

   public void setExperimentalColorImage(RawImage colorImage)
   {
      if (colorImage.get() != null)
         synchronized (experimentalColorHistory) { experimentalColorHistory.addFirst(colorImage); }
   }

   @Override
   public void update()
   {
      super.update();
      if (!newImageNotification.poll())
         return;

      RawImage steppingDepth, steppingColor, experimentalDepth, experimentalColor;

      synchronized (steppingDepthHistory)
      {
         if (steppingDepthHistory.isEmpty()) return;
         steppingDepth = steppingDepthHistory.getFirst();
      }
      synchronized (steppingColorHistory)
      {
         if (steppingColorHistory.isEmpty()) return;
         steppingColor = findClosestByTime(steppingDepth, steppingColorHistory);
      }
      synchronized (experimentalDepthHistory)
      {
         if (experimentalDepthHistory.isEmpty()) return;
         experimentalDepth = findClosestByTime(steppingDepth, experimentalDepthHistory);
      }
      synchronized (experimentalColorHistory)
      {
         if (experimentalColorHistory.isEmpty()) return;
         experimentalColor = findClosestByTime(experimentalDepth, experimentalColorHistory);
      }

      if (extractor == null)
         extractor = new CUDAFusedVoxelMapExtractor();

      CUDAFusedVoxelMapExtractor.DualCameraPoints raw =
            extractor.extractPointClouds(steppingDepth, steppingColor,
                                         experimentalDepth, experimentalColor,
                                         steppingMinRange.get(), steppingMaxRange.get(),
                                         experimentalMinRange.get(), experimentalMaxRange.get());

      List<float[]> voxelPoints = new ArrayList<>();
      List<float[]> voxelColors = new ArrayList<>();
      buildVoxelMap(raw, voxelPoints, voxelColors);

      if (voxelPoints.size() > currentMaxPoints)
      {
         if (renderer != null) renderer.dispose();
         currentMaxPoints = Math.max(voxelPoints.size(), 512);
         renderer = new RDXColoredWorldPointCloudRenderer();
         renderer.create(currentMaxPoints);
      }

      if (renderer != null)
         renderer.updateMesh(voxelPoints, voxelColors);

      float hist = historyLengthS.get();
      synchronized (steppingDepthHistory)     { clearExpiredHistory(steppingDepthHistory, hist); }
      synchronized (steppingColorHistory)     { clearExpiredHistory(steppingColorHistory, hist); }
      synchronized (experimentalDepthHistory) { clearExpiredHistory(experimentalDepthHistory, hist); }
      synchronized (experimentalColorHistory) { clearExpiredHistory(experimentalColorHistory, hist); }
   }

   @Override
   public void renderImGuiWidgets()
   {
      ImGui.separator();
      ImGui.sliderFloat(labels.get("Voxel Size (m)"), voxelSize.getData(), 0.02f, 0.20f);
      ImGui.separator();

      ImGui.text("Stepping Camera (belly, 4 mm):");
      ImGui.sliderFloat(labels.get("SC Min Range"), steppingMinRange.getData(), 0.05f, 2.0f);
      ImGui.sliderFloat(labels.get("SC Max Range"), steppingMaxRange.getData(), 1.0f, 15.0f);
      ImGui.separator();

      ImGui.text("Experimental Camera (head, 2.2 mm):");
      ImGui.sliderFloat(labels.get("EC Min Range"), experimentalMinRange.getData(), 0.05f, 2.0f);
      ImGui.sliderFloat(labels.get("EC Max Range"), experimentalMaxRange.getData(), 1.0f, 12.0f);
      ImGui.separator();

      ImGui.sliderFloat(labels.get("Sync History (s)"), historyLengthS.getData(), 0.1f, 3.0f);
      ImGui.text(extractor != null ? "CUDA extractor: running" : "CUDA extractor: waiting for first frame");
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool,
                              Set<RDXSceneLevel> sceneLevels)
   {
      if (renderer != null && sceneLevelCheck(sceneLevels))
         renderer.getRenderables(renderables, pool);
   }

   @Override
   public void destroy()
   {
      super.destroy();
      if (extractor != null) extractor.close();
      if (renderer != null)  renderer.dispose();
      releaseAll(steppingDepthHistory);
      releaseAll(steppingColorHistory);
      releaseAll(experimentalDepthHistory);
      releaseAll(experimentalColorHistory);
   }

   /**
    * Confidence-weighted voxelisation: the point with the highest Gaussian range-confidence wins per voxel.
    */
   private void buildVoxelMap(CUDAFusedVoxelMapExtractor.DualCameraPoints raw,
                               List<float[]> outPoints, List<float[]> outColors)
   {
      // voxelData: key → float[7] = {x, y, z, conf, r, g, b}
      HashMap<Long, float[]> voxelData = new HashMap<>();
      float vs = voxelSize.get();

      float ox0 = raw.steppingOriginX(), oy0 = raw.steppingOriginY(), oz0 = raw.steppingOriginZ();
      List<Point3D32> pts0 = raw.steppingPoints();
      List<float[]>   col0 = raw.steppingColors();

      for (int i = 0; i < pts0.size(); i++)
      {
         Point3D32 p = pts0.get(i);
         float[] c = col0.get(i);
         float dist = dist(p.getX32() - ox0, p.getY32() - oy0, p.getZ32() - oz0);
         float conf = gaussConf(dist, STEPPING_OPTIMAL_M, STEPPING_SIGMA_M);
         insertBest(voxelData, p.getX32(), p.getY32(), p.getZ32(), conf, c[0], c[1], c[2], vs);
      }

      float ox1 = raw.experimentalOriginX(), oy1 = raw.experimentalOriginY(), oz1 = raw.experimentalOriginZ();
      List<Point3D32> pts1 = raw.experimentalPoints();
      List<float[]>   col1 = raw.experimentalColors();

      for (int i = 0; i < pts1.size(); i++)
      {
         Point3D32 p = pts1.get(i);
         float[] c = col1.get(i);
         float dist = dist(p.getX32() - ox1, p.getY32() - oy1, p.getZ32() - oz1);
         float conf = gaussConf(dist, EXPERIMENTAL_OPTIMAL_M, EXPERIMENTAL_SIGMA_M);
         insertBest(voxelData, p.getX32(), p.getY32(), p.getZ32(), conf, c[0], c[1], c[2], vs);
      }

      for (float[] v : voxelData.values())
      {
         outPoints.add(new float[]{v[0], v[1], v[2]});
         outColors.add(new float[]{v[4], v[5], v[6]});
      }
   }

   private void insertBest(HashMap<Long, float[]> map,
                            float x, float y, float z, float conf,
                            float r, float g, float b, float vs)
   {
      long key = voxelKey(x, y, z, vs);
      float[] existing = map.get(key);
      if (existing == null || conf > existing[3])
         map.put(key, new float[]{x, y, z, conf, r, g, b});
   }

   private static float gaussConf(float dist, float optimal, float sigma)
   {
      float d = dist - optimal;
      return (float) Math.exp(-d * d / (2.0f * sigma * sigma));
   }

   private static float dist(float dx, float dy, float dz)
   {
      return (float) Math.sqrt(dx * dx + dy * dy + dz * dz);
   }

   private long voxelKey(float x, float y, float z, float vs)
   {
      int ix = (int) Math.floor(x / vs) + VOXEL_KEY_OFFSET;
      int iy = (int) Math.floor(y / vs) + VOXEL_KEY_OFFSET;
      int iz = (int) Math.floor(z / vs) + VOXEL_KEY_OFFSET;
      return ((long) (ix & 0xFFFFF)) << 40 | ((long) (iy & 0xFFFFF)) << 20 | (iz & 0xFFFFF);
   }

   private static RawImage findClosestByTime(RawImage ref, LinkedList<RawImage> candidates)
   {
      RawImage best = candidates.getFirst();
      double bestDiff = Math.abs(TimeTools.secondsBetween(ref.getAcquisitionTime(),
                                                          best.getAcquisitionTime()));
      for (RawImage c : candidates)
      {
         double diff = Math.abs(TimeTools.secondsBetween(ref.getAcquisitionTime(),
                                                         c.getAcquisitionTime()));
         if (diff < bestDiff) { best = c; bestDiff = diff; }
      }
      return best;
   }

   private static void clearExpiredHistory(LinkedList<RawImage> history, float maxS)
   {
      while (history.size() > 1
             && TimeTools.secondsBetween(history.getLast().getAcquisitionTime(),
                                         history.getFirst().getAcquisitionTime()) > maxS)
         history.removeLast().release();
   }

   private static void releaseAll(LinkedList<RawImage> history)
   {
      synchronized (history) { history.forEach(RawImage::release); history.clear(); }
   }
}
