package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImFloat;
import us.ihmc.perception.DualCameraVoxelGridThread;
import us.ihmc.perception.cuda.CUDAGPUVoxelGrid;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;

import java.util.Set;

/**
 * In-process visualisation of the entire fused log-odds voxel grid (NOT the RL crop): every occupied
 * voxel of the world grid, coloured by occupancy probability. Reads the whole-grid snapshot directly
 * from {@link DualCameraVoxelGridThread} (same JVM), so it is independent of the RL-observation crop
 * and the raw camera point clouds, each of which is its own toggleable visualizer.
 *
 * <p>A probability-threshold slider hides low-confidence voxels: only voxels with occupancy
 * probability ≥ the threshold are drawn.
 */
public class RDXVoxelMapVisualizer extends RDXVisualizer
{
   /** Cap on occupied voxels drawn per frame. */
   private static final int MAX_VIZ_VOXELS = 500_000;

   private final DualCameraVoxelGridThread voxelGridThread;
   private final RDXColoredWorldPointCloudRenderer renderer = new RDXColoredWorldPointCloudRenderer();
   private boolean rendererCreated = false;

   private final ImFloat probabilityThreshold = new ImFloat(0.5f);
   private final float[] filteredPositions = new float[3 * MAX_VIZ_VOXELS];
   private final float[] filteredColors = new float[3 * MAX_VIZ_VOXELS];
   private int drawnVoxelCount = 0;

   public RDXVoxelMapVisualizer(String title, DualCameraVoxelGridThread voxelGridThread)
   {
      super(title);
      this.voxelGridThread = voxelGridThread;
      setSceneLevels(RDXSceneLevel.MODEL);
   }

   @Override
   public void create()
   {
      super.create();
      renderer.create(MAX_VIZ_VOXELS);
      rendererCreated = true;
   }

   @Override
   public void update()
   {
      super.update();
      if (!rendererCreated)
         return;

      // Only pay the per-frame whole-grid readback while this visualizer is active.
      voxelGridThread.setReadbackOccupiedVoxelsForViz(isActive());
      if (!isActive())
         return;

      CUDAGPUVoxelGrid.OccupiedVoxels voxels = voxelGridThread.getLatestOccupiedVoxels();
      float[] positions = voxels.positions();
      float[] probabilities = voxels.probabilities();
      int available = Math.min(voxels.count(), MAX_VIZ_VOXELS);
      float threshold = probabilityThreshold.get();

      int drawn = 0;
      for (int i = 0; i < available; i++)
      {
         if (probabilities[i] < threshold)
            continue; // hide voxels below the selected probability

         int src = 3 * i;
         int dst = 3 * drawn;
         filteredPositions[dst]     = positions[src];
         filteredPositions[dst + 1] = positions[src + 1];
         filteredPositions[dst + 2] = positions[src + 2];

         // Probability -> colour: at threshold (least confident shown) blue, confident (->1) red.
         float t = Math.min(Math.max((probabilities[i] - 0.5f) / 0.4f, 0.0f), 1.0f);
         filteredColors[dst]     = t;
         filteredColors[dst + 1] = 0.2f;
         filteredColors[dst + 2] = 1.0f - t;
         drawn++;
      }
      drawnVoxelCount = drawn;
      renderer.updateMesh(filteredPositions, filteredColors, drawn);
   }

   @Override
   public void renderImGuiWidgets()
   {
      ImGui.sliderFloat(labels.get("Probability Threshold"), probabilityThreshold.getData(), 0.5f, 1.0f);
      ImGui.text("Voxels shown: " + drawnVoxelCount);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (rendererCreated && isActive() && sceneLevelCheck(sceneLevels))
         renderer.getRenderables(renderables, pool);
   }

   @Override
   public void destroy()
   {
      voxelGridThread.setReadbackOccupiedVoxelsForViz(false);
      if (rendererCreated)
         renderer.dispose();
      super.destroy();
   }
}
