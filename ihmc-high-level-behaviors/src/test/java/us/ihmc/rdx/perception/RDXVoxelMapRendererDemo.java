package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImInt;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.Loader;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.perception.voxelMap.VoxelMap;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXVoxelMapRenderer;

/**
 * Interactive demo for {@link RDXVoxelMapRenderer}.
 * <p>
 * Opens a 3D window showing a hollow sphere of occupied voxels. The draggable gizmo moves the
 * map origin in world space so the world-to-map transform applied inside the renderer can be
 * visually verified. The sphere radius can be adjusted via the ImGui panel.
 */
public class RDXVoxelMapRendererDemo
{
   static
   {
      Loader.load(opencv_core.class);
   }

   private static final int MAP_SIZE = 40;      // voxels per axis: 40 × 40 × 40
   private static final float VOXEL_SIZE = 0.1f; // 10 cm per voxel → 4 m × 4 m × 4 m map
   private static final int TOTAL_VOXELS = MAP_SIZE * MAP_SIZE * MAP_SIZE;

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final RDXVoxelMapRenderer voxelMapRenderer = new RDXVoxelMapRenderer();
   private RDXPose3DGizmo originGizmo;

   private final ImInt radiusSlider = new ImInt(10);
   // Shared Pose3D: passed by reference to VoxelMap so the gizmo can update it each frame
   private final Pose3D origin = new Pose3D();
   // Initialized in constructor after Loader.load() — FloatPointer requires the native library first
   private VoxelMap voxelMap;

   public RDXVoxelMapRendererDemo()
   {
      FloatPointer cpuData = new FloatPointer(TOTAL_VOXELS);
      voxelMap = new VoxelMap(cpuData, null, MAP_SIZE, MAP_SIZE, MAP_SIZE, VOXEL_SIZE, origin);
      fillSphere(radiusSlider.get());

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().addCoordinateFrame(0.3);

            // Draggable gizmo: move it to verify the world-T-map transform in the renderer
            originGizmo = new RDXPose3DGizmo();
            originGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
            baseUI.getPrimaryScene().addRenderableProvider(originGizmo);

            voxelMapRenderer.create(TOTAL_VOXELS);
            baseUI.getPrimaryScene().addRenderableProvider(voxelMapRenderer);

            baseUI.getImGuiPanelManager().addPanel("Voxel Map", RDXVoxelMapRendererDemo.this::renderImGuiWidgets);
         }

         @Override
         public void render()
         {
            // Mutate the shared Pose3D — VoxelMap.getOrigin() sees the update automatically
            origin.set(originGizmo.getTransformToParent());
            voxelMapRenderer.update(voxelMap);

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            voxelMapRenderer.dispose();
            voxelMap.getCpuData().close();
            baseUI.dispose();
         }
      });
   }

   private void renderImGuiWidgets()
   {
      if (ImGui.sliderInt("Shell radius (voxels)", radiusSlider.getData(), 1, MAP_SIZE / 2 - 1))
         fillSphere(radiusSlider.get());

      ImGui.text(String.format("Map: %d × %d × %d voxels @ %.0f cm each (%.1f m span)",
                               MAP_SIZE, MAP_SIZE, MAP_SIZE, VOXEL_SIZE * 100, MAP_SIZE * VOXEL_SIZE));
   }

   /**
    * Populates the voxel map with a hollow spherical shell of the given radius.
    * The shell is 2 voxels thick so it remains visible from all viewing angles.
    */
   private void fillSphere(int radiusVoxels)
   {
      // Center-anchored: voxel i's local offset is (i - (N-1)/2.0) * voxelSize.
      // For distance computation we only need the dimensionless offset (i - (N-1)/2.0),
      // so the sphere radius is also expressed in voxels here.
      double center = (MAP_SIZE - 1) / 2.0;
      double rInner = radiusVoxels - 1.0;
      double rOuter = radiusVoxels + 1.0;

      FloatPointer cpuData = voxelMap.getCpuData();
      for (int ix = 0; ix < MAP_SIZE; ix++)
      {
         for (int iy = 0; iy < MAP_SIZE; iy++)
         {
            for (int iz = 0; iz < MAP_SIZE; iz++)
            {
               double dx = ix - center, dy = iy - center, dz = iz - center;
               double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
               float value = (dist >= rInner && dist <= rOuter) ? 1.0f : 0.0f;
               cpuData.put((long) (ix * MAP_SIZE + iy) * MAP_SIZE + iz, value);
            }
         }
      }
   }

   public static void main(String[] args)
   {
      new RDXVoxelMapRendererDemo();
   }
}
