package us.ihmc.gdx.ui;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.internal.ImGui;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.ui.graphics.live.GDXROS1PointCloudVisualizer;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;

public class GDXROS1PointCloudViewerUI
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/libgdx/resources",
                                                              "ROS 1 Point Cloud Viewer");

   public GDXROS1PointCloudViewerUI()
   {
      RosMainNode ros1Node = RosTools.createRosNode(NetworkParameters.getROSURI(), "ros1_point_cloud_viewer");

      GDXROS1PointCloudVisualizer ros1PointCloudVisualizer = new GDXROS1PointCloudVisualizer("Point cloud", "/multisense/lidar_points2_color");
      ros1PointCloudVisualizer.subscribe(ros1Node);
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(GDXModelBuilder.createCoordinateFrame(0.3)));
            ros1PointCloudVisualizer.create();
            ros1PointCloudVisualizer.setActive(true);
            baseUI.getPrimaryScene().addRenderableProvider(ros1PointCloudVisualizer);

            ros1Node.execute();
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();

            ImGui.begin("Stats");
            ros1PointCloudVisualizer.renderImGuiWidgets();
            ImGui.end();

            ros1PointCloudVisualizer.updateMesh();

            baseUI.renderEnd();
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXROS1PointCloudViewerUI();
   }
}
