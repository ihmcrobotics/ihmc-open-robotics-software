package us.ihmc.gdx.ui;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.ui.graphics.live.*;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;

public class ImGuiGDXGlobalVisualizersPanel implements RenderableProvider
{
   private static final String WINDOW_NAME = "Global Visualizers";

   private final RemoteSyncedRobotModel syncedRobot;

   private final GDXROS2PointCloudVisualizer multisenseLidarScanVisualizer;
   private final ImBoolean multisenseLidarScanChecked = new ImBoolean(false);
   private final GDXROS2PointCloudVisualizer realsensePointCloudVisualizer;
   private final ImBoolean realsensePointCloudChecked = new ImBoolean(false);
   private final GDXROS1PointCloudVisualizer realsenseROS1PointCloudVisualizer;
   private final ImBoolean realsenseROS1PointCloudChecked = new ImBoolean(false);
   private final GDXROS2PlanarRegionsVisualizer mapSensePlanarRegionsROS2Visualizer;
   private final ImBoolean realsenseSLAMPlanarRegionsChecked = new ImBoolean(false);
   private final GDXROS2PlanarRegionsVisualizer lidarREAPlanarRegionsVisualizer;
   private final ImBoolean lidarREAPlanarRegionsChecked = new ImBoolean(false);
   //   private final GDXROS1PlanarRegionsVisualizer mapSensePlanarRegionsVisualizer;
   private final ImBoolean mapSensePlanarRegionsChecked = new ImBoolean(true);

   //   private final GDXROS1VideoVisualizer realsenseL515ColorVideo;
   private final ImBoolean realsenseL515ColorVideoChecked = new ImBoolean(false);
   private final GDXROS1VideoVisualizer realsenseL515DepthVideo;
   private final ImBoolean realsenseL515DepthVideoChecked = new ImBoolean(false);

   public ImGuiGDXGlobalVisualizersPanel(DRCRobotModel robotModel, ROS2Node ros2Node, RosMainNode ros1Node)
   {
      syncedRobot = new RemoteSyncedRobotModel(robotModel, ros2Node);

      multisenseLidarScanVisualizer = new GDXROS2PointCloudVisualizer(ros2Node, ROS2Tools.MULTISENSE_LIDAR_SCAN);
      realsensePointCloudVisualizer = new GDXROS2PointCloudVisualizer(ros2Node, ROS2Tools.L515_POINT_CLOUD);
      realsenseROS1PointCloudVisualizer = new GDXROS1PointCloudVisualizer(ros1Node,
                                                                          RosTools.L515_POINT_CLOUD,
                                                                          robotModel.getSensorInformation().getSteppingCameraFrame(syncedRobot.getReferenceFrames()),
                                                                          robotModel.getSensorInformation().getSteppingCameraTransform());
      mapSensePlanarRegionsROS2Visualizer = new GDXROS2PlanarRegionsVisualizer(ros2Node, ROS2Tools.MAPSENSE_REGIONS);
      lidarREAPlanarRegionsVisualizer = new GDXROS2PlanarRegionsVisualizer(ros2Node, ROS2Tools.LIDAR_REA_REGIONS);

      //      mapSensePlanarRegionsVisualizer = new GDXROS1PlanarRegionsVisualizer(ros1Node, ros2Node, robotModel, RosTools.MAPSENSE_REGIONS);

      //      realsenseL515ColorVideo = new GDXROS1VideoVisualizer(ros1Node, RosTools.L515_VIDEO);
      realsenseL515DepthVideo = new GDXROS1VideoVisualizer(RosTools.L515_DEPTH);
   }

   public void create()
   {
      multisenseLidarScanVisualizer.create();
      realsensePointCloudVisualizer.create();
      realsenseROS1PointCloudVisualizer.create();
   }

   public void render()
   {
      if (syncedRobot != null)
         syncedRobot.update();

      ImGui.begin(WINDOW_NAME);

      ImGui.checkbox("Multisense lidar scan", multisenseLidarScanChecked);
      multisenseLidarScanVisualizer.setEnabled(multisenseLidarScanChecked.get());
      multisenseLidarScanVisualizer.renderImGuiWidgets();

      ImGui.checkbox("Realsense point cloud", realsensePointCloudChecked);
      realsensePointCloudVisualizer.setEnabled(realsensePointCloudChecked.get());
      realsensePointCloudVisualizer.renderImGuiWidgets();

      ImGui.checkbox("Realsense ROS 1 point cloud", realsenseROS1PointCloudChecked);
      realsenseROS1PointCloudVisualizer.setEnabled(realsenseROS1PointCloudChecked.get());
      realsenseROS1PointCloudVisualizer.renderImGuiWidgets();

      ImGui.checkbox("Realsense SLAM planar regions", realsenseSLAMPlanarRegionsChecked);
      mapSensePlanarRegionsROS2Visualizer.setEnabled(realsenseSLAMPlanarRegionsChecked.get());
      mapSensePlanarRegionsROS2Visualizer.render();

      ImGui.checkbox("Lidar REA planar regions", lidarREAPlanarRegionsChecked);
      lidarREAPlanarRegionsVisualizer.setEnabled(lidarREAPlanarRegionsChecked.get());
      lidarREAPlanarRegionsVisualizer.render();

      ImGui.checkbox("MapSense planar regions", mapSensePlanarRegionsChecked);
      //      mapSensePlanarRegionsVisualizer.setEnabled(mapSensePlanarRegionsChecked.get());
      //      mapSensePlanarRegionsVisualizer.render();

      ImGui.checkbox("Show L515 Color Video", realsenseL515ColorVideoChecked);
      //      realsenseL515ColorVideo.renderWidgets();
      ImGui.checkbox("Show L515 Depth Video", realsenseL515DepthVideoChecked);
      realsenseL515DepthVideo.renderWidgets();

      ImGui.end();

      multisenseLidarScanVisualizer.updateMesh();
      realsensePointCloudVisualizer.updateMesh();
      realsenseROS1PointCloudVisualizer.updateMesh();

      //      if (realsenseL515ColorVideoChecked.get())
      //         realsenseL515ColorVideo.renderVideo();
      if (realsenseL515DepthVideoChecked.get())
         realsenseL515DepthVideo.renderVideo();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      multisenseLidarScanVisualizer.getRenderables(renderables, pool);
      realsensePointCloudVisualizer.getRenderables(renderables, pool);
      realsenseROS1PointCloudVisualizer.getRenderables(renderables, pool);
      mapSensePlanarRegionsROS2Visualizer.getRenderables(renderables, pool);
      lidarREAPlanarRegionsVisualizer.getRenderables(renderables, pool);
      //      mapSensePlanarRegionsVisualizer.getRenderables(renderables, pool);
   }

   public void destroy()
   {
      mapSensePlanarRegionsROS2Visualizer.destroy();
      lidarREAPlanarRegionsVisualizer.destroy();
      //      mapSensePlanarRegionsVisualizer.destroy();
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }
}
