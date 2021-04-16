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
import us.ihmc.communication.configuration.NetworkParameters;
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
//   private final GDXROS2PointCloudVisualizer realsenseROS2PointCloudVisualizer;
//   private final ImBoolean realsenseROS2PointCloudChecked = new ImBoolean(false);
   private final GDXROS1PointCloudVisualizer realsenseROS1PointCloudVisualizer;
   private final ImBoolean realsenseROS1PointCloudChecked = new ImBoolean(false);
//   private final GDXROS2PlanarRegionsVisualizer mapSensePlanarRegionsROS2Visualizer;
//   private final ImBoolean realsenseSLAMPlanarRegionsChecked = new ImBoolean(false);
   private final GDXROS2PlanarRegionsVisualizer lidarREAPlanarRegionsVisualizer;
   private final ImBoolean lidarREAPlanarRegionsChecked = new ImBoolean(false);
   private final GDXROS1PlanarRegionsVisualizer mapSensePlanarRegionsVisualizer;
   private final ImBoolean mapSensePlanarRegionsChecked = new ImBoolean(false);

//   private final GDXROS1VideoVisualizer realsenseL515ColorVideo;
   private final ImBoolean realsenseL515ColorVideoChecked = new ImBoolean(false);
   private final GDXROS1VideoVisualizer realsenseL515DepthVideo;
   private final ImBoolean realsenseL515DepthVideoChecked = new ImBoolean(false);

   private RosMainNode ros1Node;

   public ImGuiGDXGlobalVisualizersPanel(DRCRobotModel robotModel, ROS2Node ros2Node)
   {
      syncedRobot = new RemoteSyncedRobotModel(robotModel, ros2Node);

      multisenseLidarScanVisualizer = new GDXROS2PointCloudVisualizer(ros2Node, ROS2Tools.MULTISENSE_LIDAR_SCAN);
//      realsenseROS2PointCloudVisualizer = new GDXROS2PointCloudVisualizer(ros2Node, ROS2Tools.L515_POINT_CLOUD);
      realsenseROS1PointCloudVisualizer = new GDXROS1PointCloudVisualizer(RosTools.L515_POINT_CLOUD,
                                                                          robotModel.getSensorInformation().getSteppingCameraFrame(syncedRobot.getReferenceFrames()),
                                                                          robotModel.getSensorInformation().getSteppingCameraTransform());
//      mapSensePlanarRegionsROS2Visualizer = new GDXROS2PlanarRegionsVisualizer(ros2Node, ROS2Tools.MAPSENSE_REGIONS);
      lidarREAPlanarRegionsVisualizer = new GDXROS2PlanarRegionsVisualizer(ros2Node, ROS2Tools.LIDAR_REA_REGIONS);

      mapSensePlanarRegionsVisualizer = new GDXROS1PlanarRegionsVisualizer(ros2Node, robotModel, RosTools.MAPSENSE_REGIONS);

//      realsenseL515ColorVideo = new GDXROS1VideoVisualizer(ros1Node, RosTools.L515_VIDEO);
      realsenseL515DepthVideo = new GDXROS1VideoVisualizer(RosTools.L515_DEPTH);
   }

   public void create()
   {
      multisenseLidarScanVisualizer.create();
//      realsenseROS2PointCloudVisualizer.create();
      realsenseROS1PointCloudVisualizer.create();

      recreateRos1Node();
   }

   public void recreateRos1Node()
   {
      if (ros1Node != null)
      {
         ros1Node.shutdown();
      }
      ros1Node = RosTools.createRosNode(NetworkParameters.getROSURI(), "global_visualizers");

      if (realsenseROS1PointCloudChecked.get())
      {
         realsenseROS1PointCloudVisualizer.subscribe(ros1Node);
      }
      if (mapSensePlanarRegionsChecked.get())
      {
         mapSensePlanarRegionsVisualizer.subscribe(ros1Node);
      }
      if (realsenseL515DepthVideoChecked.get())
      {
         realsenseL515DepthVideo.subscribe(ros1Node);
      }

      ros1Node.execute();
   }

   public void render()
   {
      if (syncedRobot != null)
         syncedRobot.update();

      ImGui.begin(WINDOW_NAME);

      boolean anyNewROS1Enabled = false;
      boolean anyROS1Changed = false;
      boolean changed;

      ImGui.checkbox("Multisense lidar scan (ROS 2)", multisenseLidarScanChecked);
      multisenseLidarScanVisualizer.setEnabled(multisenseLidarScanChecked.get());
      multisenseLidarScanVisualizer.renderImGuiWidgets();
      ImGui.separator();

      ImGui.checkbox("Lidar REA planar regions (ROS 2)", lidarREAPlanarRegionsChecked);
      lidarREAPlanarRegionsVisualizer.setEnabled(lidarREAPlanarRegionsChecked.get());
      lidarREAPlanarRegionsVisualizer.render();
      ImGui.separator();

//      ImGui.checkbox("Realsense point cloud (ROS 2)", realsenseROS2PointCloudChecked);
//      realsenseROS2PointCloudVisualizer.setEnabled(realsenseROS2PointCloudChecked.get());
      //      realsenseROS2PointCloudVisualizer.renderImGuiWidgets();

      changed = ImGui.checkbox("Realsense L515 point cloud (ROS 1)", realsenseROS1PointCloudChecked);
      anyROS1Changed |= changed;
      anyNewROS1Enabled |= changed && realsenseROS1PointCloudChecked.get();
      realsenseROS1PointCloudVisualizer.setEnabled(realsenseROS1PointCloudChecked.get());
      realsenseROS1PointCloudVisualizer.renderImGuiWidgets();
      if (changed && !realsenseROS1PointCloudChecked.get())
      {
         realsenseROS1PointCloudVisualizer.unsubscribe(ros1Node);
      }
      ImGui.separator();

//      ImGui.checkbox("Realsense SLAM planar regions", realsenseSLAMPlanarRegionsChecked);
//      mapSensePlanarRegionsROS2Visualizer.setEnabled(realsenseSLAMPlanarRegionsChecked.get());
//      mapSensePlanarRegionsROS2Visualizer.render();

      changed = ImGui.checkbox("MapSense planar regions (ROS 1)", mapSensePlanarRegionsChecked);
      anyROS1Changed |= changed;
      anyNewROS1Enabled |= changed && mapSensePlanarRegionsChecked.get();
      mapSensePlanarRegionsVisualizer.setEnabled(mapSensePlanarRegionsChecked.get());
      mapSensePlanarRegionsVisualizer.render();
      if (changed && !mapSensePlanarRegionsChecked.get())
      {
         mapSensePlanarRegionsVisualizer.unsubscribe(ros1Node);
      }
      ImGui.separator();

//      ros1Changed |= ImGui.checkbox("Show L515 Color Video", realsenseL515ColorVideoChecked);
//      realsenseL515ColorVideo.renderWidgets();
      changed = ImGui.checkbox("Show L515 Depth Video (ROS 1)", realsenseL515DepthVideoChecked);
      anyROS1Changed |= changed;
      anyNewROS1Enabled |= changed && realsenseL515DepthVideoChecked.get();
      realsenseL515DepthVideo.renderWidgets();
      realsenseL515DepthVideo.setEnabled(realsenseL515DepthVideoChecked.get());
      if (changed && !realsenseL515DepthVideoChecked.get())
      {
         realsenseL515DepthVideo.unsubscribe(ros1Node);
      }

      ImGui.end();

      if (anyNewROS1Enabled)
         recreateRos1Node();

      multisenseLidarScanVisualizer.updateMesh();
//      realsenseROS2PointCloudVisualizer.updateMesh();
      realsenseROS1PointCloudVisualizer.updateMesh();

//      if (realsenseL515ColorVideoChecked.get())
//         realsenseL515ColorVideo.renderVideo();

      realsenseL515DepthVideo.renderVideo();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      multisenseLidarScanVisualizer.getRenderables(renderables, pool);
//      realsenseROS2PointCloudVisualizer.getRenderables(renderables, pool);
      realsenseROS1PointCloudVisualizer.getRenderables(renderables, pool);
//      mapSensePlanarRegionsROS2Visualizer.getRenderables(renderables, pool);
      lidarREAPlanarRegionsVisualizer.getRenderables(renderables, pool);
      mapSensePlanarRegionsVisualizer.getRenderables(renderables, pool);
   }

   public void destroy()
   {
//      mapSensePlanarRegionsROS2Visualizer.destroy();
      lidarREAPlanarRegionsVisualizer.destroy();
      mapSensePlanarRegionsVisualizer.destroy();
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }
}