package us.ihmc.gdx.perception;

import controller_msgs.msg.dds.StoredPropertySetMessage;
import imgui.ImGui;
import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtractionComms;
import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtractionParameters;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.property.StoredPropertySetMessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.gdx.ui.graphics.live.GDXROS1VideoVisualizer;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.utilities.ros.ROS1Helper;

public class GDXRemoteGPUPlanarRegionExtractionUI
{
   private final GPUPlanarRegionExtractionParameters gpuRegionParameters = new GPUPlanarRegionExtractionParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private final GDXROS1VideoVisualizer debugExtractionPanel = new GDXROS1VideoVisualizer("GPU Planar Regions Debug Image",
                                                                                    GPUPlanarRegionExtractionComms.DEBUG_EXTRACTION_IMAGE);
   private final ImGuiPanel panel = new ImGuiPanel("GPU Planar Regions", this::renderImGuiWidgets);
   private final ImGuiStoredPropertySetTuner gpuPlanarRegionsParametersTuner = new ImGuiStoredPropertySetTuner("GPU Planar Regions Parameters");
   private final ImGuiStoredPropertySetTuner polygonizerParametersTuner = new ImGuiStoredPropertySetTuner("Polygonizer Parameters");
   private final ImGuiStoredPropertySetTuner concaveHullFactoryParametersTuner = new ImGuiStoredPropertySetTuner("Concave Hull Factory Parameters");
//   private final ROS2Input<StoredPropertySetMessage> gpuRegionParametersROS2Input;
//   private final ROS2Input<StoredPropertySetMessage> polygonizerParametersROS2Input;
//   private final ROS2Input<StoredPropertySetMessage> concaveHullFactoryParametersROS2Input;
   private final TypedNotification<StoredPropertySetMessage> gpuRegionParametersROS2Notification = new TypedNotification<>();
   private final TypedNotification<StoredPropertySetMessage> polygonizerParametersROS2Notification = new TypedNotification<>();
   private final TypedNotification<StoredPropertySetMessage> concaveHullFactoryParametersROS2Notification = new TypedNotification<>();
   private boolean gpuRegionParametersHaveBeenReceived = false;
   private boolean polygonizerParametersHaveBeenReceived = false;
   private boolean concaveHullFactoryParametersHaveBeenReceived = false;
   private ROS1Helper ros1Helper;

   public GDXRemoteGPUPlanarRegionExtractionUI(ROS1Helper ros1Helper, ROS2Helper ros2Helper)
   {
      this.ros1Helper = ros1Helper;
      panel.addChild(debugExtractionPanel.getPanel());

      debugExtractionPanel.create();

      gpuPlanarRegionsParametersTuner.create(gpuRegionParameters,
                                             GPUPlanarRegionExtractionParameters.keys,
                                             () -> ros2Helper.publish(GPUPlanarRegionExtractionComms.PARAMETERS_INPUT,
                                                                      StoredPropertySetMessageTools.newMessage(gpuRegionParameters)));
      polygonizerParametersTuner.create(polygonizerParameters,
                                        PolygonizerParameters.keys,
                                        () -> ros2Helper.publish(GPUPlanarRegionExtractionComms.POLYGONIZER_PARAMETERS_INPUT,
                                                                 StoredPropertySetMessageTools.newMessage(polygonizerParameters)));
      concaveHullFactoryParametersTuner.create(concaveHullFactoryParameters,
                                               ConcaveHullFactoryParameters.keys,
                                               () -> ros2Helper.publish(GPUPlanarRegionExtractionComms.CONVEX_HULL_FACTORY_PARAMETERS_INPUT,
                                                                        StoredPropertySetMessageTools.newMessage(concaveHullFactoryParameters)));

      ros2Helper.subscribeViaCallback(GPUPlanarRegionExtractionComms.PARAMETERS_OUTPUT, gpuRegionParametersROS2Notification::set);
      ros2Helper.subscribeViaCallback(GPUPlanarRegionExtractionComms.POLYGONIZER_PARAMETERS_OUTPUT, polygonizerParametersROS2Notification::set);
      ros2Helper.subscribeViaCallback(GPUPlanarRegionExtractionComms.CONVEX_HULL_FACTORY_PARAMETERS_OUTPUT, concaveHullFactoryParametersROS2Notification::set);
//      gpuRegionParametersROS2Input = ros2Helper.subscribe(GPUPlanarRegionExtractionComms.PARAMETERS_OUTPUT);
//      polygonizerParametersROS2Input = ros2Helper.subscribe(GPUPlanarRegionExtractionComms.POLYGONIZER_PARAMETERS_OUTPUT);
//      concaveHullFactoryParametersROS2Input = ros2Helper.subscribe(GPUPlanarRegionExtractionComms.CONVEX_HULL_FACTORY_PARAMETERS_OUTPUT);
   }

   public void update()
   {
      debugExtractionPanel.updateSubscribers(ros1Helper);
      debugExtractionPanel.update();
   }

   public void renderImGuiWidgets()
   {
      debugExtractionPanel.renderImGuiWidgets();

      ImGui.separator();

//      if (!gpuRegionParametersHaveBeenReceived && gpuRegionParametersROS2Input.getMessageNotification().poll())
      if (!gpuRegionParametersHaveBeenReceived && gpuRegionParametersROS2Notification.poll())
      {
         StoredPropertySetMessageTools.copyToStoredPropertySet(gpuRegionParametersROS2Notification.read(),
                                                               gpuRegionParameters,
                                                               () -> LogTools.info("Updating GPU planar regions parameters from remote."));
         gpuRegionParametersHaveBeenReceived = true;
      }

      ImGui.text("GPU Planar Regions Parameters");
      if (gpuRegionParametersHaveBeenReceived)
      {
         gpuPlanarRegionsParametersTuner.renderImGuiWidgets();
      }
      else
      {
         ImGui.text("Waiting for initial values from remote...");
      }

      ImGui.separator();

      if (!polygonizerParametersHaveBeenReceived && polygonizerParametersROS2Notification.poll())
      {
         StoredPropertySetMessageTools.copyToStoredPropertySet(polygonizerParametersROS2Notification.read(),
                                                               polygonizerParameters,
                                                               () -> LogTools.info("Updating polygonizer parameters from remote."));
         polygonizerParametersHaveBeenReceived = true;
      }

      ImGui.text("Polygonizer Parameters");
      if (polygonizerParametersHaveBeenReceived)
      {
         polygonizerParametersTuner.renderImGuiWidgets();
      }
      else
      {
         ImGui.text("Waiting for initial values from remote...");
      }

      ImGui.separator();

      if (!concaveHullFactoryParametersHaveBeenReceived && concaveHullFactoryParametersROS2Notification.poll())
      {
         StoredPropertySetMessageTools.copyToStoredPropertySet(concaveHullFactoryParametersROS2Notification.read(),
                                                               concaveHullFactoryParameters,
                                                               () -> LogTools.info("Updating concave hull factory parameters from remote."));
         concaveHullFactoryParametersHaveBeenReceived = true;
      }

      ImGui.text("Concave Hull Factory Parameters");
      if (concaveHullFactoryParametersHaveBeenReceived)
      {
         concaveHullFactoryParametersTuner.renderImGuiWidgets();
      }
      else
      {
         ImGui.text("Waiting for initial values from remote...");
      }
   }

   public void destroy()
   {
      debugExtractionPanel.destroy();
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}
