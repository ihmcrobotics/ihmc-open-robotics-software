package us.ihmc.gdx.perception;

import imgui.ImGui;
import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtractionComms;
import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtractionParameters;
import us.ihmc.communication.property.StoredPropertySetMessageTools;
import us.ihmc.communication.property.StoredPropertySetROS2Input;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.gdx.ui.graphics.live.GDXROS1VideoVisualizer;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.utilities.ros.ROS1Helper;

public class GDXRemoteGPUPlanarRegionExtractionUI
{
   private final ROS1Helper ros1Helper;
   private final ROS2Helper ros2Helper;
   private final GPUPlanarRegionExtractionParameters gpuRegionParameters = new GPUPlanarRegionExtractionParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private GDXROS1VideoVisualizer debugExtractionPanel;
   private final ImGuiPanel panel = new ImGuiPanel("GPU Planar Regions", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final StoredPropertySetROS2Input gpuRegionParametersROS2Input;
   private final StoredPropertySetROS2Input polygonizerParametersROS2Input;
   private final StoredPropertySetROS2Input concaveHullFactoryParametersROS2Input;
   private final ImGuiStoredPropertySetTuner gpuRegionParametersTuner;
   private final ImGuiStoredPropertySetTuner concaveHullFactoryParametersTuner;
   private final ImGuiStoredPropertySetTuner polygonizerParametersTuner;
   private boolean gpuRegionParametersChangedByImGuiUser = false;
   private boolean concaveHullFactoryParametersChangedByImGuiUser = false;
   private boolean polygonizerParametersChangedByImGuiUser = false;

   public GDXRemoteGPUPlanarRegionExtractionUI(ROS1Helper ros1Helper, ROS2Helper ros2Helper)
   {
      this.ros1Helper = ros1Helper;
      this.ros2Helper = ros2Helper;

      if (ros1Helper != null)
      {
         debugExtractionPanel = new GDXROS1VideoVisualizer("GPU Planar Regions Debug Image", GPUPlanarRegionExtractionComms.DEBUG_EXTRACTION_IMAGE, true);
         debugExtractionPanel.create();
         panel.addChild(debugExtractionPanel.getPanel());
      }

      gpuRegionParametersROS2Input = new StoredPropertySetROS2Input(ros2Helper, GPUPlanarRegionExtractionComms.PARAMETERS_OUTPUT, gpuRegionParameters);
      polygonizerParametersROS2Input = new StoredPropertySetROS2Input(ros2Helper,
                                                                      GPUPlanarRegionExtractionComms.POLYGONIZER_PARAMETERS_OUTPUT,
                                                                      polygonizerParameters);
      concaveHullFactoryParametersROS2Input = new StoredPropertySetROS2Input(ros2Helper,
                                                                             GPUPlanarRegionExtractionComms.CONVEX_HULL_FACTORY_PARAMETERS_OUTPUT,
                                                                             concaveHullFactoryParameters);
      gpuRegionParametersTuner = new ImGuiStoredPropertySetTuner(gpuRegionParameters.getTitle());
      gpuRegionParametersTuner.create(gpuRegionParameters, () -> gpuRegionParametersChangedByImGuiUser = true);
      concaveHullFactoryParametersTuner = new ImGuiStoredPropertySetTuner(concaveHullFactoryParameters.getTitle());
      concaveHullFactoryParametersTuner.create(concaveHullFactoryParameters, () -> concaveHullFactoryParametersChangedByImGuiUser = true);
      polygonizerParametersTuner = new ImGuiStoredPropertySetTuner(polygonizerParameters.getTitle());
      polygonizerParametersTuner.create(polygonizerParameters, () -> polygonizerParametersChangedByImGuiUser = true);
   }

   public void update()
   {
      if (ros1Helper != null)
      {
         debugExtractionPanel.updateSubscribers(ros1Helper);
         debugExtractionPanel.update();
      }
   }

   public void renderImGuiWidgets()
   {
      if (ros1Helper != null)
         debugExtractionPanel.renderImGuiWidgets();

      if (ImGui.button("Update parameters from remote"))
      {
         gpuRegionParametersROS2Input.setToAcceptUpdate();
         polygonizerParametersROS2Input.setToAcceptUpdate();
         concaveHullFactoryParametersROS2Input.setToAcceptUpdate();
      }

      if (ImGui.button("Reconnect remote ROS 1 node"))
      {
         ros2Helper.publish(GPUPlanarRegionExtractionComms.RECONNECT_ROS1_NODE);
      }

      ImGui.separator();

      gpuRegionParametersROS2Input.update();

      ImGui.text("GPU Planar Regions Parameters");
      if (gpuRegionParametersROS2Input.getWaitingForUpdate())
      {
         ImGui.text("Waiting for updated values from remote...");
      }
      else
      {
         renderGPUParameterWidgets();
      }

      ImGui.separator();

      polygonizerParametersROS2Input.update();

      ImGui.text("Polygonizer Parameters");
      if (polygonizerParametersROS2Input.getWaitingForUpdate())
      {
         ImGui.text("Waiting for initial values from remote...");
      }
      else
      {
         renderPolygonizerParameterWidgets();
      }

      ImGui.separator();

      concaveHullFactoryParametersROS2Input.update();

      ImGui.text("Concave Hull Factory Parameters");
      if (concaveHullFactoryParametersROS2Input.getWaitingForUpdate())
      {
         ImGui.text("Waiting for initial values from remote...");
      }
      else
      {
         renderConcaveHullFactoryParameterWidgets();
      }
   }

   private void renderGPUParameterWidgets()
   {
      gpuRegionParametersTuner.renderImGuiWidgets();

      if (gpuRegionParametersChangedByImGuiUser)
      {
         gpuRegionParametersChangedByImGuiUser = false;
         ros2Helper.publish(GPUPlanarRegionExtractionComms.PARAMETERS_INPUT, StoredPropertySetMessageTools.newMessage(gpuRegionParameters));
      }
   }

   private void renderPolygonizerParameterWidgets()
   {
      polygonizerParametersTuner.renderImGuiWidgets();

      if (polygonizerParametersChangedByImGuiUser)
      {
         polygonizerParametersChangedByImGuiUser = false;
         ros2Helper.publish(GPUPlanarRegionExtractionComms.POLYGONIZER_PARAMETERS_INPUT, StoredPropertySetMessageTools.newMessage(polygonizerParameters));
      }
   }

   private void renderConcaveHullFactoryParameterWidgets()
   {
      concaveHullFactoryParametersTuner.renderImGuiWidgets();

      if (concaveHullFactoryParametersChangedByImGuiUser)
      {
         concaveHullFactoryParametersChangedByImGuiUser = false;
         ros2Helper.publish(GPUPlanarRegionExtractionComms.CONVEX_HULL_FACTORY_PARAMETERS_INPUT,
                            StoredPropertySetMessageTools.newMessage(concaveHullFactoryParameters));
      }
   }

   public void destroy()
   {
      if (debugExtractionPanel != null)
         debugExtractionPanel.destroy();
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}
