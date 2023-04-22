package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.behaviors.activeMapping.ActiveMappingModule;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySetGroup;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;

public class RDXActiveMappingUIPanel implements RenderableProvider
{
   public enum Mode
   {
      REMOTE, LOCAL
   }

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RDXPlanarRegionsGraphic mapPlanarRegionsGraphic = new RDXPlanarRegionsGraphic();

   private PerceptionConfigurationParameters perceptionConfigurationParameters;
   private ImGuiRemoteROS2StoredPropertySetGroup remotePropertySets;
   private ActiveMappingModule activeMappingModule;
   private ImGuiPanel imGuiPanel;
   private Mode mode = Mode.LOCAL;

   private final ImBoolean renderEnabled = new ImBoolean(true);

   public RDXActiveMappingUIPanel(String name, ActiveMappingModule mappingManager)
   {
      mode = Mode.LOCAL;
      this.activeMappingModule = mappingManager;
      imGuiPanel = new ImGuiPanel(name, this::renderImGuiWidgets);
   }

   public RDXActiveMappingUIPanel(String name, ROS2Helper ros2Helper)
   {
      mode = Mode.REMOTE;
      imGuiPanel = new ImGuiPanel(name, this::renderImGuiWidgets);
      perceptionConfigurationParameters = new PerceptionConfigurationParameters();
      remotePropertySets = new ImGuiRemoteROS2StoredPropertySetGroup(ros2Helper);
      remotePropertySets.registerRemotePropertySet(perceptionConfigurationParameters, PerceptionComms.PERCEPTION_CONFIGURATION_PARAMETERS);
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox(labels.get("Render Enabled"), renderEnabled);

      if (mode == Mode.REMOTE)
      {
         remotePropertySets.renderImGuiWidgets();
      }
      else
      {
         if (ImGui.button("Calculate Footstep Plan") || ImGui.isKeyPressed(ImGuiTools.getSpaceKey()))
         {
            activeMappingModule.updateFootstepPlan();
            activeMappingModule.setWalkingEnabled(true);
         }
      }
   }

   public void render3DGraphics()
   {
      if (mode == Mode.REMOTE)
      {

      }
      else if (mode == Mode.LOCAL)
      {
         synchronized (mapPlanarRegionsGraphic)
         {
            if (renderEnabled.get())
            {
               mapPlanarRegionsGraphic.generateMeshes(activeMappingModule.getPlanarRegionMap().getMapRegions());
               mapPlanarRegionsGraphic.update();
            }
         }
      }
   }

   public ImGuiPanel getImGuiPanel()
   {
      return imGuiPanel;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (renderEnabled.get())
      {
         mapPlanarRegionsGraphic.getRenderables(renderables, pool);
      }
   }

   public ImGuiPanel getPanel()
   {
      return imGuiPanel;
   }

   public void destroy()
   {
      mapPlanarRegionsGraphic.destroy();
   }
}
