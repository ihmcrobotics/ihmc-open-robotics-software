package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.behaviors.activeMapping.ActiveMapper;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.mapping.PlanarRegionMap;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySetGroup;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;

import java.util.ArrayList;

public class RDXActiveMappingUI implements RenderableProvider
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RDXPlanarRegionsGraphic mapPlanarRegionsGraphic = new RDXPlanarRegionsGraphic();

   private final RDXFootstepPlanGraphic footstepPlanGraphic = new RDXFootstepPlanGraphic();

   private PerceptionConfigurationParameters perceptionConfigurationParameters;
   private ImGuiRemoteROS2StoredPropertySetGroup remotePropertySets;
   private ActiveMapper activeMapper;
   private RDXPanel panel;

   private ArrayList<ModelInstance> gridCylinders = new ArrayList<>();

   private final ImBoolean renderEnabled = new ImBoolean(true);

   private PlanarRegionMap planarRegionMap;

   public RDXActiveMappingUI(String name, PlanarRegionMap planarRegionMap, ActiveMapper mappingManager)
   {
      this.planarRegionMap = planarRegionMap;
      this.activeMapper = mappingManager;
      panel = new RDXPanel(name, this::renderImGuiWidgets);

      for (int i = 0; i < activeMapper.getGridSize(); i++)
      {
         for (int j = 0; j < activeMapper.getGridSize(); j++)
         {
            ModelInstance cylinderModel = RDXModelBuilder.createCylinder(0.02f, 0.03f, Color.BLUE);
            cylinderModel.transform.setToTranslation(activeMapper.getGridOrigin().getX32() + i * activeMapper.getGridResolution(),
                                                     activeMapper.getGridOrigin().getY32() + j * activeMapper.getGridResolution(),
                                                     0.0f);
            gridCylinders.add(cylinderModel);
         }
      }
   }

   public RDXActiveMappingUI(String name, ROS2Helper ros2Helper)
   {
      panel = new RDXPanel(name, this::renderImGuiWidgets);
      perceptionConfigurationParameters = new PerceptionConfigurationParameters();
      remotePropertySets = new ImGuiRemoteROS2StoredPropertySetGroup(ros2Helper);
      remotePropertySets.registerRemotePropertySet(perceptionConfigurationParameters, PerceptionComms.PERCEPTION_CONFIGURATION_PARAMETERS);
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox(labels.get("Render Enabled"), renderEnabled);

      if (remotePropertySets != null)
      {
         remotePropertySets.renderImGuiWidgets();

         if (ImGui.button("Calculate Footstep Plan") || ImGui.isKeyPressed(ImGuiTools.getSpaceKey()))
         {
            LogTools.info("Enabled Remove Active Mapping");
            if (perceptionConfigurationParameters != null)
            {
               //perceptionConfigurationParameters.setActiveMapping(true);
            }
         }
      }
      else
      {
         if (ImGui.button("Calculate Footstep Plan") || ImGui.isKeyPressed(ImGuiTools.getSpaceKey()))
         {
            LogTools.info("Triggered footstep plan calculation");
            activeMapper.updatePlan(planarRegionMap);
            activeMapper.setPlanAvailable(true);
         }
      }
   }

   public void render3DGraphics()
   {
      if (remotePropertySets == null)
      {
         synchronized (mapPlanarRegionsGraphic)
         {
            if (renderEnabled.get())
            {
               mapPlanarRegionsGraphic.generateMeshes(planarRegionMap.getMapRegions());
               mapPlanarRegionsGraphic.update();
            }
         }
      }
   }

   public RDXPanel getImGuiPanel()
   {
      return panel;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (renderEnabled.get())
      {
         mapPlanarRegionsGraphic.getRenderables(renderables, pool);

         for (ModelInstance gridCylinder : gridCylinders)
         {
            gridCylinder.getRenderables(renderables, pool);
         }
      }
   }

   public RDXPanel getPanel()
   {
      return panel;
   }

   public void destroy()
   {
      mapPlanarRegionsGraphic.destroy();
   }
}
