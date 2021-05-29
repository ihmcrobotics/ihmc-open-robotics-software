package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import imgui.flag.ImGuiTreeNodeFlags;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import org.apache.commons.lang3.StringUtils;
import us.ihmc.behaviors.demo.BuildingExplorationBehavior;
import us.ihmc.behaviors.demo.BuildingExplorationStateName;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.gdx.imgui.ImGuiEnumPlot;
import us.ihmc.gdx.imgui.ImGuiLabelMap;
import us.ihmc.gdx.simulation.environment.object.objects.GDXDoorOnlyObject;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.affordances.ImGuiGDXPoseGoalAffordance;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIDefinition;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;
import us.ihmc.gdx.ui.graphics.GDXFootstepPlanGraphic;

import static us.ihmc.behaviors.demo.BuildingExplorationBehaviorAPI.*;

public class ImGuiGDXBuildingExplorationBehaviorUI extends GDXBehaviorUIInterface
{
   public static final GDXBehaviorUIDefinition DEFINITION = new GDXBehaviorUIDefinition(BuildingExplorationBehavior.DEFINITION,
                                                                                        ImGuiGDXBuildingExplorationBehaviorUI::new);

   private final BehaviorHelper helper;
   private final GDXFootstepPlanGraphic controllerFootsteps = new GDXFootstepPlanGraphic();
   private final ImGuiGDXPoseGoalAffordance goalAffordance = new ImGuiGDXPoseGoalAffordance();
   private final ImGuiGDXLookAndStepBehaviorUI lookAndStepUI;
   private final ImGuiGDXTraverseStairsBehaviorUI traverseStairsUI;
   private GDXDoorOnlyObject door;
   private ImGuiLabelMap labels = new ImGuiLabelMap();
   private String[] stateNames = new String[BuildingExplorationStateName.values().length];
   private final ImGuiEnumPlot currentStatePlot = new ImGuiEnumPlot(1000, 250, 30);
   private BuildingExplorationStateName currentState = null;
   private ImBoolean ignoreDebris = new ImBoolean();

   public ImGuiGDXBuildingExplorationBehaviorUI(BehaviorHelper helper)
   {
      this.helper = helper;

      for (int i = 0; i < BuildingExplorationStateName.values.length; i++)
      {
         stateNames[i] = StringUtils.capitalize(BuildingExplorationStateName.values[i].name().toLowerCase().replaceAll("_", " "));
      }

      helper.subscribeToControllerViaCallback(FootstepDataListMessage.class, footsteps ->
      {
         controllerFootsteps.generateMeshesAsync(MinimalFootstep.convertFootstepDataListMessage(footsteps));
      });

      lookAndStepUI = new ImGuiGDXLookAndStepBehaviorUI(helper);
      traverseStairsUI = new ImGuiGDXTraverseStairsBehaviorUI(helper);

      helper.subscribeToDoorLocationViaCallback(doorLocation -> door.set(doorLocation.getDoorTransformToWorld()));
      helper.subscribeViaCallback(CurrentState, state -> currentState = state);
   }

   @Override
   public void create(GDXImGuiBasedUI baseUI)
   {
      goalAffordance.create(baseUI, goalPose -> helper.publish(Goal, goalPose), Color.GREEN);
      baseUI.addImGui3DViewInputProcessor(goalAffordance::processImGui3DViewInput);

      door = new GDXDoorOnlyObject();

      lookAndStepUI.create(baseUI);
   }

   @Override
   public void render()
   {
      ImGui.text("Building Exploration");
      goalAffordance.renderPlaceGoalButton();

      for (int i = 0; i < BuildingExplorationStateName.values.length; i++)
      {
         if (ImGui.button(labels.get(stateNames[i])))
         {
            helper.publish(RequestedState, BuildingExplorationStateName.values[i]);
            helper.publish(Start);
         }
         if (i < BuildingExplorationStateName.values.length - 1)
            ImGui.sameLine();
      }
      if (ImGui.button(labels.get("Stop")))
      {
         helper.publish(Stop);
      }
      if (ImGui.button(labels.get("Ignore debris")))
      {
         helper.publish(IgnoreDebris, true); // TODO: Fix this
      }

      ImGui.text("Current state:");
      currentStatePlot.render(currentState == null ? -1 : currentState.ordinal(), currentState == null ? "" : currentState.name());

      int defaultOpen = ImGuiTreeNodeFlags.DefaultOpen;
      if (ImGui.collapsingHeader("Look and Step", defaultOpen))
      {
         lookAndStepUI.render();
      }

      controllerFootsteps.render();
   }

   @Override
   public void destroy()
   {
      controllerFootsteps.destroy();
      lookAndStepUI.destroy();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      goalAffordance.getRenderables(renderables, pool);
      controllerFootsteps.getRenderables(renderables, pool);
      lookAndStepUI.getRenderables(renderables, pool);
   }
}
