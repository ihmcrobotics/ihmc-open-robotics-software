package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import org.apache.commons.lang3.StringUtils;
import us.ihmc.behaviors.demo.BuildingExplorationBehavior;
import us.ihmc.behaviors.demo.BuildingExplorationStateName;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.gdx.imgui.ImGuiEnumPlot;
import us.ihmc.gdx.imgui.ImGuiLabelMap;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.simulation.environment.object.objects.GDXDoorOnlyObject;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.affordances.ImGuiGDXPoseGoalAffordance;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIDefinition;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;

import static us.ihmc.behaviors.demo.BuildingExplorationBehaviorAPI.*;

public class ImGuiGDXBuildingExplorationBehaviorUI extends GDXBehaviorUIInterface
{
   public static final GDXBehaviorUIDefinition DEFINITION = new GDXBehaviorUIDefinition(BuildingExplorationBehavior.DEFINITION,
                                                                                        ImGuiGDXBuildingExplorationBehaviorUI::new);

   private final BehaviorHelper helper;
   private final ImGuiGDXPoseGoalAffordance goalAffordance = new ImGuiGDXPoseGoalAffordance();
   private final ImGuiGDXLookAndStepBehaviorUI lookAndStepUI;
   private final ImGuiGDXTraverseStairsBehaviorUI traverseStairsUI;
   private GDXDoorOnlyObject door;
   private ImGuiLabelMap labels = new ImGuiLabelMap();
   private String[] stateNames = new String[BuildingExplorationStateName.values().length];
   private final ImGuiEnumPlot currentStatePlot = new ImGuiEnumPlot(1000, 250, 30);
   private final ImGuiPlot debrisDetectedPlot = new ImGuiPlot("Debris detected", 1000, 250, 30);
   private volatile boolean debrisDetected = false;
   private final ImGuiPlot stairsDetectedPlot = new ImGuiPlot("Stairs detected", 1000, 250, 30);
   private volatile boolean stairsDetected = false;
   private final ImGuiPlot doorDetectedPlot = new ImGuiPlot("Door detected", 1000, 250, 30);
   private volatile boolean doorDetected = false;
   private BuildingExplorationStateName currentState = null;
   private ImBoolean ignoreDebris = new ImBoolean();

   public ImGuiGDXBuildingExplorationBehaviorUI(BehaviorHelper helper)
   {
      this.helper = helper;

      for (int i = 0; i < BuildingExplorationStateName.values.length; i++)
      {
         stateNames[i] = StringUtils.capitalize(BuildingExplorationStateName.values[i].name().toLowerCase().replaceAll("_", " "));
      }

      lookAndStepUI = new ImGuiGDXLookAndStepBehaviorUI(helper);
      traverseStairsUI = new ImGuiGDXTraverseStairsBehaviorUI(helper);

      helper.subscribeToDoorLocationViaCallback(doorLocation -> door.set(doorLocation.getDoorTransformToWorld()));
      helper.subscribeViaCallback(CurrentState, state -> currentState = state);
      helper.subscribeViaCallback(DebrisDetected, detected -> debrisDetected = detected);
      helper.subscribeViaCallback(StairsDetected, detected -> stairsDetected = detected);
      helper.subscribeViaCallback(DoorDetected, detected -> doorDetected = detected);
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
   public void renderNode(String nodeName)
   {
      if (nodeName.equals(DEFINITION.getName()))
      {
         render();
      }
      else
      {
         lookAndStepUI.renderNode(nodeName); // TODO: UIs need children
      }
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
      if (ImGui.button(labels.get("Confirm door")))
      {
         helper.publish(ConfirmDoor, true);
      }

      ImGui.text("Current state:");
      currentStatePlot.render(currentState == null ? -1 : currentState.ordinal(), currentState == null ? "" : currentState.name());
      debrisDetectedPlot.render(debrisDetected ? 1.0f : 0.0f);
      stairsDetectedPlot.render(stairsDetected ? 1.0f : 0.0f);
      doorDetectedPlot.render(doorDetected ? 1.0f : 0.0f);

//      int defaultOpen = ImGuiTreeNodeFlags.DefaultOpen;
//      if (ImGui.collapsingHeader("Look and Step", defaultOpen))
//      {
//         lookAndStepUI.render();
//      }
   }

   @Override
   public void destroy()
   {
      lookAndStepUI.destroy();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      goalAffordance.getRenderables(renderables, pool);
      lookAndStepUI.getRenderables(renderables, pool);
      door.getRealisticModelInstance().getRenderables(renderables, pool);
   }
}
