package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.StoredPropertySetMessage;
import imgui.flag.ImGuiInputTextFlags;
import imgui.flag.ImGuiTreeNodeFlags;
import imgui.internal.ImGui;
import com.badlogic.gdx.graphics.*;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.behaviors.BehaviorModule;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParameterKeys;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.gdx.imgui.*;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.gdx.ui.affordances.ImGuiGDXPoseGoalAffordance;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIDefinition;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;
import us.ihmc.gdx.ui.graphics.GDXBodyPathPlanGraphic;
import us.ihmc.gdx.ui.graphics.GDXFootstepPlanGraphic;
import us.ihmc.gdx.ui.yo.ImGuiYoDoublePlot;
import us.ihmc.gdx.visualizers.GDXPlanarRegionsGraphic;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorParameters;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;

import static us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class ImGuiGDXLookAndStepBehaviorUI extends GDXBehaviorUIInterface
{
   public static final GDXBehaviorUIDefinition DEFINITION = new GDXBehaviorUIDefinition(LookAndStepBehavior.DEFINITION,
                                                                                        ImGuiGDXLookAndStepBehaviorUI::new);

   private final BehaviorHelper behaviorHelper;

   private String currentState = "";
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private final ImBoolean operatorReview = new ImBoolean(true);
   private final ImGuiEnumPlot currentStatePlot = new ImGuiEnumPlot(1000, 250, 50);
   private long numberOfSteppingRegionsReceived = 0;
   private final ImGuiPlot steppingRegionsPlot = new ImGuiPlot("", 1000, 230, 30);
   private final ImBoolean showGraphics = new ImBoolean(true);
   private final ImBoolean showLookAndStepParametersTuner = new ImBoolean(true);
   private final ImBoolean showFootstepPlanningParametersTuner = new ImBoolean(true);
   private final ImBoolean showSwingPlanningParametersTuner = new ImBoolean(true);
   private final ImGuiYoDoublePlot footholdVolumePlot;

   private boolean reviewingBodyPath = true;
   private final ImString latestFootstepPlannerLogPath = new ImString();
   private ArrayList<Pair<Integer, Double>> latestFootstepPlannerRejectionReasons = new ArrayList<>();

   private final GDXPlanarRegionsGraphic planarRegionsGraphic = new GDXPlanarRegionsGraphic();
   private final GDXBodyPathPlanGraphic bodyPathPlanGraphic = new GDXBodyPathPlanGraphic();
   private final GDXFootstepPlanGraphic footstepPlanGraphic = new GDXFootstepPlanGraphic();
   private final GDXFootstepPlanGraphic commandedFootstepsGraphic = new GDXFootstepPlanGraphic();
   private final GDXFootstepPlanGraphic startAndGoalFootstepsGraphic = new GDXFootstepPlanGraphic();
   private final ImGuiStoredPropertySetTuner lookAndStepParameterTuner = new ImGuiStoredPropertySetTuner("Look and Step Parameters");
   private final ImGuiStoredPropertySetTuner footstepPlannerParameterTuner = new ImGuiStoredPropertySetTuner("Footstep Planner Parameters (for Look and Step)");
   private final ImGuiStoredPropertySetTuner swingPlannerParameterTuner = new ImGuiStoredPropertySetTuner("Swing Planner Parameters (for Look and Step)");
   private final ImGuiBehaviorTreePanel treePanel = new ImGuiBehaviorTreePanel("Look and step");
   private final ImGuiGDXPoseGoalAffordance goalAffordance = new ImGuiGDXPoseGoalAffordance();

   public ImGuiGDXLookAndStepBehaviorUI(BehaviorHelper behaviorHelper)
   {
      this.behaviorHelper = behaviorHelper;
      behaviorHelper.subscribeViaCallback(CurrentState, state -> currentState = state);
      behaviorHelper.subscribeViaCallback(OperatorReviewEnabledToUI, operatorReview::set);
      behaviorHelper.subscribeViaCallback(PlanarRegionsForUI, regions ->
      {
         goalAffordance.setLatestRegions(regions);
         ++numberOfSteppingRegionsReceived;
         if (regions != null)
            planarRegionsGraphic.generateMeshesAsync(regions);
      });
      behaviorHelper.subscribeViaCallback(BodyPathPlanForUI, bodyPath ->
      {
         if (bodyPath != null)
            bodyPathPlanGraphic.generateMeshesAsync(bodyPath);
      });
      footstepPlanGraphic.setTransparency(0.2);
      behaviorHelper.subscribeViaCallback(FootstepPlanForUI, footsteps ->
      {
         reviewingBodyPath = false;
         footstepPlanGraphic.generateMeshesAsync(footsteps);
      });
      behaviorHelper.subscribeViaCallback(LastCommandedFootsteps, commandedFootstepsGraphic::generateMeshesAsync);
      startAndGoalFootstepsGraphic.setColor(RobotSide.LEFT, Color.BLUE);
      startAndGoalFootstepsGraphic.setColor(RobotSide.RIGHT, Color.BLUE);
      startAndGoalFootstepsGraphic.setTransparency(0.4);
      behaviorHelper.subscribeViaCallback(StartAndGoalFootPosesForUI, startAndGoalFootstepsGraphic::generateMeshesAsync);
      behaviorHelper.subscribeViaCallback(FootstepPlannerLatestLogPath, latestFootstepPlannerLogPath::set);
      behaviorHelper.subscribeViaCallback(FootstepPlannerRejectionReasons, reasons -> latestFootstepPlannerRejectionReasons = reasons);
      footholdVolumePlot = new ImGuiYoDoublePlot("footholdVolume", behaviorHelper);
   }

   @Override
   public void create(GDXImGuiBasedUI baseUI)
   {
      LookAndStepBehaviorParameters lookAndStepParameters = new LookAndStepBehaviorParameters();
      lookAndStepParameterTuner.create(lookAndStepParameters,
                                       LookAndStepBehaviorParameters.keys,
                                       () ->
                                       {
                                          StoredPropertySetMessage storedPropertySetMessage = new StoredPropertySetMessage();
                                          lookAndStepParameters.getAllAsStrings().forEach(value -> storedPropertySetMessage.getStrings().add(value));
                                          behaviorHelper.publish(LOOK_AND_STEP_PARAMETERS, storedPropertySetMessage);
                                       });

      FootstepPlannerParametersBasics footstepPlannerParameters = behaviorHelper.getRobotModel().getFootstepPlannerParameters("ForLookAndStep");
      footstepPlannerParameterTuner.create(footstepPlannerParameters,
                                           FootstepPlannerParameterKeys.keys,
                                           () -> behaviorHelper.publish(FootstepPlannerParameters, footstepPlannerParameters.getAllAsStrings()));

      SwingPlannerParametersBasics swingPlannerParameters = behaviorHelper.getRobotModel().getSwingPlannerParameters("ForLookAndStep");
      swingPlannerParameterTuner.create(swingPlannerParameters, SwingPlannerParameterKeys.keys,
                                        () -> behaviorHelper.publish(SwingPlannerParameters, swingPlannerParameters.getAllAsStrings()));

      goalAffordance.create(baseUI, goalPose -> behaviorHelper.publish(GOAL_INPUT, goalPose), Color.CYAN);

      baseUI.addImGui3DViewInputProcessor(this::processImGui3DViewInput);
   }

   public void processImGui3DViewInput(ImGui3DViewInput input)
   {
      goalAffordance.processImGui3DViewInput(input);
   }

   public void renderAsWindow()
   {
      ImGui.begin(getWindowName());
      render();
      ImGui.end();
   }

   @Override
   public void render()
   {
      ImGui.text("Current state:");
      if (!currentState.isEmpty())
      {
         LookAndStepBehavior.State state = LookAndStepBehavior.State.valueOf(currentState);
         currentStatePlot.render(state.ordinal(), state.name());
      }
      else
      {
         currentStatePlot.render(-1, "");
      }

      if (ImGui.button(labels.get("Select behavior")))
      {
         behaviorHelper.publish(BehaviorModule.API.BehaviorSelection, LookAndStepBehavior.DEFINITION.getName());
      }
      ImGui.sameLine();
      if (ImGui.button("Reset"))
      {
         behaviorHelper.publish(RESET);
      }
      ImGui.sameLine();

      goalAffordance.renderPlaceGoalButton();

      if (ImGui.checkbox("Operator review", operatorReview))
      {
         behaviorHelper.publish(OperatorReviewEnabled, operatorReview.get());
      }
      if (ImGui.button("Reject"))
      {
         behaviorHelper.publish(ReviewApproval, false);
      }
      ImGui.sameLine();
      if (ImGui.button("Approve"))
      {
         behaviorHelper.publish(ReviewApproval, true);
      }
      ImGui.text("Footstep planning regions recieved:");
      steppingRegionsPlot.render(numberOfSteppingRegionsReceived);
      footholdVolumePlot.render();

      ImGui.checkbox("Show graphics", showGraphics);
      ImGui.sameLine();
      if (ImGui.button("Add support regions once"))
      {
         behaviorHelper.publish(PublishSupportRegions);
      }

//      if (ImGui.collapsingHeader("Behavior Visualization"))
//      {
//         ImGui.checkbox("Show tuner", showLookAndStepParametersTuner);
//         treePanel.renderWidgetsOnly();
//      }
      int defaultOpen = ImGuiTreeNodeFlags.DefaultOpen;
      if (ImGui.collapsingHeader("Footstep Planning", defaultOpen))
      {
         int flags = ImGuiInputTextFlags.ReadOnly;
         latestFootstepPlannerLogPath.set(latestFootstepPlannerLogPath.get().replace(System.getProperty("user.home"), "~"));
         ImGui.pushItemWidth(ImGui.getWindowWidth() - 3);
         ImGui.text("Latest log:");
         ImGui.inputText("", latestFootstepPlannerLogPath, flags);
         ImGui.popItemWidth();
//         ImGui.checkbox("Show tuner", showFootstepPlanningParametersTuner);

         ImGui.text("Rejection reasons:");
         for (Pair<Integer, Double> latestFootstepPlannerRejectionReason : latestFootstepPlannerRejectionReasons)
         {
            ImGui.text(latestFootstepPlannerRejectionReason.getRight() + "%: "
                       + BipedalFootstepPlannerNodeRejectionReason.values[latestFootstepPlannerRejectionReason.getLeft()].name());
         }
      }
//      if (ImGui.collapsingHeader("Swing Planning"))
//      {
//         ImGui.checkbox("Show tuner", showSwingPlanningParametersTuner);
//      }

      if (showLookAndStepParametersTuner.get())
         lookAndStepParameterTuner.render();
      if (showFootstepPlanningParametersTuner.get())
         footstepPlannerParameterTuner.render();
      if (showSwingPlanningParametersTuner.get())
         swingPlannerParameterTuner.render();

      if (areGraphicsEnabled())
      {
         footstepPlanGraphic.render();
         commandedFootstepsGraphic.render();
         startAndGoalFootstepsGraphic.render();
         planarRegionsGraphic.render();
         bodyPathPlanGraphic.render();
      }
   }

   private boolean areGraphicsEnabled()
   {
      return showGraphics.get() && !currentState.isEmpty() && !currentState.equals(LookAndStepBehavior.State.RESET.name());
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (showGraphics.get())
      {
         goalAffordance.getRenderables(renderables, pool);
      }
      if (areGraphicsEnabled())
      {
         footstepPlanGraphic.getRenderables(renderables, pool);
         commandedFootstepsGraphic.getRenderables(renderables, pool);
         startAndGoalFootstepsGraphic.getRenderables(renderables, pool);
         planarRegionsGraphic.getRenderables(renderables, pool);
         bodyPathPlanGraphic.getRenderables(renderables, pool);
      }
   }

   @Override
   public void destroy()
   {
      footstepPlanGraphic.destroy();
      commandedFootstepsGraphic.destroy();
      startAndGoalFootstepsGraphic.destroy();
      planarRegionsGraphic.destroy();
      bodyPathPlanGraphic.destroy();
   }

   public String getWindowName()
   {
      return LookAndStepBehavior.DEFINITION.getName();
   }
}
