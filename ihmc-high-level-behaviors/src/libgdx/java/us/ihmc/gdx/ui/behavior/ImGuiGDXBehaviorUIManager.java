package us.ihmc.gdx.ui.behavior;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.extension.imnodes.ImNodes;
import imgui.flag.ImGuiInputTextFlags;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import org.apache.logging.log4j.Level;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.BehaviorModule;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.yo.YoBooleanClientHelper;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeControlFlowNode;
import us.ihmc.behaviors.tools.behaviorTree.FallbackNode;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.gdx.imgui.*;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.behavior.registry.ImGuiGDXBehaviorUIDefinition;
import us.ihmc.gdx.ui.behavior.registry.ImGuiGDXBehaviorUIInterface;
import us.ihmc.gdx.ui.behavior.registry.ImGuiGDXBehaviorUIRegistry;
import us.ihmc.gdx.ui.behavior.tree.ImGuiImNodesBehaviorTreeUI;
import us.ihmc.gdx.ui.tools.ImGuiLogWidget;
import us.ihmc.gdx.ui.tools.ImGuiMessagerManagerWidget;
import us.ihmc.gdx.ui.yo.ImGuiYoVariableClientManagerWidget;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Node;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import static us.ihmc.behaviors.BehaviorModule.API.BehaviorTreeStatus;
import static us.ihmc.behaviors.BehaviorModule.API.StatusLog;

public class ImGuiGDXBehaviorUIManager
{
   private final ImString behaviorModuleHost = new ImString("localhost", 100);
   private final AtomicReference<BehaviorTreeControlFlowNode> behaviorTreeStatus = new AtomicReference<>(new FallbackNode());
   private final Stopwatch statusStopwatch = new Stopwatch();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiMovingPlot statusReceivedPlot = new ImGuiMovingPlot("Tree status", 1000, 230, 15);
   private final ImGuiLogWidget logWidget;
   private final ImGuiMessagerManagerWidget messagerManagerWidget;
   private final ImGuiYoVariableClientManagerWidget yoVariableClientManagerWidget;
   private ImGuiGDXBehaviorUIInterface highestLevelUI;
   private final BehaviorHelper helper;
   private final ImGuiImNodesBehaviorTreeUI imNodeBehaviorTreeUI;
   private final ImGuiPanel treeViewPanel;
   private final ImBoolean imEnabled = new ImBoolean(false);
   private final YoBooleanClientHelper yoEnabled;
   private final ImGuiGDXBehaviorUIRegistry behaviorRegistry;
   private ImGuiGDXBehaviorUIInterface rootBehaviorUI;

   public ImGuiGDXBehaviorUIManager(ROS2Node ros2Node,
                                    Supplier<? extends DRCRobotModel> robotModelSupplier,
                                    ImGuiGDXBehaviorUIRegistry behaviorRegistry,
                                    boolean enableROS1)
   {
      this.behaviorRegistry = behaviorRegistry;
      helper = new BehaviorHelper("Behaviors panel", robotModelSupplier.get(), ros2Node, enableROS1);
      messagerManagerWidget = new ImGuiMessagerManagerWidget(helper.getMessagerHelper(), behaviorModuleHost::get);
      yoVariableClientManagerWidget = new ImGuiYoVariableClientManagerWidget(helper.getYoVariableClientHelper(),
                                                                             behaviorModuleHost::get,
                                                                             NetworkPorts.BEHAVIOR_MODULE_YOVARIABLESERVER_PORT.getPort());
      logWidget = new ImGuiLogWidget("Behavior status");
      helper.subscribeViaCallback(StatusLog, logWidget::submitEntry);
      helper.subscribeViaCallback(ROS2Tools.TEXT_STATUS, textStatus ->
      {
         LogTools.info("TextToSpeech: {}", textStatus.getTextToSpeakAsString());
         logWidget.submitEntry(Level.INFO, textStatus.getTextToSpeakAsString());
      });
      helper.subscribeViaCallback(BehaviorTreeStatus, status ->
      {
         statusStopwatch.reset();
         behaviorTreeStatus.set(status);
      });

      treeViewPanel = new ImGuiPanel(ImGuiTools.uniqueLabel(getClass(), "Behavior Tree Panel"), this::renderBehaviorTreeImGuiWidgets, false, true);
      treeViewPanel.addChild(new ImGuiPanel("Behaviors Status Log", logWidget::renderImGuiWidgets));

      rootBehaviorUI = new ImGuiGDXRootBehaviorUI(this::renderRootUIImGuiWidgets);
      imNodeBehaviorTreeUI = new ImGuiImNodesBehaviorTreeUI();

      yoEnabled = helper.subscribeToYoBoolean("enabled");

      highestLevelUI = behaviorRegistry.getHighestLevelNode().getBehaviorUISupplier().create(helper);
      rootBehaviorUI.addChild(highestLevelUI);
      imNodeBehaviorTreeUI.setRootNode(rootBehaviorUI);

      highestLevelUI.addChildPanelsIncludingChildren(treeViewPanel);
   }

   public void switchHighestLevelBehavior(String behaviorName)
   {
      // TODO: This needs to be a message sent to the module. This panel should react to differently shaped incoming trees.

      // disable things
      setEnabled(false);

      helper.publish(BehaviorModule.API.SET_HIGHEST_LEVEL_BEHAVIOR, behaviorName);

      highestLevelUI.destroy();

      rootBehaviorUI.clearChildren();
      behaviorRegistry.setHighestLevelNode(behaviorRegistry.getBehaviorFromName(behaviorName));
      highestLevelUI = behaviorRegistry.getHighestLevelNode().getBehaviorUISupplier().create(helper);
      rootBehaviorUI.addChild(highestLevelUI);

      imNodeBehaviorTreeUI.setRootNode(rootBehaviorUI);
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      ImNodesTools.initialize();

      imNodeBehaviorTreeUI.create();

      highestLevelUI.create(baseUI);
      baseUI.get3DSceneManager().addRenderableProvider(this::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
   }

   public void handleVREvents(GDXVRContext vrContext)
   {
      highestLevelUI.handleVREvents(vrContext);
   }

   public void update()
   {
      rootBehaviorUI.syncTree(behaviorTreeStatus.get());
      rootBehaviorUI.updateIncludingChildren();
   }

   public void renderBehaviorTreeImGuiWidgets()
   {
      ImGui.beginMenuBar();
      if (ImGui.beginMenu(labels.get("File")))
      {
         if (ImGui.menuItem(labels.get("Save imnodes layout")))
         {
            imNodeBehaviorTreeUI.saveLayoutToFile();
         }
         ImGui.endMenu();
      }
      if (ImGui.beginMenu(labels.get("Behavior")))
      {
         ImGui.text("Highest level behavior:");
         for (ImGuiGDXBehaviorUIDefinition behaviorUIDefinition : behaviorRegistry.getUIDefinitionEntries())
         {
            String behaviorName = behaviorUIDefinition.getName();
            boolean selected = behaviorName.equals(highestLevelUI.getName());
            if (ImGui.radioButton(labels.get(behaviorName), selected))
            {
               switchHighestLevelBehavior(behaviorName);
            }
         }

         ImGui.endMenu();
      }
      ImGui.endMenuBar();

      imNodeBehaviorTreeUI.renderImGuiWidgets();
   }

   public void renderRootUIImGuiWidgets()
   {
      ImGui.pushItemWidth(150.0f);
      int flags = ImGuiInputTextFlags.None;
      flags += ImGuiInputTextFlags.CallbackResize;
      ImGui.text("Behavior module host:");
      ImGui.sameLine();
      ImGui.inputText(ImGuiTools.uniqueIDOnly(getClass(), "BehaviorModuleHostInput"), behaviorModuleHost, flags);
      ImGui.popItemWidth();
      messagerManagerWidget.renderImGuiWidgets();
      yoVariableClientManagerWidget.renderImGuiWidgets();
      statusReceivedPlot.setNextValue((float) statusStopwatch.totalElapsed());
      statusReceivedPlot.calculate("");

      if (ImGui.checkbox(labels.get("Enabled"), imEnabled))
      {
         yoEnabled.set(imEnabled.get());
      }
      ImGui.sameLine();
      ImGui.text("Server: " + yoEnabled.get());
   }

   public void setEnabled(boolean enabled)
   {
      imEnabled.set(enabled);
      yoEnabled.set(enabled);
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      highestLevelUI.getRenderables(renderables, pool);
   }

   public void connectViaKryo(String hostname)
   {
      behaviorModuleHost.set(hostname);
      messagerManagerWidget.connect();
   }

   public void connectMessager()
   {
      messagerManagerWidget.connect();
   }

   public void connectYoVariableClient()
   {
      helper.getYoVariableClientHelper().start(behaviorModuleHost.get(), NetworkPorts.BEHAVIOR_MODULE_YOVARIABLESERVER_PORT.getPort());
   }

   public void disconnectMessager()
   {
      messagerManagerWidget.disconnectMessager();
   }

   public void destroy()
   {
      helper.destroy();
      highestLevelUI.destroy();
   }

   public ImGuiPanel getPanel()
   {
      return treeViewPanel;
   }
}
