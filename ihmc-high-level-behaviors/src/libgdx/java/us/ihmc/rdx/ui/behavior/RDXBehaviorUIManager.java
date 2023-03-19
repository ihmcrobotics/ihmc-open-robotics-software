package us.ihmc.rdx.ui.behavior;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
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
import us.ihmc.communication.ros2.ROS2HeartbeatMonitor;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIDefinition;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIInterface;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIRegistry;
import us.ihmc.rdx.ui.behavior.tree.RDXImNodesBehaviorTreeUI;
import us.ihmc.rdx.ui.tools.ImGuiLogWidget;
import us.ihmc.rdx.ui.tools.ImGuiMessagerManagerWidget;
import us.ihmc.rdx.ui.yo.ImGuiYoVariableClientManagerWidget;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Node;

import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import static us.ihmc.behaviors.BehaviorModule.API.BehaviorTreeStatus;
import static us.ihmc.behaviors.BehaviorModule.API.StatusLog;

public class RDXBehaviorUIManager
{
   private final ImString behaviorModuleHost = new ImString("localhost", 100);
   private final AtomicReference<BehaviorTreeControlFlowNode> behaviorTreeStatus = new AtomicReference<>(new FallbackNode());
   private final Stopwatch statusStopwatch = new Stopwatch();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiMovingPlot statusReceivedPlot = new ImGuiMovingPlot("Tree status", 1000, 230, 15);
   private final ImGuiLogWidget logWidget;
   private final ImGuiMessagerManagerWidget messagerManagerWidget;
   private final ImGuiYoVariableClientManagerWidget yoVariableClientManagerWidget;
   private RDXBehaviorUIInterface highestLevelUI;
   private final BehaviorHelper helper;
   private final RDXImNodesBehaviorTreeUI imNodeBehaviorTreeUI;
   private final ImGuiPanel treeViewPanel;
   private final ImBoolean imEnabled = new ImBoolean(false);
   private final YoBooleanClientHelper yoEnabled;
   private final RDXBehaviorUIRegistry behaviorRegistry;
   private RDXBehaviorUIInterface rootBehaviorUI;
   private final ROS2HeartbeatMonitor heartbeatMonitor;

   public RDXBehaviorUIManager(ROS2Node ros2Node,
                               Supplier<? extends DRCRobotModel> robotModelSupplier,
                               RDXBehaviorUIRegistry behaviorRegistry,
                               boolean enableROS1)
   {
      this.behaviorRegistry = behaviorRegistry;
      helper = new BehaviorHelper("Behaviors panel", robotModelSupplier.get(), ros2Node, enableROS1);
      heartbeatMonitor = new ROS2HeartbeatMonitor(helper, BehaviorModule.API.HEARTBEAT);
      messagerManagerWidget = new ImGuiMessagerManagerWidget(helper.getMessagerHelper(),
                                                             behaviorModuleHost::get,
                                                             NetworkPorts.BEHAVIOR_MODULE_MESSAGER_PORT.getPort());
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

      rootBehaviorUI = new RDXRootBehaviorUI(this::renderRootUIImGuiWidgets);
      imNodeBehaviorTreeUI = new RDXImNodesBehaviorTreeUI();

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

   public void create(RDXBaseUI baseUI)
   {
      ImNodesTools.initialize();

      imNodeBehaviorTreeUI.create();

      highestLevelUI.create(baseUI);
      baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);
   }

   public void handleVREvents(RDXVRContext vrContext)
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
         for (RDXBehaviorUIDefinition behaviorUIDefinition : behaviorRegistry.getUIDefinitionEntries())
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

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      highestLevelUI.getRenderables(renderables, pool, sceneLevels);
   }

   public void setBehaviorModuleHost(String hostname)
   {
      behaviorModuleHost.set(hostname);
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

   public ROS2HeartbeatMonitor getHeartbeatMonitor()
   {
      return heartbeatMonitor;
   }
}
