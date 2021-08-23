package us.ihmc.gdx.ui.behaviors;

import imgui.flag.ImGuiInputTextFlags;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import org.apache.logging.log4j.Level;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.yo.YoBooleanClientHelper;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeControlFlowNode;
import us.ihmc.behaviors.tools.behaviorTree.FallbackNode;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.gdx.imgui.*;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIRegistry;
import us.ihmc.gdx.ui.tools.ImGuiLogWidget;
import us.ihmc.gdx.ui.tools.ImGuiMessagerManagerWidget;
import us.ihmc.gdx.ui.yo.ImGuiYoVariableClientManagerWidget;
import us.ihmc.gdx.vr.GDXVRManager;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Node;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import static us.ihmc.behaviors.BehaviorModule.API.BehaviorTreeStatus;
import static us.ihmc.behaviors.BehaviorModule.API.StatusLog;

public class GDXBehaviorsPanel extends GDXBehaviorUIInterface
{
   private final String windowName = ImGuiTools.uniqueLabel(getClass(), "Behaviors");
   private final ImString behaviorModuleHost = new ImString("localhost", 100);
   private final Point2D nodePosition = new Point2D(7.0, 5.0);
   private final AtomicReference<BehaviorTreeControlFlowNode> behaviorTreeStatus = new AtomicReference<>(new FallbackNode());
   private final Stopwatch statusStopwatch = new Stopwatch();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiMovingPlot statusReceivedPlot = new ImGuiMovingPlot("Tree status", 1000, 230, 15);
   private final ImGuiLogWidget logWidget;
   private final ImGuiMessagerManagerWidget messagerManagerWidget;
   private final ImGuiYoVariableClientManagerWidget yoVariableClientManagerWidget;
   private final GDXBehaviorUIInterface highestLevelUI;
   private final ImGuiBehaviorModuleDisabledNodeUI disabledNodeUI;
   private final BehaviorHelper behaviorHelper;
   private final ImGuiImNodesBehaviorTreePanel behaviorTreePanel = new ImGuiImNodesBehaviorTreePanel("Behavior Tree", this.getClass());
   private final ImGuiPanel panel = new ImGuiPanel(windowName, this::renderRegularPanelImGuiWidgetsAndChildren);
   private final ImBoolean imEnabled = new ImBoolean(false);
   private final YoBooleanClientHelper yoEnabled;

   public GDXBehaviorsPanel(ROS2Node ros2Node,
                            Supplier<? extends DRCRobotModel> robotModelSupplier,
                            GDXBehaviorUIRegistry behaviorRegistry)
   {
      behaviorHelper = new BehaviorHelper("Behaviors panel", robotModelSupplier.get(), ros2Node);
      messagerManagerWidget = new ImGuiMessagerManagerWidget(behaviorHelper.getMessagerHelper(), behaviorModuleHost::get);
      yoVariableClientManagerWidget = new ImGuiYoVariableClientManagerWidget(behaviorHelper.getYoVariableClientHelper(),
                                                                             behaviorModuleHost::get,
                                                                             NetworkPorts.BEHAVIOR_MODULE_YOVARIABLESERVER_PORT.getPort());
      logWidget = new ImGuiLogWidget("Behavior status");
      behaviorHelper.subscribeViaCallback(StatusLog, logWidget::submitEntry);
      behaviorHelper.subscribeViaCallback(ROS2Tools.TEXT_STATUS, textStatus ->
      {
         LogTools.info("TextToSpeech: {}", textStatus.getTextToSpeakAsString());
         logWidget.submitEntry(Level.INFO, textStatus.getTextToSpeakAsString());
      });
      behaviorHelper.subscribeViaCallback(BehaviorTreeStatus, status ->
      {
         statusStopwatch.reset();
         behaviorTreeStatus.set(status);
      });
      panel.addChild(new ImGuiPanel(behaviorTreePanel.getWindowName(), () -> behaviorTreePanel.renderImGuiWidgets(this)));
      disabledNodeUI = new ImGuiBehaviorModuleDisabledNodeUI(behaviorHelper);
      addChild(disabledNodeUI);
      highestLevelUI = behaviorRegistry.getHighestLevelNode().getBehaviorUISupplier().create(behaviorHelper);
      addChild(highestLevelUI);

      yoEnabled = behaviorHelper.subscribeToYoBoolean("enabled");
   }

   @Override
   public void create(GDXImGuiBasedUI baseUI)
   {
//      behaviorTreePanel.create();
      addChildPanelsIncludingChildren(panel);
      highestLevelUI.create(baseUI);
   }

   public void handleVREvents(GDXVRManager vrManager)
   {
      highestLevelUI.handleVREvents(vrManager);
   }

   @Override
   public void update()
   {
      syncTree(behaviorTreeStatus.get());
   }

   @Override
   public void renderRegularPanelImGuiWidgets()
   {
      logWidget.renderImGuiWidgets();
   }

   @Override
   public Point2D getTreeNodeInitialPosition()
   {
      return nodePosition;
   }

   @Override
   public void renderTreeNodeImGuiWidgets()
   {
      ImGui.pushItemWidth(150.0f);
      int flags = ImGuiInputTextFlags.None;
      flags += ImGuiInputTextFlags.CallbackResize;
      ImGui.inputText(labels.get("Behavior module host"), behaviorModuleHost, flags);
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

   public void connectViaKryo(String hostname)
   {
      behaviorModuleHost.set(hostname);
      messagerManagerWidget.connectViaKryo(hostname);
   }

   public void connectYoVariableClient()
   {
      behaviorHelper.getYoVariableClientHelper().start(behaviorModuleHost.get(), NetworkPorts.BEHAVIOR_MODULE_YOVARIABLESERVER_PORT.getPort());
   }

   public void disconnectMessager()
   {
      messagerManagerWidget.disconnectMessager();
   }

   @Override
   public void destroy()
   {
      behaviorHelper.destroy();
      highestLevelUI.destroy();
   }

   public String getWindowName()
   {
      return windowName;
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}
