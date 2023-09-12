package us.ihmc.rdx.ui.behavior;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import org.apache.logging.log4j.Level;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.BehaviorDefinition;
import us.ihmc.behaviors.BehaviorModule;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.BehaviorMessageTools;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeBasics;
import us.ihmc.behaviors.tools.behaviorTree.FallbackNode;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIDefinition;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIInterface;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIRegistry;
import us.ihmc.rdx.ui.behavior.tree.RDXImNodesBehaviorTreeUI;
import us.ihmc.rdx.ui.tools.ImGuiLogWidget;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Node;

import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import static us.ihmc.behaviors.BehaviorModule.API.*;

/**
 * This is the UI for interacting with a remotely running behavior tree.
 * TODO:
 *   - Somehow rework the ImNodes area, it takes a lot of space. Perhaps showing the
 *     nodes as normal panels as an option would be good.
 */
public class RDXBehaviorUIManager
{
   private final AtomicReference<BehaviorTreeNodeBasics> behaviorTreeStatus = new AtomicReference<>(new FallbackNode());
   private final Stopwatch statusStopwatch = new Stopwatch();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiMovingPlot statusReceivedPlot = new ImGuiMovingPlot("Tree status", 1000, 230, 15);
   private final ImGuiLogWidget logWidget;
   private final BehaviorHelper helper;
   private final RDXBaseUI baseUI;
   private final RDXImNodesBehaviorTreeUI imNodeBehaviorTreeUI;
   private final RDXPanel treeViewPanel;
   private final RDXBehaviorUIRegistry behaviorRegistry;
   private final RDXBehaviorUIInterface rootBehaviorUI;
   private RDXBehaviorUIInterface highestLevelUI;

   public RDXBehaviorUIManager(ROS2Node ros2Node,
                               Supplier<? extends DRCRobotModel> robotModelSupplier,
                               RDXBaseUI baseUI,
                               RDXBehaviorUIRegistry behaviorRegistry)
   {
      this.baseUI = baseUI;
      this.behaviorRegistry = behaviorRegistry;

      helper = new BehaviorHelper("Behaviors panel", robotModelSupplier.get(), ros2Node);
      logWidget = new ImGuiLogWidget("Behavior status");
      helper.subscribeViaCallback(STATUS_LOG, message ->
            logWidget.submitEntry(message.getLogLevel(), MessageTools.unpackLongStringFromByteSequence(message.getLogMessage())));
      helper.subscribeViaCallback(ROS2Tools.TEXT_STATUS, textStatus ->
      {
         LogTools.info("TextToSpeech: {}", textStatus.getTextToSpeakAsString());
         logWidget.submitEntry(Level.INFO, textStatus.getTextToSpeakAsString());
      });
      helper.subscribeViaCallback(BEHAVIOR_TREE_STATUS, behaviorTreeMessage ->
      {
         statusStopwatch.reset();
         behaviorTreeStatus.set(BehaviorMessageTools.unpackBehaviorTreeMessage(behaviorTreeMessage));
      });

      treeViewPanel = new RDXPanel("Behavior Tree Panel", this::renderBehaviorTreeImGuiWidgets, false, true);
      treeViewPanel.addChild(new RDXPanel("Behaviors Status Log", logWidget::renderImGuiWidgets));

      rootBehaviorUI = new RDXRootBehaviorUI(this::renderRootUIImGuiWidgets);
      imNodeBehaviorTreeUI = new RDXImNodesBehaviorTreeUI();
   }

   public void switchHighestLevelBehavior(String behaviorName)
   {
      // TODO: This needs to be a message sent to the module. This panel should react to differently shaped incoming trees.
      helper.publish(BehaviorModule.API.SET_HIGHEST_LEVEL_BEHAVIOR, behaviorName);

      if (highestLevelUI != null)
      {
         rootBehaviorUI.clearChildren();
         highestLevelUI.destroy();
         highestLevelUI = null;
         behaviorRegistry.setHighestLevelNode(null);
      }

      if (!behaviorName.equals(BehaviorDefinition.NONE))
      {
         behaviorRegistry.setHighestLevelNode(behaviorRegistry.getBehaviorFromName(behaviorName));
         highestLevelUI = behaviorRegistry.getHighestLevelNode().getBehaviorUISupplier().create(helper);
         highestLevelUI.addChildPanelsIncludingChildren(treeViewPanel);
         highestLevelUI.create(baseUI);

         rootBehaviorUI.addChild(highestLevelUI);
         imNodeBehaviorTreeUI.setRootNode(rootBehaviorUI); // rebuilds the UI
      }
   }

   public void create(RDXBaseUI baseUI)
   {
      ImNodesTools.initialize();
      imNodeBehaviorTreeUI.create();
      baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);
   }

   public void handleVREvents(RDXVRContext vrContext)
   {
      if (highestLevelUI != null)
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

         String selectedBehaviorName = highestLevelUI == null ? BehaviorDefinition.NONE : highestLevelUI.getName();
         if (ImGui.radioButton(labels.get(BehaviorDefinition.NONE), selectedBehaviorName.equals(BehaviorDefinition.NONE)))
         {
            switchHighestLevelBehavior(BehaviorDefinition.NONE);
         }
         for (RDXBehaviorUIDefinition behaviorUIDefinition : behaviorRegistry.getUIDefinitionEntries())
         {
            String behaviorName = behaviorUIDefinition.getName();
            if (ImGui.radioButton(labels.get(behaviorName), selectedBehaviorName.equals(behaviorName)))
            {
               switchHighestLevelBehavior(behaviorName);
            }
         }

         ImGui.endMenu();
      }
      ImGui.endMenuBar();

      if (highestLevelUI != null)
         imNodeBehaviorTreeUI.renderImGuiWidgets();
   }

   public void renderRootUIImGuiWidgets()
   {
      statusReceivedPlot.setNextValue((float) statusStopwatch.totalElapsed());
      statusReceivedPlot.calculate("");

      ImGui.text("Root node: %s".formatted(highestLevelUI == null ? "None." : highestLevelUI.getName()));
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (highestLevelUI != null)
         highestLevelUI.getRenderables(renderables, pool, sceneLevels);
   }

   public void destroy()
   {
      if (highestLevelUI != null)
         highestLevelUI.destroy();
      helper.destroy();
   }

   public RDXPanel getPanel()
   {
      return treeViewPanel;
   }
}
