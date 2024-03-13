package us.ihmc.rdx.ui.behavior.tree;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeLayer;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeExtensionSubtreeRebuilder;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionType;
import us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTreeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.rdx.imgui.ImGuiExpandCollapseRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionSequence;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXBehaviorTree
{
   private final CRDTInfo crdtInfo = new CRDTInfo(ROS2ActorDesignation.OPERATOR, (int) ROS2BehaviorTreeState.SYNC_FREQUENCY);
   private final WorkspaceResourceDirectory treeFilesDirectory;
   private final RDXBehaviorTreeNodeBuilder nodeBuilder;
   private final BehaviorTreeExtensionSubtreeRebuilder treeRebuilder;
   private final BehaviorTreeState behaviorTreeState;
   private RDXBehaviorTreeNode<?, ?> rootNode;
   /**
    * Useful for accessing nodes by ID instead of searching.
    * Also, sometimes, the tree will be disassembled and this is used in putting it
    * back together.
    */
   private transient final TLongObjectMap<RDXBehaviorTreeNode<?, ?>> idToNodeMap = new TLongObjectHashMap<>();
   private final RDXPanel panel = new RDXPanel("Behavior Tree", this::renderImGuiWidgets, false, true);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RDXBehaviorTreeFileMenu fileMenu;
   private final RDXBehaviorTreeNodeCreationMenu nodeCreationMenu;
   private final RDXBehaviorTreeFileLoader fileLoader;
   private final ImGuiExpandCollapseRenderer expandCollapseAllRenderer = new ImGuiExpandCollapseRenderer();
   private final RDXBehaviorTreeWidgetsVerticalLayout treeWidgetsVerticalLayout;
   private boolean anyNodeSelected;
   private RDXBehaviorTreeNode<?, ?> selectedNode;
   private boolean enableChildScrollableAreas;

   public RDXBehaviorTree(WorkspaceResourceDirectory treeFilesDirectory,
                          DRCRobotModel robotModel,
                          ROS2SyncedRobotModel syncedRobot,
                          RobotCollisionModel selectionCollisionModel,
                          RDXBaseUI baseUI,
                          RDX3DPanel panel3D,
                          ReferenceFrameLibrary referenceFrameLibrary,
                          FootstepPlannerParametersBasics footstepPlannerParametersBasics)
   {
      this.treeFilesDirectory = treeFilesDirectory;

      nodeBuilder = new RDXBehaviorTreeNodeBuilder(robotModel,
                                                   syncedRobot,
                                                   selectionCollisionModel,
                                                   baseUI,
                                                   panel3D,
                                                   referenceFrameLibrary,
                                                   footstepPlannerParametersBasics);
      treeRebuilder = new BehaviorTreeExtensionSubtreeRebuilder(this::getRootNode, crdtInfo);
      fileMenu = new RDXBehaviorTreeFileMenu(treeFilesDirectory);

      behaviorTreeState = new BehaviorTreeState(nodeBuilder, treeRebuilder, this::getRootNode, crdtInfo, treeFilesDirectory);
      fileLoader = new RDXBehaviorTreeFileLoader(behaviorTreeState, nodeBuilder);
      nodeCreationMenu = new RDXBehaviorTreeNodeCreationMenu(this, treeFilesDirectory, referenceFrameLibrary);
      treeWidgetsVerticalLayout = new RDXBehaviorTreeWidgetsVerticalLayout(this);
      baseUI.getImGuiPanelManager().addPanel(panel);
   }

   public void loadBehaviorFromFile(String fileName)
   {
      nodeCreationMenu.loadBehaviorNodeFromFileName(fileName);
   }

   public void deleteNode(RDXBehaviorTreeNode<?, ?> node)
   {
      treeWidgetsVerticalLayout.deleteNode(node);
   }

   public void createAndSetupDefault(RDXBaseUI baseUI)
   {
      baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables, RDXSceneLevel.VIRTUAL);
      baseUI.getVRManager().getContext().addVRPickCalculator(this::calculateVRPick);
      baseUI.getVRManager().getContext().addVRInputProcessor(this::processVRInput);
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(this::calculate3DViewPick);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::process3DViewInput);
   }

   public void update()
   {
      // Perform any modifications we made in the last tick.
      behaviorTreeState.modifyTreeTopology();

      idToNodeMap.clear();

      if (rootNode != null)
      {
         updateCaches(rootNode);
         update(rootNode);
      }
   }

   private void updateCaches(RDXBehaviorTreeNode<?, ?> node)
   {
      idToNodeMap.put(node.getState().getID(), node);

      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         updateCaches(child);
      }
   }

   private void update(RDXBehaviorTreeNode<?, ?> node)
   {
      node.update();

      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         update(child);
      }
   }

   private void calculateVRPick(RDXVRContext vrContext)
   {
      if (rootNode != null)
         calculateVRPick(vrContext, rootNode);
   }

   private void calculateVRPick(RDXVRContext vrContext, RDXBehaviorTreeNode<?, ?> node)
   {
      node.calculateVRPick(vrContext);

      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         calculateVRPick(vrContext, child);
      }
   }

   private void processVRInput(RDXVRContext vrContext)
   {
      if (rootNode != null)
         processVRInput(vrContext, rootNode);
   }

   private void processVRInput(RDXVRContext vrContext, RDXBehaviorTreeNode<?, ?> node)
   {
      node.processVRInput(vrContext);

      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         processVRInput(vrContext, child);
      }
   }

   public void renderImGuiWidgets()
   {
      renderImGuiWidgetsPre();
      renderImGuiWidgetsPost();
   }

   protected void renderImGuiWidgetsPre()
   {
      ImGui.beginMenuBar();
      fileMenu.renderFileMenu(rootNode, nodeCreationMenu);
   }

   protected void renderImGuiWidgetsPost()
   {
      ImGui.endMenuBar();

      if (rootNode != null)
      {
         if (rootNode instanceof RDXActionSequence actionSequence)
         {
            actionSequence.renderExecutionControlAndProgressWidgets();
         }

         float cursorYAfterControlWidgets = ImGui.getCursorPosY();

         anyNodeSelected = false;
         RDXBehaviorTreeTools.runForSubtreeNodes(rootNode, node -> anyNodeSelected |= node.getSelected());

         float titleHeight = ImGui.getFrameHeightWithSpacing();
         float menuBarHeight = ImGui.getFrameHeightWithSpacing();
         float windowHeight = ImGui.getWindowHeight();
         float availableHeight = windowHeight - titleHeight - menuBarHeight - cursorYAfterControlWidgets;
         // There are ~9 rows of stuff in the screw primitive action settings,
         // which is the tallest one currently. We could think of ways to improve on this.
         float tallestNodeSettings = 9 * ImGui.getFrameHeightWithSpacing();

         // 60% seems to be the desirable ratio for the visible area
         // of the tree view vs the settings area
         float treeExplorerPercentage = 0.6f;
         float treeExplorerHeight = availableHeight * treeExplorerPercentage;
         float nodeSettingsHeight = availableHeight * (1.0f - treeExplorerPercentage);

         float treeContentStartY = ImGui.getCursorPosY();

         if (enableChildScrollableAreas)
            ImGui.beginChild(labels.get("Tree Explorer Scroll Area"), 0.0f, treeExplorerHeight);

         if (expandCollapseAllRenderer.render(false, true))
            expandCollapseAll(true, rootNode);
         if (expandCollapseAllRenderer.getIsHovered())
            ImGui.setTooltip("Expand all nodes");
         ImGui.sameLine();
         if (expandCollapseAllRenderer.render(true, true))
            expandCollapseAll(false, rootNode);
         if (expandCollapseAllRenderer.getIsHovered())
            ImGui.setTooltip("Collapse all nodes");

         treeWidgetsVerticalLayout.renderImGuiWidgets(rootNode);

         boolean updatedEnableChildScrollableAreas;
         float treeContentHeight;
         if (enableChildScrollableAreas)
         {
            float scrollMaxY = ImGui.getScrollMaxY();
            float childWindowHeight = ImGui.getWindowHeight();
            if (scrollMaxY == 0.0)
               treeContentHeight = ImGui.getCursorPosY();
            else
               treeContentHeight = childWindowHeight + scrollMaxY;
         }
         else
         {
            treeContentHeight = ImGui.getCursorPosY() - treeContentStartY;
         }
         updatedEnableChildScrollableAreas = windowHeight - treeContentStartY - treeContentHeight < tallestNodeSettings;

         if (enableChildScrollableAreas)
            ImGui.endChild();

         enableChildScrollableAreas = updatedEnableChildScrollableAreas;

         ImGui.spacing();
         ImGui.spacing();

         if (rootNode != null) // It can become null above
         {
            anyNodeSelected = false;
            RDXBehaviorTreeTools.runForSubtreeNodes(rootNode, node ->
            {
               anyNodeSelected |= node.getSelected();
               if (node.getSelected())
                  selectedNode = node;
            });

            if (anyNodeSelected)
               ImGuiTools.separatorText("Node Settings > \"%s\"".formatted(selectedNode.getDefinition().getName()));
            else
               ImGuiTools.separatorText("Node Settings");

            if (enableChildScrollableAreas)
               ImGui.beginChild(labels.get("Node Settings Scroll Area"), 0.0f, nodeSettingsHeight);

            renderSelectedNodeSettingsWidgets(rootNode);

            if (enableChildScrollableAreas)
               ImGui.endChild();

            if (ImGui.isWindowHovered() && ImGui.getIO().getKeyCtrl() && ImGui.isKeyPressed('S'))
            {
               RDXBaseUI.pushNotification("Saving %s".formatted(rootNode.getDefinition().getName()));
               rootNode.getDefinition().saveToFile();
            }
         }
      }
      else
      {
         ImGui.pushFont(ImGuiTools.getMediumFont());
         ImGui.text("Add a root node:");
         ImGui.popFont();
         nodeCreationMenu.renderImGuiWidgets(rootNode, BehaviorTreeNodeInsertionType.INSERT_ROOT);
      }
   }

   private void expandCollapseAll(boolean expandOrCollapse, RDXBehaviorTreeNode<?, ?> node)
   {
      node.setTreeWidgetExpanded(expandOrCollapse);

      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         expandCollapseAll(expandOrCollapse, child);
      }
   }

   private void renderSelectedNodeSettingsWidgets(RDXBehaviorTreeNode<?, ?> node)
   {
      if (node.getSelected())
         node.renderNodeSettingsWidgets();

      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         renderSelectedNodeSettingsWidgets(child);
      }
   }

   private void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (rootNode != null)
         calculate3DViewPick(input, rootNode);
   }

   private void calculate3DViewPick(ImGui3DViewInput input, RDXBehaviorTreeNode<?, ?> node)
   {
      node.calculate3DViewPick(input);

      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         calculate3DViewPick(input, child);
      }
   }

   private void process3DViewInput(ImGui3DViewInput input)
   {
      if (rootNode != null)
         process3DViewInput(input, rootNode);
   }

   private void process3DViewInput(ImGui3DViewInput input, RDXBehaviorTreeNode<?, ?> node)
   {
      node.process3DViewInput(input);

      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         process3DViewInput(input, child);
      }
   }

   private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (rootNode != null)
         getRenderables(renderables, pool, rootNode);
   }

   private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, RDXBehaviorTreeNode<?, ?> node)
   {
      node.getRenderables(renderables, pool);

      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         getRenderables(renderables, pool, child);
      }
   }

   public void destroy()
   {
      RDXBaseUI.getInstance().getPrimaryScene().removeRenderable(this);
      RDXBaseUI.getInstance().getVRManager().getContext().removeVRPickCalculator(this);
      RDXBaseUI.getInstance().getVRManager().getContext().removeVRInputProcessor(this);
      RDXBaseUI.getInstance().getPrimary3DPanel().removeImGui3DViewPickCalculator(this);
      RDXBaseUI.getInstance().getPrimary3DPanel().removeImGui3DViewInputProcessor(this);
   }

   public BehaviorTreeState getBehaviorTreeState()
   {
      return behaviorTreeState;
   }

   public void setRootNode(BehaviorTreeNodeLayer<?, ?, ?, ?> rootNode)
   {
      this.rootNode = (RDXBehaviorTreeNode<?, ?>) rootNode;
   }

   public RDXBehaviorTreeNode<?, ?> getRootNode()
   {
      return rootNode;
   }

   public RDXBehaviorTreeFileLoader getFileLoader()
   {
      return fileLoader;
   }

   public RDXBehaviorTreeNodeCreationMenu getNodeCreationMenu()
   {
      return nodeCreationMenu;
   }

   public RDXBehaviorTreeNodeBuilder getNodeBuilder()
   {
      return nodeBuilder;
   }

   public WorkspaceResourceDirectory getTreeFilesDirectory()
   {
      return treeFilesDirectory;
   }
}
