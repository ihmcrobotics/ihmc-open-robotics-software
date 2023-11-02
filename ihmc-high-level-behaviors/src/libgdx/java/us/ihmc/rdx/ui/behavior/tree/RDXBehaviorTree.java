package us.ihmc.rdx.ui.behavior.tree;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import imgui.ImGui;
import imgui.ImVec2;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeExtensionSubtreeDestroy;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeExtensionSubtreeRebuilder;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeNodeExtensionAddAndFreeze;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeNodeSetRoot;
import us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTreeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiTreeRenderer;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXBehaviorTree
{
   private final CRDTInfo crdtInfo = new CRDTInfo(ROS2ActorDesignation.OPERATOR, (int) ROS2BehaviorTreeState.SYNC_FREQUENCY);
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
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RDXBehaviorTreeFileMenu fileMenu;
   private final RDXBehaviorTreeNodesMenu nodesMenu;
   private final RDXBehaviorTreeFileLoader fileLoader;
   private final ImVec2 expandButtonSize = new ImVec2();
   private final ImGuiTreeRenderer treeRenderer = new ImGuiTreeRenderer();

   public RDXBehaviorTree(WorkspaceResourceDirectory treeFilesDirectory,
                          DRCRobotModel robotModel,
                          ROS2SyncedRobotModel syncedRobot,
                          RobotCollisionModel selectionCollisionModel,
                          RDXBaseUI baseUI,
                          RDX3DPanel panel3D,
                          ReferenceFrameLibrary referenceFrameLibrary,
                          FootstepPlannerParametersBasics footstepPlannerParametersBasics)
   {
      nodeBuilder = new RDXBehaviorTreeNodeBuilder(robotModel,
                                                   syncedRobot,
                                                   selectionCollisionModel,
                                                   baseUI,
                                                   panel3D,
                                                   referenceFrameLibrary,
                                                   footstepPlannerParametersBasics);
      treeRebuilder = new BehaviorTreeExtensionSubtreeRebuilder(this::getRootNode, crdtInfo);
      fileMenu = new RDXBehaviorTreeFileMenu(treeFilesDirectory);
      nodesMenu = new RDXBehaviorTreeNodesMenu(treeFilesDirectory);

      behaviorTreeState = new BehaviorTreeState(nodeBuilder, treeRebuilder, this::getRootNode, crdtInfo);
      fileLoader = new RDXBehaviorTreeFileLoader(behaviorTreeState, nodeBuilder);
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
      if (nodesMenu.getLoadFileRequest().poll())
      {
         RDXBehaviorTreeNode<?, ?> selectedNode = null; // TODO: Traverse tree for selected node, probably RDXBTNode

         behaviorTreeState.modifyTree(modificationQueue ->
         {
            BehaviorTreeNodeExtension<?, ?, ?, ?> loadedNode = fileLoader.loadFromFile(nodesMenu.getLoadFileRequest().read(), modificationQueue);

            if (selectedNode == null)
            {
               modificationQueue.accept(new BehaviorTreeNodeSetRoot(loadedNode, newRootNode -> rootNode = (RDXBehaviorTreeNode<?, ?>) newRootNode));
               behaviorTreeState.freeze();
            }
            else
               modificationQueue.accept(new BehaviorTreeNodeExtensionAddAndFreeze(loadedNode, selectedNode));
         });
      }

      if (nodesMenu.getDeleteRootNode().poll() && rootNode != null) // poll must be first to get cleared every time
      {
         behaviorTreeState.modifyTree(modificationQueue ->
         {
            modificationQueue.accept(new BehaviorTreeExtensionSubtreeDestroy(rootNode));
         });

         setRootNode(null);
         behaviorTreeState.freeze();
      }

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

   protected void renderImGuiWidgetsPre()
   {
      ImGui.beginMenuBar();
      fileMenu.renderFileMenu();
      nodesMenu.renderNodesMenu();
      ImGui.endMenuBar();

      for (int i = 0; i < 100; i++)
         ImGui.spacing();

      if (ImGui.button(labels.get("[+]")))
         expandCollapseAll(true, rootNode);
      ImGuiTools.previousWidgetTooltip("Expand all nodes");
      ImGui.getItemRectSize(expandButtonSize);
      ImGui.sameLine();
      if (ImGui.button(labels.get("[-]"), expandButtonSize.x, expandButtonSize.y))
         expandCollapseAll(false, rootNode);
      ImGuiTools.previousWidgetTooltip("Collapse all nodes");
      ImGui.sameLine();
   }

   protected void renderImGuiWidgetsPost()
   {
      if (rootNode != null)
         renderImGuiWidgetsAsTree(rootNode);
   }

   private void expandCollapseAll(boolean expandOrCollapse, RDXBehaviorTreeNode<?, ?> node)
   {
      node.getExpandCollapseRequest().set(expandOrCollapse);

      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         expandCollapseAll(expandOrCollapse, child);
      }
   }

   private void renderImGuiWidgetsAsTree(RDXBehaviorTreeNode<?, ?> node)
   {
      treeRenderer.render(node.getState().getID(), node.getDefinition().getDescription(), () ->
      {
         node.renderImGuiWidgets();

         for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
         {
            renderImGuiWidgetsAsTree(child);
         }
      }, node.getExpandCollapseRequest());
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

   public void setRootNode(BehaviorTreeNodeExtension<?, ?, ?, ?> rootNode)
   {
      this.rootNode = (RDXBehaviorTreeNode<?, ?>) rootNode;
   }

   public RDXBehaviorTreeNode<?, ?> getRootNode()
   {
      return rootNode;
   }
}
