package us.ihmc.rdx.behaviorTree;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import imgui.flag.ImGuiMouseCursor;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTree;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionType;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.behaviorTree.actions.RDXActionProgressWidgetsManager.Type;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.ArrayDeque;
import java.util.Deque;

public class RDXBehaviorTree extends BehaviorTree<RDXBehaviorTreeRootNode, RDXBehaviorTreeNode<?, ?>>
{
   public static final RDXBehaviorTreeSettings SETTINGS = new RDXBehaviorTreeSettings();
   /**
    * Useful for accessing nodes by ID instead of searching.
    * Also, sometimes, the tree will be disassembled and this is used in putting it
    * back together.
    */
   private transient final TLongObjectMap<RDXBehaviorTreeNode<?, ?>> idToNodeMap = new TLongObjectHashMap<>();
   private final RDXPanel panel = new RDXPanel("Behavior Tree", this::renderImGuiWidgets, false, true);
   private final RDXPanel scenePanel = new RDXPanel("Scene", this::renderScenePanel);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RDXBehaviorTreeNodeCreationMenu nodeCreationMenu;
   private final RDXBehaviorTreeWidgetsVerticalLayout treeWidgetsVerticalLayout;
   private boolean anyNodeSelected;
   private RDXBehaviorTreeNode<?, ?> selectedNode;
   private boolean draggingDivider;
   private boolean shouldSave = false;
   private final Deque<Runnable> preUpdateOperationQueue = new ArrayDeque<>();

   public RDXBehaviorTree(WorkspaceResourceDirectory treeFilesDirectory,
                          ROS2SyncedRobotModel syncedRobot,
                          ROS2PeerClockOffsetEstimator peerClockEstimator,
                          RobotCollisionModel selectionCollisionModel,
                          RDXBaseUI baseUI,
                          RDX3DPanel panel3D)
   {
      super(syncedRobot, ROS2ActorDesignation.OPERATOR, peerClockEstimator, treeFilesDirectory, new RDXBehaviorTreeNodeBuilder());

      ((RDXBehaviorTreeNodeBuilder) getNodeBuilder()).initialize(this,
                                                                 saveFileDirectory,
                                                                 syncedRobot,
                                                                 selectionCollisionModel,
                                                                 baseUI,
                                                                 panel3D);

      nodeCreationMenu = new RDXBehaviorTreeNodeCreationMenu(this, treeFilesDirectory);
      treeWidgetsVerticalLayout = new RDXBehaviorTreeWidgetsVerticalLayout(this);
      panel.addChild(scenePanel);
      baseUI.getImGuiPanelManager().addPanel(panel);
   }

   public void createAndSetupDefault(RDXBaseUI baseUI)
   {
      baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables, RDXSceneLevel.VIRTUAL);
      baseUI.getVRManager().getContext().addVRPickCalculator(this::calculateVRPick);
      baseUI.getVRManager().getContext().addVRInputProcessor(this::processVRInput);
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(this::calculate3DViewPick);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::process3DViewInput);
      baseUI.getStatusBar().add(() -> RDXBehaviorTreeTimeline.render(rootNode),
                                RDXBehaviorTreeTimeline::heightNeeded,
                                RDXBehaviorTreeTimeline::renderIcon);
   }

   public void update()
   {
      while (!preUpdateOperationQueue.isEmpty())
         preUpdateOperationQueue.poll().run();

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
         updateCaches(child);
   }

   private void update(RDXBehaviorTreeNode<?, ?> node)
   {
      node.update();

      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
         update(child);
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
         calculateVRPick(vrContext, child);
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
         processVRInput(vrContext, child);
   }

   public void renderImGuiWidgets()
   {
      renderImGuiWidgetsPre();
      ImGui.endMenuBar();
      renderImGuiWidgetsPost();
   }

   protected void renderImGuiWidgetsPre()
   {
      shouldSave = false;
      ImGui.beginMenuBar();
      if (ImGui.beginMenu(labels.get("File")))
      {
         if (rootNode == null)
         {
            if (ImGui.menuItem(labels.get("Refresh File List")))
               nodeCreationMenu.reindexDirectory();
         }
         else
         {
            if (ImGui.menuItem(labels.get("Save All"), "Ctrl + S"))
            {
               RDXBaseUI.pushNotification("Saving %s".formatted(rootNode.getDefinition().getName()));
               rootNode.getDefinition().saveToFile();
            }
            if (ImGui.menuItem(labels.get("Undo All Non-topological Changes")))
            {
               RDXBaseUI.pushNotification("Undoing all non-topological behavior tree changes");
               rootNode.getDefinition().undoAllNontopologicalChanges();
            }
         }

         ImGui.endMenu();
      }
      if (ImGui.beginMenu(labels.get("View")))
      {
         if (rootNode != null)
         {
            ImGui.text("Progress Widgets:");
            if (ImGui.menuItem(labels.get("Time Only"), null, SETTINGS.getProgressWidgetsType() == Type.TIME_ONLY))
               SETTINGS.setProgressWidgetsType(Type.TIME_ONLY);
            if (ImGui.menuItem(labels.get("Progress Bars"), null, SETTINGS.getProgressWidgetsType() == Type.PROGRESS_BARS))
               SETTINGS.setProgressWidgetsType(Type.PROGRESS_BARS);
            if (ImGui.menuItem(labels.get("Scrolling Plots"), null, SETTINGS.getProgressWidgetsType() == Type.SCROLLING_PLOTS))
               SETTINGS.setProgressWidgetsType(Type.SCROLLING_PLOTS);
         }

         ImGui.endMenu();
      }
   }

   protected void renderImGuiWidgetsPost()
   {
      if (rootNode != null)
      {
         rootNode.renderExecutionControlAndProgressWidgets();

         anyNodeSelected = false;
         RDXBehaviorTreeTools.runForSubtreeNodes(rootNode, node ->
         {
            anyNodeSelected |= node.getSelected();
            if (node.getSelected())
               selectedNode = node;
         });

         shouldSave |= ImGui.isWindowHovered() && ImGui.getIO().getKeyCtrl() && ImGui.isKeyPressed('S');

         float remainingHeight = ImGui.getContentRegionAvailY();
         float treeExplorerPercentage = SETTINGS.getTreeExplorerHeightPercentage();
         float treeExplorerHeight = anyNodeSelected ? remainingHeight * treeExplorerPercentage : remainingHeight;

         ImGui.beginChild(labels.get("Tree Explorer Scroll Area"), 0.0f, treeExplorerHeight);
         treeWidgetsVerticalLayout.renderImGuiWidgets();
         shouldSave |= ImGui.isWindowHovered() && ImGui.getIO().getKeyCtrl() && ImGui.isKeyPressed('S');
         ImGui.endChild();

         if (rootNode != null && anyNodeSelected) // It can become null above
         {
            float dividerHeight = ImGui.getFrameHeight();
            float closeOffsetY = ImGui.getCursorScreenPosY();
            float closeOffsetX = ImGui.getCursorScreenPosX() + ImGui.getColumnWidth() - dividerHeight;
            boolean closeHovered = false;

            if (ImGuiTools.isItemHovered(ImGui.getColumnWidth(), dividerHeight) || draggingDivider)
            {
               if (!draggingDivider || ImGui.isMouseDown(ImGuiMouseButton.Left))
               {
                  boolean dragging = ImGui.isMouseDragging(ImGuiMouseButton.Left, 0.1f);
                  if (dragging) // default threshold 0.3f is too much
                  {
                     treeExplorerPercentage += ImGui.getIO().getMouseDeltaY() / remainingHeight;
                     treeExplorerPercentage = (float) MathTools.clamp(treeExplorerPercentage, 0.05f, 0.95f);
                     SETTINGS.setTreeExplorerHeightPercentage(treeExplorerPercentage);
                     draggingDivider = true;
                  }

                  float mouseX = ImGui.getMousePosX();
                  float mouseY = ImGui.getMousePosY();
                  closeHovered = mouseX >= closeOffsetX && mouseX <= closeOffsetX + dividerHeight && mouseY >= closeOffsetY && mouseY <= closeOffsetY + dividerHeight;

                  if (dragging || !closeHovered)
                  {
                     ImGui.setMouseCursor(ImGuiMouseCursor.ResizeNS);
                     closeHovered = false;
                  }
               }
               else
                  draggingDivider = false;
            }

            // Draw separator with text
            ImGui.dummy(0.0f, ImGui.getFrameHeight()); // Make the lines as tall as when they have and input box
            ImGui.sameLine(0.0f, 0.0f);
            ImGui.alignTextToFramePadding(); // Centers the node descriptions vertically in the frame height area
            float cursorScreenPosX = ImGui.getCursorScreenPosX();
            float cursorScreenPosY = ImGui.getCursorScreenPosY();
            int fontSize = ImGui.getFontSize();
            float itemSpacingX = ImGui.getStyle().getItemSpacingX();
            float lineThickness = fontSize * 0.2f;
            float lineY = ImGui.getFrameHeight() / 2.0f + (lineThickness / 2.0f);
            float initialLineWidth = fontSize * 1.5f;
            int separatorColor = ImGui.getColorU32(ImGuiCol.Separator);
            ImGui.getWindowDrawList().addLine(cursorScreenPosX, cursorScreenPosY + lineY,
                                              cursorScreenPosX + initialLineWidth, cursorScreenPosY + lineY,
                                              separatorColor, lineThickness);
            ImGui.setCursorPosX(ImGui.getCursorPosX() + initialLineWidth + itemSpacingX);
            ImGui.text("Node Settings > \"%s\"".formatted(selectedNode.getDefinition().getName()));
            ImGui.sameLine();
            cursorScreenPosX = ImGui.getCursorScreenPosX();
            ImGui.getWindowDrawList()
                 .addLine(cursorScreenPosX + itemSpacingX, cursorScreenPosY + lineY, closeOffsetX, cursorScreenPosY + lineY, separatorColor, lineThickness);

            if (closeHovered) // Draw circle on hover
            {
               float circleRadius = dividerHeight * 0.43f;
               float circleCenterX = closeOffsetX + dividerHeight / 2.0f;
               float circleCenterY = closeOffsetY + dividerHeight / 2.0f;
               int circleColor = ImGui.getColorU32(ImGuiCol.ButtonHovered);
               ImGui.getWindowDrawList().addCircleFilled(circleCenterX, circleCenterY, circleRadius, circleColor);

               if (ImGuiTools.mouseReleasedWithoutDrag(ImGuiMouseButton.Left, 1.0f))
                  RDXBehaviorTreeTools.runForEntireTree(rootNode, RDXBehaviorTreeNode::clearSelections);
            }

            // Draw close X
            float xSize = dividerHeight * 0.46f;
            float xOffset = (dividerHeight - xSize) / 2.0f;
            int xColor = ImGui.getColorU32(closeHovered ? ImGuiCol.Text : ImGuiCol.TextDisabled);
            float xThickness = 1.0f;
            ImGui.getWindowDrawList().addLine(closeOffsetX + xOffset, closeOffsetY + xOffset,
                                              closeOffsetX + xOffset + xSize, closeOffsetY + xOffset + xSize,
                                              xColor, xThickness);
            ImGui.getWindowDrawList().addLine(closeOffsetX + xOffset, closeOffsetY + xOffset + xSize,
                                              closeOffsetX + xOffset + xSize, closeOffsetY + xOffset,
                                              xColor, xThickness);
            ImGui.dummy(0.0f, dividerHeight);

            ImGui.beginChild(labels.get("Node Settings Scroll Area"), 0.0f, ImGui.getContentRegionAvailY());
            renderSelectedNodeSettingsWidgets(rootNode);
            shouldSave |= ImGui.isWindowHovered() && ImGui.getIO().getKeyCtrl() && ImGui.isKeyPressed('S');
            ImGui.endChild();
         }
      }
      else
      {
         nodeCreationMenu.renderImGuiWidgets(null, BehaviorTreeNodeInsertionType.INSERT_ROOT);
      }

      // Perform any modifications that were made via user interaction.
      modifyTreeTopology();

      if (shouldSave)
      {
         RDXBaseUI.pushNotification("Saving %s".formatted(rootNode.getDefinition().getName()));
         rootNode.getDefinition().saveToFile();
      }
   }

   private void renderScenePanel()
   {
      if (rootNode == null)
      {
         ImGui.text("Root node is null. No scene.");
         return;
      }

      rootNode.getScene().renderImGuiWidgets();
   }

   private void renderSelectedNodeSettingsWidgets(RDXBehaviorTreeNode<?, ?> node)
   {
      if (node.getSelected())
         node.renderNodeSettingsWidgets();

      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
         renderSelectedNodeSettingsWidgets(child);
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
         calculate3DViewPick(input, child);
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
         process3DViewInput(input, child);
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
         getRenderables(renderables, pool, child);
   }

   public void destroy()
   {
      RDXBaseUI.getInstance().getPrimaryScene().removeRenderable(this);
      RDXBaseUI.getInstance().getVRManager().getContext().removeVRPickCalculator(this);
      RDXBaseUI.getInstance().getVRManager().getContext().removeVRInputProcessor(this);
      RDXBaseUI.getInstance().getPrimary3DPanel().removeImGui3DViewPickCalculator(this);
      RDXBaseUI.getInstance().getPrimary3DPanel().removeImGui3DViewInputProcessor(this);
   }

   public RDXBehaviorTreeNodeCreationMenu getNodeCreationMenu()
   {
      return nodeCreationMenu;
   }

   public void addPreUpdateOperation(Runnable runnable)
   {
      preUpdateOperationQueue.add(runnable);
   }
}
