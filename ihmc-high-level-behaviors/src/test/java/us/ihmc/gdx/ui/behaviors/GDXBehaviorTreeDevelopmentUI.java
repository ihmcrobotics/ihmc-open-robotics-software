package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.behaviors.tools.behaviorTree.*;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;
import us.ihmc.log.LogTools;

import java.util.Timer;
import java.util.TimerTask;

public class GDXBehaviorTreeDevelopmentUI
{
   private final String WINDOW_NAME = "Behavior Tree Development UI";

   private final GDXImGuiBasedUI baseUI;

   private final ImGuiNodeEditorBehaviorTreePanel treePanel;
   private final BehaviorTreeControlFlowNode tree;
   private final GDXBehaviorUIInterface treeGui;

   private final Timer timer;

   public static ImBoolean FALLBACK_PRIMARY = new ImBoolean(true);
   public static ImBoolean FALLBACK_SECONDARY = new ImBoolean(true);
   public static ImBoolean FALLBACK_TERTIARY = new ImBoolean(true);
   public static ImBoolean SEQUENCE_SECONDARY = new ImBoolean(true);

   public GDXBehaviorTreeDevelopmentUI()
   {
      LogTools.info("Starting UI");
      baseUI = new GDXImGuiBasedUI(getClass(), "atlas-user-interface", "src/libgdx/resources", WINDOW_NAME);

      timer = new Timer();

      tree = new SequenceNode();
      BehaviorTreeControlFlowNode node = new FallbackNode();
      node.addChild(new BehaviorTreeNode()
      {
         @Override
         public BehaviorTreeNodeStatus tickInternal()
         {
            if (FALLBACK_PRIMARY.get())
               return BehaviorTreeNodeStatus.SUCCESS;
            else
               return BehaviorTreeNodeStatus.FAILURE;
         }

         @Override
         public String getName() {
            return "Primary";
         }
      });
      node.addChild(new BehaviorTreeNode()
      {
         @Override
         public BehaviorTreeNodeStatus tickInternal()
         {
            if (FALLBACK_SECONDARY.get())
               return BehaviorTreeNodeStatus.SUCCESS;
            else
               return BehaviorTreeNodeStatus.FAILURE;
         }

         @Override
         public String getName() {
            return "Secondary";
         }
      });
      node.addChild(new BehaviorTreeNode()
      {
         @Override
         public BehaviorTreeNodeStatus tickInternal()
         {
            if (FALLBACK_TERTIARY.get())
               return BehaviorTreeNodeStatus.SUCCESS;
            else
               return BehaviorTreeNodeStatus.FAILURE;
         }

         @Override
         public String getName() {
            return "Tertiary";
         }
      });

      tree.addChild(node);

      tree.addChild(new BehaviorTreeNode()
      {
         @Override
         public BehaviorTreeNodeStatus tickInternal()
         {
            if (SEQUENCE_SECONDARY.get())
               return BehaviorTreeNodeStatus.SUCCESS;
            else
               return BehaviorTreeNodeStatus.FAILURE;
         }

         @Override
         public String getName() {
            return "Other thing";
         }
      });

      treePanel = new ImGuiNodeEditorBehaviorTreePanel("Test");

      treeGui = new ExampleSimpleNodeInterface("SequenceNode");
      GDXBehaviorUIInterface nodeGui = new ExampleSimpleNodeInterface("FallbackNode");
      nodeGui.addChild(new ExampleSimpleNodeInterface("Primary"));
      nodeGui.addChild(new ExampleSimpleNodeInterface("Secondary"));
      nodeGui.addChild(new ExampleSimpleNodeInterface("Tertiary"));
      treeGui.addChild(nodeGui);
      treeGui.addChild(new ExampleSimpleNodeInterface("Other thing"));

      timer.scheduleAtFixedRate(new TimerTask()
      {
         @Override
         public void run()
         {
            tree.tick();
         }
      }, 1000, 200);
   }

   public void launch()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.get3DSceneManager().addModelInstance(new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3)));
            baseUI.getImGuiPanelManager().addPanel("Tree Control", this::renderPanel);

            treePanel.create();
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();

            treeGui.syncTree(tree);
            treePanel.renderAsWindow(treeGui);
            baseUI.renderEnd();
         }

         private void renderPanel()
         {
            ImGui.text("Fallback Node");
            ImGui.checkbox("FB Primary", FALLBACK_PRIMARY);
            ImGui.checkbox("FB Secondary", FALLBACK_SECONDARY);
            ImGui.checkbox("FB Tertiary", FALLBACK_TERTIARY);

            ImGui.text("Sequence Node");
            ImGui.checkbox("S Secondary", SEQUENCE_SECONDARY);
         }

         @Override
         public void dispose()
         {
            treePanel.dispose();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      GDXBehaviorTreeDevelopmentUI ui = new GDXBehaviorTreeDevelopmentUI();
      ui.launch();
   }
}
