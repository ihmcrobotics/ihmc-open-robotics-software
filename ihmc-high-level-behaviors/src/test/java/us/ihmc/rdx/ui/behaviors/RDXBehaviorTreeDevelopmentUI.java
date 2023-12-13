package us.ihmc.rdx.ui.behaviors;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.behaviors.behaviorTree.*;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.tree.RDXImNodesBehaviorTreeUI;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIInterface;
import us.ihmc.log.LogTools;

import java.util.Timer;
import java.util.TimerTask;

public class RDXBehaviorTreeDevelopmentUI
{
   private final String WINDOW_NAME = "Behavior Tree Development UI";

   private final RDXBaseUI baseUI;

   private final RDXImNodesBehaviorTreeUI treePanel;
   private final BehaviorTreeNodeExecutor tree;
   private final RDXBehaviorUIInterface treeGui;

   private final Timer timer;

   public static ImBoolean FALLBACK_PRIMARY = new ImBoolean(true);
   public static ImBoolean FALLBACK_SECONDARY = new ImBoolean(true);
   public static ImBoolean FALLBACK_TERTIARY = new ImBoolean(true);
   public static ImBoolean SEQUENCE_SECONDARY = new ImBoolean(true);

   public RDXBehaviorTreeDevelopmentUI()
   {
      LogTools.info("Starting UI");
      baseUI = new RDXBaseUI(WINDOW_NAME);

      timer = new Timer();

      tree = new SequenceNode();
      BehaviorTreeNodeExecutor node = new FallbackNode();
      node.getChildren().add(new LocalOnlyBehaviorTreeNodeExecutor()
      {
         @Override
         public BehaviorTreeNodeStatus determineStatus()
         {
            if (FALLBACK_PRIMARY.get())
               return BehaviorTreeNodeStatus.SUCCESS;
            else
               return BehaviorTreeNodeStatus.FAILURE;
         }

         public String getName()
         {
            return "Primary";
         }
      });
      node.getChildren().add(new LocalOnlyBehaviorTreeNodeExecutor()
      {
         @Override
         public BehaviorTreeNodeStatus determineStatus()
         {
            if (FALLBACK_SECONDARY.get())
               return BehaviorTreeNodeStatus.SUCCESS;
            else
               return BehaviorTreeNodeStatus.FAILURE;
         }

         public String getName()
         {
            return "Secondary";
         }
      });
      node.getChildren().add(new LocalOnlyBehaviorTreeNodeExecutor()
      {
         @Override
         public BehaviorTreeNodeStatus determineStatus()
         {
            if (FALLBACK_TERTIARY.get())
               return BehaviorTreeNodeStatus.SUCCESS;
            else
               return BehaviorTreeNodeStatus.FAILURE;
         }

         public String getName()
         {
            return "Tertiary";
         }
      });

      tree.getChildren().add(node);

      tree.getChildren().add(new LocalOnlyBehaviorTreeNodeExecutor()
      {
         @Override
         public BehaviorTreeNodeStatus determineStatus()
         {
            if (SEQUENCE_SECONDARY.get())
               return BehaviorTreeNodeStatus.SUCCESS;
            else
               return BehaviorTreeNodeStatus.FAILURE;
         }

         public String getName()
         {
            return "Other thing";
         }
      });

      treeGui = new ExampleSimpleNodeInterface("SequenceNode");
      treePanel = new RDXImNodesBehaviorTreeUI();
      treePanel.setRootNode(treeGui);
      RDXBehaviorUIInterface nodeGui = new ExampleSimpleNodeInterface("FallbackNode");
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
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));
            baseUI.getImGuiPanelManager().addPanel("Tree Control", this::renderPanel);
            baseUI.getImGuiPanelManager().addPanel("Behavior Tree Panel", () -> {
               treeGui.syncTree(tree);
               treePanel.renderImGuiWidgets();
            });

            treePanel.create();
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();
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
            treePanel.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      RDXBehaviorTreeDevelopmentUI ui = new RDXBehaviorTreeDevelopmentUI();
      ui.launch();
   }
}
