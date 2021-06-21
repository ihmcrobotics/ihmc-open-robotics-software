package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.extension.imnodes.ImNodes;
import imgui.flag.ImGuiDir;
import imgui.type.ImBoolean;
import us.ihmc.behaviors.tools.behaviorTree.*;
import us.ihmc.euclid.tuple2D.Point2D32;
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

   private final ImGuiBehaviorTreePanel treePanel;
   private final GDXBehaviorUIInterface tree;

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

      tree = new ExampleSequenceGDXBehaviorUIInterface();
      ExampleFallbackGDXBehaviorUIInterface node = new ExampleFallbackGDXBehaviorUIInterface();
      node.addChild(new ExampleAbstractGDXBehaviorUIInterface()
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
      node.addChild(new ExampleAbstractGDXBehaviorUIInterface()
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
      node.addChild(new ExampleAbstractGDXBehaviorUIInterface()
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

      tree.addChild(new ExampleAbstractGDXBehaviorUIInterface()
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

      treePanel = new ImGuiBehaviorTreePanel("Test");

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
            baseUI.getSceneManager().addModelInstance(new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3)));
            baseUI.getImGuiDockingSetup().splitAdd("Behavior Tree Development UI", ImGuiDir.Right, 0.20);

            ImNodes.createContext();
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();
            Point2D32 point = new Point2D32();
            treePanel.renderAsWindow(tree);

            ImGui.begin("Tree Control");

            ImGui.text("Fallback Node");
            ImGui.checkbox("FB Primary", FALLBACK_PRIMARY);
            ImGui.checkbox("FB Secondary", FALLBACK_SECONDARY);
            ImGui.checkbox("FB Tertiary", FALLBACK_TERTIARY);

            ImGui.text("Sequence Node");
            ImGui.checkbox("S Secondary", SEQUENCE_SECONDARY);

            ImGui.end();

            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            ImNodes.destroyContext();
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
