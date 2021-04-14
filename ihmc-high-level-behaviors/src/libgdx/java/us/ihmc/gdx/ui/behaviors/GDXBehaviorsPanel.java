package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiInputTextFlags;
import imgui.internal.ImGui;
import imgui.type.ImString;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.BehaviorModule;
import us.ihmc.behaviors.RemoteBehaviorInterface;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIRegistry;
import us.ihmc.gdx.vr.GDXVRManager;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.messager.kryo.KryoMessager;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.utilities.ros.RosMainNode;

import java.util.function.Supplier;

public class GDXBehaviorsPanel implements RenderableProvider
{
   private final String windowName = ImGuiTools.uniqueLabel(getClass(), "Behaviors");
   private final GDXBehaviorUIRegistry behaviorRegistry;
   private final ImString messagerHost = new ImString("localhost", 100);
   private volatile boolean disconnecting = false;
   private volatile boolean connecting = false;
   private Messager messager = null;

   private final GDXBuildingExplorationBehaviorUI buildingExplorationUI;
   private final ImGuiGDXLookAndStepBehaviorUI lookAndStepUI;

   public GDXBehaviorsPanel(RosMainNode ros1Node,
                            ROS2Node ros2Node,
                            Supplier<? extends DRCRobotModel> robotModelSupplier,
                            GDXBehaviorUIRegistry behaviorRegistry)
   {
      this.behaviorRegistry = behaviorRegistry;

      buildingExplorationUI = new GDXBuildingExplorationBehaviorUI();
      lookAndStepUI = new ImGuiGDXLookAndStepBehaviorUI(new BehaviorHelper(robotModelSupplier.get(), null, ros1Node, ros2Node));
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      buildingExplorationUI.create(baseUI.getVRManager());
      baseUI.getSceneManager().addRenderableProvider(buildingExplorationUI, GDXSceneLevel.VIRTUAL);

      lookAndStepUI.create(baseUI);
      baseUI.getSceneManager().addRenderableProvider(lookAndStepUI, GDXSceneLevel.VIRTUAL);
   }

   public void handleVREvents(GDXVRManager vrManager)
   {
      buildingExplorationUI.handleVREvents(vrManager);
   }

   public void render()
   {
      ImGui.begin(windowName);
      if (connecting)
      {
         ImGui.text("Connecting...");
         if (messager.isMessagerOpen())
         {
            connecting = false;
            updateMessager();
         }
      }
      else if (disconnecting)
      {
         ImGui.text("Disconnecting...");
      }
      else if (messager == null)
      {
         int flags = ImGuiInputTextFlags.None;
         flags += ImGuiInputTextFlags.CallbackResize;
         ImGui.inputText(ImGuiTools.uniqueIDOnly(getClass(), "messagerHost"), messagerHost, flags);
         ImGui.sameLine();
         if (ImGui.button("Connect"))
         {
            connectViaKryo(messagerHost.get());
         }

         SharedMemoryMessager potentialSharedMemoryMessager = BehaviorModule.getSharedMemoryMessager();
         if (potentialSharedMemoryMessager != null && potentialSharedMemoryMessager.isMessagerOpen())
         {
            if (ImGui.button("Use shared memory"))
            {
               messager = potentialSharedMemoryMessager;
               updateMessager();
            }
         }
      }
      else
      {
         if (messager.isMessagerOpen())
         {
            if (messager instanceof SharedMemoryMessager)
            {
               ImGui.text("Using shared memory messager.");
            }
            else if (messager instanceof KryoMessager)
            {
               ImGui.text("Connected to: " + messagerHost.get());
            }

            if (ImGui.button(ImGuiTools.uniqueLabel(this, "Disconnect")))
            {
               if (messager instanceof SharedMemoryMessager)
               {
                  messager = null;
               }
               else
               {
                  disconnect();
               }
            }
         }
      }
      ImGui.end();
   }

   public void connectViaKryo(String hostname)
   {
      messagerHost.set(hostname);
      messager = RemoteBehaviorInterface.createForUI(behaviorRegistry, hostname);
      connecting = true;
   }

   public void disconnect()
   {
      disconnecting = true;
      ThreadTools.startAThread(() ->
      {
         ExceptionTools.handle(messager::closeMessager, DefaultExceptionHandler.RUNTIME_EXCEPTION);
         messager = null;
         disconnecting = false;
      }, "MessagerDisconnectionThread");
   }

   private void updateMessager()
   {
      buildingExplorationUI.setMessager(messager);
      lookAndStepUI.setMessager(messager);
   }

   public void destroy()
   {
      disconnect();
      lookAndStepUI.destroy();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      buildingExplorationUI.getRenderables(renderables, pool);
      lookAndStepUI.getRenderables(renderables, pool);
   }

   public String getWindowName()
   {
      return windowName;
   }
}
