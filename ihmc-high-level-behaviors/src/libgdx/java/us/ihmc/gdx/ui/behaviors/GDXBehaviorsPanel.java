package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiInputTextFlags;
import imgui.internal.ImGui;
import imgui.type.ImInt;
import imgui.type.ImString;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.BehaviorModule;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.MessagerHelper;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIRegistry;
import us.ihmc.gdx.vr.GDXVRManager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.ros2.ROS2Node;

import java.time.LocalDateTime;
import java.util.LinkedList;
import java.util.function.Supplier;

import static us.ihmc.behaviors.BehaviorModule.API.StatusLog;

public class GDXBehaviorsPanel implements RenderableProvider
{
   private final String windowName = ImGuiTools.uniqueLabel(getClass(), "Behaviors");
   private final GDXBehaviorUIRegistry behaviorRegistry;
   private final ImString behaviorModuleHost = new ImString("localhost", 100);
   private volatile boolean messagerConnecting = false;
   private final MessagerHelper messagerHelper;
   private volatile boolean yoClientDisconnecting = false;
   private volatile boolean yoClientConnecting = false;
   private YoVariableClient yoVariableClient = null;

   private final GDXBuildingExplorationBehaviorUI buildingExplorationUI;
   private final ImGuiGDXLookAndStepBehaviorUI lookAndStepUI;
   private final BehaviorHelper behaviorHelper;
   private final LinkedList<String> logArray = new LinkedList<>();
   private final ImInt selectedLogEntry = new ImInt();

   public GDXBehaviorsPanel(String ros1NodeName,
                            ROS2Node ros2Node,
                            Supplier<? extends DRCRobotModel> robotModelSupplier,
                            GDXBehaviorUIRegistry behaviorRegistry)
   {
      this.behaviorRegistry = behaviorRegistry;

      behaviorHelper = new BehaviorHelper(robotModelSupplier.get(), ros1NodeName, ros2Node);
      messagerHelper = behaviorHelper.getMessagerHelper();
      buildingExplorationUI = new GDXBuildingExplorationBehaviorUI(messagerHelper);
      lookAndStepUI = new ImGuiGDXLookAndStepBehaviorUI(behaviorHelper);

      logArray.addLast("Log started at " + LocalDateTime.now());
      behaviorHelper.subscribeViaCallback(StatusLog, logEntry ->
      {
         synchronized (logArray)
         {
            logArray.addLast(logEntry.getRight());
         }
      });
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
      if (messagerConnecting)
      {
         ImGui.text("Connecting...");
         if (messagerHelper.isConnected())
         {
            messagerConnecting = false;
         }
      }
      else if (messagerHelper.isDisconnecting())
      {
         ImGui.text("Disconnecting...");
      }
      else if (!messagerHelper.isConnected())
      {
         int flags = ImGuiInputTextFlags.None;
         flags += ImGuiInputTextFlags.CallbackResize;
         ImGui.inputText(ImGuiTools.uniqueIDOnly(getClass(), "messagerHost"), behaviorModuleHost, flags);
         ImGui.sameLine();
         if (ImGui.button("Connect"))
         {
            connectViaKryo(behaviorModuleHost.get());
         }

         SharedMemoryMessager potentialSharedMemoryMessager = BehaviorModule.getSharedMemoryMessager();
         if (potentialSharedMemoryMessager != null && potentialSharedMemoryMessager.isMessagerOpen())
         {
            if (ImGui.button("Use shared memory"))
            {
               messagerHelper.connectViaSharedMemory(potentialSharedMemoryMessager);
            }
         }
      }
      else
      {
         if (messagerHelper.isConnected())
         {
            if (messagerHelper.isUsingSharedMemory())
            {
               ImGui.text("Using shared memory messager.");
            }
            else
            {
               ImGui.text("Connected to: " + behaviorModuleHost.get());
            }

            if (ImGui.button(ImGuiTools.uniqueLabel(this, "Disconnect")))
            {
               disconnect();
            }

            lookAndStepUI.renderWidgetsOnly();
         }
      }

      synchronized (logArray)
      {
         selectedLogEntry.set(logArray.size() - 1);
         ImGui.text("Behavior status log:");
         ImGui.pushItemWidth(ImGui.getWindowWidth() - 10);
         int numLogEntriesToShow = 15;
         while (logArray.size() > numLogEntriesToShow)
            logArray.removeFirst();
         ImGui.listBox("", selectedLogEntry, logArray.toArray(new String[0]), numLogEntriesToShow);
      }
      ImGui.popItemWidth();

      ImGui.end();
   }

   public void connectViaKryo(String hostname)
   {
      behaviorModuleHost.set(hostname);
      messagerHelper.connectViaKryo(behaviorModuleHost.get(), NetworkPorts.BEHAVIOR_MODULE_MESSAGER_PORT.getPort());
      messagerConnecting = true;
   }

   public void disconnect()
   {
      messagerHelper.disconnect();
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
