package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.Color;
import imgui.flag.ImGuiInputTextFlags;
import imgui.internal.ImGui;
import imgui.type.ImString;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.logging.log4j.Level;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.BehaviorModule;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.MessagerHelper;
import us.ihmc.behaviors.tools.YoVariableClientHelper;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeControlFlowNode;
import us.ihmc.behaviors.tools.behaviorTree.FallbackNode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIRegistry;
import us.ihmc.gdx.vr.GDXVRManager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.ros2.ROS2Node;

import java.time.LocalDateTime;
import java.util.LinkedList;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import static us.ihmc.behaviors.BehaviorModule.API.BehaviorTreeStatus;
import static us.ihmc.behaviors.BehaviorModule.API.StatusLog;

public class GDXBehaviorsPanel
{
   private final String windowName = ImGuiTools.uniqueLabel(getClass(), "Behaviors");
   private final ImString behaviorModuleHost = new ImString("localhost", 100);
   private final AtomicReference<BehaviorTreeControlFlowNode> behaviorTreeStatus;
   private volatile boolean messagerConnecting = false;
   private String messagerConnectedHost = "";
   private final MessagerHelper messagerHelper;
   private volatile boolean yoClientDisconnecting = false;
   private volatile boolean yoClientConnecting = false;
   private final YoVariableClientHelper yoVariableClientHelper;
   private final GDXBehaviorUIInterface highestLevelUI;
   private final BehaviorHelper behaviorHelper;
   private final LinkedList<Pair<Integer, String>> logArray = new LinkedList<>();
   private final ImGuiBehaviorTreePanel behaviorTreePanel = new ImGuiBehaviorTreePanel("Behavior Tree");

   public GDXBehaviorsPanel(ROS2Node ros2Node,
                            Supplier<? extends DRCRobotModel> robotModelSupplier,
                            GDXBehaviorUIRegistry behaviorRegistry)
   {

      behaviorHelper = new BehaviorHelper("Behaviors panel", robotModelSupplier.get(), ros2Node);
      messagerHelper = behaviorHelper.getMessagerHelper();
      yoVariableClientHelper = behaviorHelper.getYoVariableClientHelper();
      highestLevelUI = behaviorRegistry.getHighestLevelNode().getBehaviorUISupplier().create(behaviorHelper);

      logArray.addLast(Pair.of(Level.INFO.intLevel(), "Log started at " + LocalDateTime.now()));
      behaviorHelper.subscribeViaCallback(StatusLog, logEntry ->
      {
         synchronized (logArray)
         {
            logArray.addLast(logEntry);
         }
      });
      behaviorHelper.subscribeViaCallback(ROS2Tools.TEXT_STATUS, textStatus ->
      {
         synchronized (logArray)
         {
            logArray.addLast(Pair.of(Level.INFO.intLevel(), textStatus.getTextToSpeakAsString()));
         }
      });
      behaviorTreeStatus = behaviorHelper.subscribeViaReference(BehaviorTreeStatus, new FallbackNode());
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      highestLevelUI.create(baseUI);
      baseUI.getSceneManager().addRenderableProvider(highestLevelUI, GDXSceneLevel.VIRTUAL);
   }

   public void handleVREvents(GDXVRManager vrManager)
   {
      highestLevelUI.handleVREvents(vrManager);
   }

   public void render()
   {
      ImGui.begin(windowName);
      if (messagerConnecting)
      {
         ImGui.text("Messager connecting...");
         if (messagerHelper.isConnected())
         {
            messagerConnecting = false;
         }
      }
      else if (messagerHelper.isDisconnecting())
      {
         ImGui.text("Messager disconnecting...");
      }
      else if (!messagerHelper.isConnected())
      {
         int flags = ImGuiInputTextFlags.None;
         flags += ImGuiInputTextFlags.CallbackResize;
         ImGui.inputText(ImGuiTools.uniqueIDOnly(getClass(), "messagerHost"), behaviorModuleHost, flags);
         ImGui.sameLine();
         if (ImGui.button("Connect messager")) // TODO: One button should connect both
         {
            connectViaKryo(behaviorModuleHost.get());
         }

         SharedMemoryMessager potentialSharedMemoryMessager = BehaviorModule.getSharedMemoryMessager();
         if (potentialSharedMemoryMessager != null && potentialSharedMemoryMessager.isMessagerOpen())
         {
            if (ImGui.button("Use shared memory messager"))
            {
               messagerHelper.connectViaSharedMemory(potentialSharedMemoryMessager);
            }
         }
      }
      else
      {
         if (messagerHelper.isUsingSharedMemory())
         {
            ImGui.text("Using shared memory messager.");
         }
         else
         {
            ImGui.text("Messager connected to " + messagerConnectedHost + ".");
         }

         if (ImGui.button(ImGuiTools.uniqueLabel(this, "Disconnect messager")))
         {
            disconnectMessager();
         }
      }
      if (yoClientConnecting)
      {
         ImGui.text("YoVariable client connecting...");
         if (yoVariableClientHelper.isConnected())
         {
            yoClientConnecting = false;
         }
      }
      else if (yoClientDisconnecting)
      {
         ImGui.text("YoVariable client disconnecting...");
      }
      else if (!yoVariableClientHelper.isConnected())
      {
         if (ImGui.button("Connect YoVariable client"))
         {
            connectYoVariableClient();
         }
      }
      else
      {
         ImGui.text("YoVariable client connected to: " + yoVariableClientHelper.getServerName() + ".");

         if (ImGui.button("Disconnect YoVariable client"))
         {
            disconnectYoVariableClient();
         }
      }

      if (messagerHelper.isConnected())
      {
//         highestLevelUI.render();
      }

      synchronized (logArray)
      {
         ImGui.text("Behavior status log:");
         ImGui.pushItemWidth(ImGui.getWindowWidth() - 10);
         int numLogEntriesToShow = 20;
         while (logArray.size() > numLogEntriesToShow)
            logArray.removeFirst();
         for (Pair<Integer, String> logEntry : logArray)
         {
            Color color = Color.WHITE;
            float[] hsv = new float[3];
            switch (logEntry.getLeft())
            {
               case 100:
               case 200:
                  color = Color.RED;
                  break;
               case 300:
                  Color.YELLOW.toHsv(hsv);
                  color = color.fromHsv(hsv[0], hsv[1], 0.7f);
                  break;
               case 400:
                  color = Color.BLACK;
                  break;
               case 500:
                  color = Color.CYAN;
                  break;
               case 600:
                  color = Color.GREEN;
                  break;
            }
            ImGui.textColored(color.r, color.g, color.b, 1.0f, logEntry.getRight());
         }
      }
      ImGui.popItemWidth();

      ImGui.end();

      behaviorTreePanel.renderAsWindow(behaviorTreeStatus.get(), highestLevelUI::renderNode);
   }

   public void connectViaKryo(String hostname)
   {
      behaviorModuleHost.set(hostname);
      messagerHelper.connectViaKryo(behaviorModuleHost.get(), NetworkPorts.BEHAVIOR_MODULE_MESSAGER_PORT.getPort());
      messagerConnectedHost = behaviorModuleHost.get();
      messagerConnecting = true;
   }

   public void connectYoVariableClient()
   {
      yoVariableClientHelper.start(behaviorModuleHost.get(), NetworkPorts.BEHAVIOR_MODULE_YOVARIABLESERVER_PORT.getPort());
      yoClientConnecting = true;
   }

   public void disconnectMessager()
   {
      messagerHelper.disconnect();
   }

   public void disconnectYoVariableClient()
   {
      yoVariableClientHelper.disconnect();
   }

   public void destroy()
   {
      behaviorHelper.destroy();
      highestLevelUI.destroy();
   }

   public String getWindowName()
   {
      return windowName;
   }
}
