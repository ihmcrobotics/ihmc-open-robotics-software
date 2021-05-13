package us.ihmc.gdx.ui.yo;

import imgui.internal.ImGui;
import imgui.type.ImInt;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataVisualizer.BasicYoVariablesUpdatedListener;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;

public class ImGuiGDXYoGraphPanel
{
   private final String title;
   private final String controllerHost = NetworkParameters.getHost(NetworkParameterKeys.robotController);
   private final int bufferSize;
   private final ArrayList<Runnable> graphs = new ArrayList<>();
   private volatile boolean handshakeComplete = false;
   private volatile boolean disconnecting = false;
   private volatile boolean connecting = false;
   private YoVariableClient yoVariableClient;

   private final ImInt graphGroupSelectedIndex = new ImInt(0);
   private String[] graphGroupNames = new String[0];
   private HashMap<String, GDXYoGraphGroup> graphGroups = new HashMap<>();

   public ImGuiGDXYoGraphPanel(String title, int bufferSize)
   {
      this.title = title;
      this.bufferSize = bufferSize;
   }

   public void startYoVariableClient()
   {
//      Set<String> graphGroupNameKeySet = graphGroups.keySet();
//      String robotControllerHost = NetworkParameters.getHost(NetworkParameterKeys.robotController);
//      String graphGroupName;
//      if (robotControllerHost.trim().toLowerCase().contains("cpu4") && graphGroupNameKeySet.contains("Real robot"))
//      {
//         Arrays.asList(graphGroupNames).indexOf("Real robot");
//         graphGroupSelectedIndex.set(graphGroupNames.);
//      }
   }

   public void startYoVariableClient(String graphGroupName)
   {
      connecting = true;
      handshakeComplete = false;
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      BasicYoVariablesUpdatedListener clientUpdateListener = new BasicYoVariablesUpdatedListener(registry);
      yoVariableClient = new YoVariableClient(clientUpdateListener);
      MissingThreadTools.startAsDaemon("YoVariableClient", DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, () ->
      {
         LogTools.info("Connecting to {}:{}", controllerHost, DataServerSettings.DEFAULT_PORT);
         boolean connectionRefused = true;
         while (connectionRefused)
         {
            try
            {
               yoVariableClient.start(controllerHost, DataServerSettings.DEFAULT_PORT);
               connectionRefused = false;
            }
            catch (RuntimeException e)
            {
               if (e.getMessage().contains("Connection refused"))
               {
                  ThreadTools.sleep(100);
               }
               else
               {
                  throw e;
               }
            }
         }
         if (!clientUpdateListener.isHandshakeComplete())
            LogTools.info("Waiting for handshake...");
         while (!clientUpdateListener.isHandshakeComplete())
         {
            ThreadTools.sleep(100);
         }
         LogTools.info("Handshake complete.");
         handshakeComplete = true;

//         listVariables(registry, 4);
      });

      GDXYoGraphGroup graphGroup = graphGroups.get(graphGroupName);
      for (String variableName : graphGroup.getVariableNames())
      {
         MissingThreadTools.startAsDaemon("AddGraph", DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, () ->
         {
            while (!handshakeComplete)
            {
               ThreadTools.sleep(150);
            }
            final YoVariable variable = registry.findVariable(registry.getName() + "." + variableName);
            if (variable != null)
            {
               LogTools.info("Setting up graph for variable: {}", variable.getFullName());
               float[] values = new float[bufferSize];
               AtomicInteger currentIndex = new AtomicInteger(0);
               synchronized (graphs)
               {
                  graphs.add(() ->
                  {
                     int currentValueIndex = currentIndex.getAndIncrement();
                     values[currentValueIndex] = (float) variable.getValueAsDouble();
                     int graphWidth = 230;
                     int graphHeight = 50;
                     ImGui.plotLines(variable.getName(),
                                     values,
                                     bufferSize,
                                     0,
                                     "" + values[currentValueIndex],
                                     Float.MAX_VALUE,
                                     Float.MAX_VALUE,
                                     graphWidth,
                                     graphHeight);
                     if (ImGui.beginPopup(variable.getName() + " hover popup"))
                     {
                        ImGui.text(variable.getFullNameString());
                        ImGui.endPopup();
                     }

                     if (currentValueIndex >= bufferSize - 1)
                     {
                        currentIndex.set(0);
                     }
                  });
               }
            }
            else
            {
               LogTools.error("Variable not found: {}", variableName);
            }
            connecting = false;
         });
      }
   }

   private void listVariables(YoRegistry registry, int depth)
   {
      for (YoVariable variable : registry.getVariables())
      {
         System.out.println(variable.getFullNameString());
      }

      if (depth > 0)
      {
         for (YoRegistry child : registry.getChildren())
         {
            listVariables(child, depth - 1);
         }
      }
   }

   public void render()
   {
      ImGui.begin(title);

      ImGui.text("Controller host: " + controllerHost);

      ImGui.combo(ImGuiTools.uniqueIDOnly(this, "Profile"), graphGroupSelectedIndex, graphGroupNames, graphGroupNames.length);
      ImGui.sameLine();
      if (connecting)
      {
         ImGui.text("Connecting...");
      }
      else if (disconnecting)
      {
         ImGui.text("Disconnecting...");
      }
      else if (yoVariableClient == null)
      {
         if (ImGui.button(ImGuiTools.uniqueLabel(this, "Connect")))
         {
            startYoVariableClient(graphGroupNames[graphGroupSelectedIndex.get()]);
         }
      }
      else
      {
         if (ImGui.button(ImGuiTools.uniqueLabel(this, "Disconnect")))
         {
            destroy();
         }
      }

      synchronized (graphs)
      {
         for (Runnable graph : graphs)
         {
            graph.run();
         }
      }

      ImGui.end();
   }

   public void graphVariable(String groupName, String yoVariableName)
   {
      GDXYoGraphGroup graphGroup = graphGroups.get(groupName);
      if (graphGroup == null)
      {
         graphGroup = new GDXYoGraphGroup();
         graphGroups.put(groupName, graphGroup);
      }

      graphGroup.addVariable(yoVariableName);
      graphGroupNames = graphGroups.keySet().toArray(new String[0]);
   }

   public void destroy()
   {
      disconnecting = true;
      graphs.clear();
      yoVariableClient.stop();
      yoVariableClient = null;
      handshakeComplete = false;
      disconnecting = false;
   }

   public String getWindowName()
   {
      return title;
   }
}
