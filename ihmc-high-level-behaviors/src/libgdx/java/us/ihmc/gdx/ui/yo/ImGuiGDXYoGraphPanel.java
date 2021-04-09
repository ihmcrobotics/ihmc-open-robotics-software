package us.ihmc.gdx.ui.yo;

import imgui.internal.ImGui;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataVisualizer.BasicYoVariablesUpdatedListener;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicInteger;

public class ImGuiGDXYoGraphPanel
{
   private final String title;
   private final int bufferSize;
   private final ArrayList<Runnable> graphs = new ArrayList<>();
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private volatile boolean handshakeComplete = false;
   private YoVariableClient yoVariableClient;

   private ArrayList<GDXYoGraphGroup> graphGroups = new ArrayList<>();

   public ImGuiGDXYoGraphPanel(String title, int bufferSize)
   {
      this.title = title;
      this.bufferSize = bufferSize;
   }

   public void startYoVariableClient(String hostname)
   {
      BasicYoVariablesUpdatedListener clientUpdateListener = new BasicYoVariablesUpdatedListener(registry);
      yoVariableClient = new YoVariableClient(clientUpdateListener);
      MissingThreadTools.startAsDaemon("YoVariableClient", DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, () ->
      {
         LogTools.info("Connecting to {}:{}", hostname, DataServerSettings.DEFAULT_PORT);
         boolean connectionRefused = true;
         while (connectionRefused)
         {
            try
            {
               yoVariableClient.start(hostname, DataServerSettings.DEFAULT_PORT);
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

      synchronized (graphs)
      {
         for (Runnable graph : graphs)
         {
            graph.run();
         }
      }

      ImGui.end();
   }

   public void graphVariable(String yoVariableName)
   {
      MissingThreadTools.startAsDaemon("AddGraph", DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, () ->
      {
         while (!handshakeComplete)
         {
            ThreadTools.sleep(150);
         }
         final YoVariable variable = registry.findVariable(registry.getName() + "." + yoVariableName);
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
            LogTools.error("Variable not found: {}", yoVariableName);
         }
      });
   }

   public void destroy()
   {
      yoVariableClient.stop();
   }

   public String getWindowName()
   {
      return title;
   }
}
