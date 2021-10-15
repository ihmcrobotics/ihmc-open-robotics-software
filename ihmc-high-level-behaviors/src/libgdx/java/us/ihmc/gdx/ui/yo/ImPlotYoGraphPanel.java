package us.ihmc.gdx.ui.yo;

import imgui.ImVec2;
import imgui.extension.implot.ImPlot;
import imgui.extension.implot.ImPlotContext;
import imgui.extension.implot.ImPlotStyle;
import imgui.flag.ImGuiInputTextFlags;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import imgui.type.ImString;
import us.ihmc.behaviors.tools.yo.YoDoubleClientHelper;
import us.ihmc.behaviors.tools.yo.YoVariableClientHelper;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.tools.ImPlotTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.*;

public class ImPlotYoGraphPanel
{
   private final String title;
   private final String controllerHost = NetworkParameters.getHost(NetworkParameterKeys.robotController);
   private final int bufferSize;
   private final HashMap<String, ArrayList<ImPlotYoGraph>> graphs = new HashMap<>();
   private final YoVariableClientHelper yoClientHelper = new YoVariableClientHelper(getClass().getSimpleName());
   private final ImInt serverSelectedIndex = new ImInt(0);
   private String[] serverGraphGroupNames = new String[0];
   private final HashMap<String, TreeSet<String>> serverGraphGroups = new HashMap<>();
   private final HashMap<String, ArrayList<ImGuiModifiableYoDouble>> modifiableVariables = new HashMap<>();
   private ImPlotContext context = null;
   private final ImBoolean showAllVariables = new ImBoolean(false);
   private final ImString searchBar = new ImString();
   private ImPlotYoGraph graphRequesting = null;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public ImPlotYoGraphPanel(String title, int bufferSize)
   {
      this.title = title;
      this.bufferSize = bufferSize;
   }

   public void create()
   {
      context = ImPlot.createContext();
      ImPlotStyle style = ImPlot.getStyle();
      style.setPlotPadding(new ImVec2(0, 0));
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
      for (int i = 0; i < serverGraphGroupNames.length; i++)
      {
         if (serverGraphGroupNames[i].equals(graphGroupName))
         {
            serverSelectedIndex.set(i);
         }
      }

      yoClientHelper.start(controllerHost, DataServerSettings.DEFAULT_PORT);
   }

   private void getAllVariablesHelper(YoRegistry registry, ArrayList<YoVariable> output)
   {
      output.addAll(registry.getVariables());

      for (YoRegistry child : registry.getChildren())
      {
         getAllVariablesHelper(child, output);
      }
   }

   private List<YoVariable> getAllVariables(YoRegistry registry)
   {
      ArrayList<YoVariable> output = new ArrayList<>();
      getAllVariablesHelper(registry, output);
      return output;
   }

   public void renderImGuiWidgetsVariablePanel()
   {
      if (graphRequesting == null)
      {
         ImGui.text("Select a graph by right clicking it to add a variable.");
         return;
      }

      ArrayList<String> variableNames = yoClientHelper.getVariableNames();
      variableNames.sort(Comparator.comparing(String::toString));

      ImGui.inputText("Variable Search", searchBar);

      ImGui.sameLine();
      if (ImGui.button("Cancel"))
      {
         showAllVariables.set(false);
         graphRequesting.cancelWantVariable();
         graphRequesting = null;
      }

      if (ImGui.beginListBox("##YoVariables", ImGui.getColumnWidth(), ImGui.getWindowSizeY() - 100))
      {
         for (String variableName : variableNames)
         {
            if (!variableName.toLowerCase().contains(searchBar.get().toLowerCase()))
               continue;

            ImGui.selectable(variableName);
            if (ImGui.isItemClicked())
            {
               graphRequesting.addVariable(variableName);
               showAllVariables.set(false);
               graphRequesting = null;
            }
         }
         ImGui.endListBox();
      }
   }

   public void renderImGuiWidgetsGraphPanel()
   {
      ImGui.text("Controller host: " + controllerHost);

      ImGui.combo(ImGuiTools.uniqueIDOnly(this, "Profile"), serverSelectedIndex, serverGraphGroupNames, serverGraphGroupNames.length);
      ImGui.sameLine();
      if (yoClientHelper.isConnecting())
      {
         ImGui.text("Connecting...");
      }
      else if (!yoClientHelper.isConnected())
      {
         if (ImGui.button(ImGuiTools.uniqueLabel(this, "Connect")))
         {
            startYoVariableClient(serverGraphGroupNames[serverSelectedIndex.get()]);
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
         Iterator<ImPlotYoGraph> graphsIterator = graphs.get(serverGraphGroupNames[serverSelectedIndex.get()]).iterator();
         while (graphsIterator.hasNext())
         {
            ImPlotYoGraph graph = graphsIterator.next();
            graph.render(context);
            if (!graph.shouldGraphExist())
               graphsIterator.remove();
            else if (graph.graphWantsVariable())
            {
               showAllVariables.set(true);
               graphRequesting = graph;
            }
         }
      }

      if (ImGui.button("Add new graph"))
      {
         graphs.get(serverGraphGroupNames[serverSelectedIndex.get()]).add(new ImPlotYoGraph(yoClientHelper, bufferSize));
      }

      if (modifiableVariables.containsKey(serverGraphGroupNames[serverSelectedIndex.get()]))
      {
         for (ImGuiModifiableYoDouble modifiableYoDouble : modifiableVariables.get(serverGraphGroupNames[serverSelectedIndex.get()]))
         {
            modifiableYoDouble.update();
            ImGui.pushItemWidth(110.0f);
            double step = 0.1;
            double stepFast = 0.0;
            String format = "%.4f";
            if (ImGui.inputDouble(labels.get(modifiableYoDouble.getYoDoubleHelper().getName()),
                                  modifiableYoDouble.getImDouble(),
                                  step,
                                  stepFast,
                                  format,
                                  ImGuiInputTextFlags.EnterReturnsTrue))
            {
               modifiableYoDouble.set();
            }
            ImGui.popItemWidth();
            ImGui.sameLine();
            ImGui.text("Server: " + modifiableYoDouble.getYoDoubleHelper().get());
         }
      }
   }

   public void graphVariable(String serverName, String yoVariableName)
   {
      TreeSet<String> graphGroup = serverGraphGroups.computeIfAbsent(serverName, k -> new TreeSet<>());

      graphGroup.add(yoVariableName);
      serverGraphGroupNames = serverGraphGroups.keySet().toArray(new String[0]);

      YoDoubleClientHelper yoDoubleHelper = yoClientHelper.subscribeToYoDouble(yoVariableName);
      LogTools.info("Setting up graph for variable: {}", yoDoubleHelper.getFullName());
      Double[] values = ImPlotTools.newNaNFilledBuffer(bufferSize);
      synchronized (graphs)
      {
         ArrayList<ImPlotYoGraph> graphsForServer = graphs.computeIfAbsent(serverName, key -> new ArrayList<>());
         graphsForServer.add(new ImPlotYoGraph(yoDoubleHelper, yoClientHelper, values, bufferSize));
      }
   }

   public void modifyVariable(String serverName, String yoVariableName)
   {
      ArrayList<ImGuiModifiableYoDouble> modifiableVariablesForServer = modifiableVariables.computeIfAbsent(serverName, key -> new ArrayList<>());
      YoDoubleClientHelper yoDoubleHelper = yoClientHelper.subscribeToYoDouble(yoVariableName);
      modifiableVariablesForServer.add(new ImGuiModifiableYoDouble(yoDoubleHelper));
   }

   public void destroy()
   {
      yoClientHelper.disconnect();
      graphs.clear();
      ImPlot.destroyContext(context);
   }

   public String getWindowName()
   {
      return title;
   }

   public String getVarWindowName()
   {
      return title + " List";
   }
}
