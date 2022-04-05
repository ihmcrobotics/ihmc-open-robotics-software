package us.ihmc.gdx.simulation.scs2;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.flag.ImGuiInputTextFlags;
import imgui.internal.ImGui;
import imgui.type.ImString;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.GDXImGuiPerspectiveManager;
import us.ihmc.gdx.ui.ImGuiConfigurationLocation;
import us.ihmc.gdx.ui.yo.*;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.HybridDirectory;
import us.ihmc.tools.io.HybridFile;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.InputStream;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;

public class SCS2YoImPlotManager
{
   private final ArrayList<ImPlotModifiableYoPlotPanel> plotPanels = new ArrayList<>();
   private GDXYoManager yoManager;
   private ImGuiYoVariableSearchPanel yoVariableSearchPanel;
   private ImGuiPanel parentPanel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString panelToCreateName = new ImString("", 100);
   private HybridFile configurationFile;

   public void create(GDXImGuiPerspectiveManager perspectiveManager, GDXYoManager yoManager, ImGuiPanel parentPanel)
   {
      this.yoManager = yoManager;
      this.parentPanel = parentPanel;

      yoVariableSearchPanel = new ImGuiYoVariableSearchPanel(yoManager.getRootRegistry());
      parentPanel.addChild(yoVariableSearchPanel.getPanel());

      perspectiveManager.getPerspectiveDirectoryUpdatedListeners().add(this::updateConfigurationFile);
      perspectiveManager.getLoadListeners().add(this::loadConfiguration);
      perspectiveManager.getSaveListeners().add(this::saveConfiguration);
      updateConfigurationFile(perspectiveManager.getPerspectiveDirectory());
      loadConfiguration(perspectiveManager.getCurrentConfigurationLocation());
   }

   private void updateConfigurationFile(HybridDirectory perspectiveDirectory)
   {
      configurationFile = new HybridFile(perspectiveDirectory, getClass().getSimpleName() + ".json");
   }

   private void loadConfiguration(ImGuiConfigurationLocation configurationLocation)
   {
      configurationFile.setMode(configurationLocation.toHybridResourceMode());
      InputStream inputStream = configurationFile.getInputStream();
      if (inputStream != null)
      {
         plotPanels.clear();
         JSONFileTools.load(inputStream, node ->
         {
            for (Iterator<JsonNode> panelNodeIterator = node.withArray("panels").elements(); panelNodeIterator.hasNext(); )
            {
               JsonNode panelNode = panelNodeIterator.next();
               String panelName = panelNode.get("name").asText();
               ImPlotModifiableYoPlotPanel plotPanel = addPlotPanel(panelName);
               for (Iterator<JsonNode> plotsNodeInterator = panelNode.withArray("plots").elements(); plotsNodeInterator.hasNext(); )
               {
                  JsonNode plotNode = plotsNodeInterator.next();
                  ImPlotModifiableYoPlot imPlotModifiableYoPlot = plotPanel.addPlot();
                  for (Iterator<JsonNode> variablesNodeInterator = plotNode.withArray("variables").elements(); variablesNodeInterator.hasNext(); )
                  {
                     JsonNode variableNode = variablesNodeInterator.next();
                     imPlotModifiableYoPlot.addVariable(yoManager.getRootRegistry().findVariable(variableNode.get("variableName").asText()), false);
                  }
               }
            }
         });
      }
   }

   private void saveConfiguration(ImGuiConfigurationLocation configurationLocation)
   {
      configurationFile.setMode(configurationLocation.toHybridResourceMode());
      if (configurationFile.isWorkspaceFileAccessAvailable())
      {
         Path fileForWriting = configurationFile.getFileForWriting();
         LogTools.info("Saving plot panels to {}", fileForWriting.toAbsolutePath().normalize().toString());
         JSONFileTools.save(fileForWriting, rootNode ->
         {
            ArrayNode panelArrayNode = rootNode.putArray("panels");
            for (ImPlotModifiableYoPlotPanel plotPanel : plotPanels)
            {
               ObjectNode panelNode = panelArrayNode.addObject();
               panelNode.put("name", plotPanel.getPanelName());
               ArrayNode plotArrayNode = panelNode.putArray("plots");
               for (ImPlotModifiableYoPlot yoPlot : plotPanel.getYoPlots())
               {
                  ObjectNode plotNode = plotArrayNode.addObject();
                  ArrayNode variableArrayNode = plotNode.putArray("variables");
                  for (Pair<YoVariable, ImPlotYoBufferPlotLineBasics> yoVariableImPlotPlotLinePair : yoPlot.getVariablePlotLinePairList())
                  {
                     ObjectNode variableNode = variableArrayNode.addObject();
                     variableNode.put("variableName", yoVariableImPlotPlotLinePair.getLeft().getFullNameString());
                  }
               }
            }
         });
      }
   }

   public void initializeLinkedVariables()
   {
      yoVariableSearchPanel.initializeYoVariablesAfterSessionStart();
      for (ImPlotModifiableYoPlotPanel plotPanel : plotPanels)
      {
         for (ImPlotModifiableYoPlot yoPlot : plotPanel.getYoPlots())
         {
            for (Pair<YoVariable, ImPlotYoBufferPlotLineBasics> yoVariableImPlotPlotLinePair : yoPlot.getVariablePlotLinePairList())
            {
               yoVariableImPlotPlotLinePair.getRight().setupLinkedVariable(yoManager);
            }
         }
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.pushItemWidth(150);
      int flags = ImGuiInputTextFlags.None;
      flags += ImGuiInputTextFlags.CallbackResize;
      ImGui.inputText(labels.get("Panel name"), panelToCreateName, flags);
      ImGui.popItemWidth();
      ImGui.sameLine();
      if (ImGui.button("Add Plot Panel") && !panelToCreateName.get().isEmpty())
      {
         addPlotPanel(panelToCreateName.get());
      }
   }

   private ImPlotModifiableYoPlotPanel addPlotPanel(String name)
   {
      ImPlotModifiableYoPlotPanel plotPanel = new ImPlotModifiableYoPlotPanel(name, yoVariableSearchPanel, yoManager, this::removePlotPanel);
      plotPanel.getIsShowing().set(true);
      parentPanel.addChild(plotPanel);
      plotPanels.add(plotPanel);
      return plotPanel;
   }

   private void removePlotPanel(ImPlotModifiableYoPlotPanel plotPanel)
   {
      plotPanels.remove(plotPanel);
      parentPanel.queueRemoveChild(plotPanel);
   }

   public void destroy()
   {
      ImPlotModifiableYoPlotPanel[] plotPanelsArray = plotPanels.toArray(new ImPlotModifiableYoPlotPanel[0]);
      for (ImPlotModifiableYoPlotPanel plotPanel : plotPanelsArray)
      {
         removePlotPanel(plotPanel);
      }
      parentPanel.queueRemoveChild(yoVariableSearchPanel.getPanel());
   }
}
