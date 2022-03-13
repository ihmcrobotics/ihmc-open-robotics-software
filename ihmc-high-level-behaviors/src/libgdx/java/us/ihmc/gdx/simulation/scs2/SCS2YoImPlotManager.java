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
import us.ihmc.gdx.ui.yo.ImGuiYoVariableSearchPanel;
import us.ihmc.gdx.ui.yo.ImPlotModifiableYoPlot;
import us.ihmc.gdx.ui.yo.ImPlotModifiableYoPlotPanel;
import us.ihmc.gdx.ui.yo.ImPlotPlotLine;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.HybridDirectory;
import us.ihmc.tools.io.HybridFile;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.yoVariables.variable.YoVariable;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;

public class SCS2YoImPlotManager
{
   private final ImGuiPanel panel = new ImGuiPanel("SCS 2 Plot Manager", this::renderImGuiWidgets);
   private final ArrayList<ImPlotModifiableYoPlotPanel> plotPanels = new ArrayList<>();
   private GDXYoManager yoManager;
   private ImGuiYoVariableSearchPanel yoVariableSearchPanel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString panelToCreateName = new ImString("", 100);
   private HybridFile configurationFile;

   public void create(GDXImGuiPerspectiveManager perspectiveManager, GDXYoManager yoManager)
   {
      this.yoManager = yoManager;

      yoVariableSearchPanel = new ImGuiYoVariableSearchPanel(yoManager.getRootRegistry());
      panel.addChild(yoVariableSearchPanel.getPanel());

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
      JSONFileTools.load(configurationFile.getInputStream(), node ->
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
                  imPlotModifiableYoPlot.addVariable(yoManager.getRootRegistry().findVariable(variableNode.get("variableName").asText()));
               }
            }
         }
      });
   }

   private void saveConfiguration(ImGuiConfigurationLocation configurationLocation)
   {
      configurationFile.setMode(configurationLocation.toHybridResourceMode());
      if (configurationFile.isWorkspaceWritingAvailable())
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
                  for (Pair<YoVariable, ImPlotPlotLine> yoVariableImPlotPlotLinePair : yoPlot.getVariablePlotLinePairList())
                  {
                     ObjectNode variableNode = variableArrayNode.addObject();
                     variableNode.put("variableName", yoVariableImPlotPlotLinePair.getLeft().getFullNameString());
                  }
               }
            }
         });
      }
   }

   private void renderImGuiWidgets()
   {
      int flags = ImGuiInputTextFlags.None;
      flags += ImGuiInputTextFlags.CallbackResize;
      ImGui.inputText(labels.getHidden("panelToCreateName"), panelToCreateName, flags);
      ImGui.sameLine();
      if (ImGui.button("Add Panel") && !panelToCreateName.get().isEmpty())
      {
         addPlotPanel(panelToCreateName.get());
      }

      for (ImPlotModifiableYoPlotPanel plotPanel : plotPanels)
      {

      }
   }

   private ImPlotModifiableYoPlotPanel addPlotPanel(String name)
   {
      ImPlotModifiableYoPlotPanel plotPanel = new ImPlotModifiableYoPlotPanel(name, yoVariableSearchPanel, yoManager);
      plotPanel.getIsShowing().set(true);
      panel.addChild(plotPanel);
      plotPanels.add(plotPanel);
      return plotPanel;
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}
