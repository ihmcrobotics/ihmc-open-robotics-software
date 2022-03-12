package us.ihmc.gdx.simulation.scs2;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.flag.ImGuiInputTextFlags;
import imgui.internal.ImGui;
import imgui.type.ImString;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.GDXImGuiPerspectiveManager;
import us.ihmc.gdx.ui.ImGuiConfigurationLocation;
import us.ihmc.gdx.ui.yo.ImGuiYoVariableSearchPanel;
import us.ihmc.gdx.ui.yo.ImPlotYoPlotPanel;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.HybridDirectory;
import us.ihmc.tools.io.HybridFile;
import us.ihmc.tools.io.JSONFileTools;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;

public class SCS2YoImPlotManager
{
   private final ImGuiPanel panel = new ImGuiPanel("SCS 2 Plot Manager", this::renderImGuiWidgets);
   private final ArrayList<ImPlotYoPlotPanel> plotPanels = new ArrayList<>();
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
         for (Iterator<JsonNode> it = node.withArray("panels").elements(); it.hasNext(); )
         {
            JsonNode objectNode = it.next();
            String panelName = objectNode.get("name").asText();
            addPlotPanel(panelName);
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
            ArrayNode arrayNode = rootNode.putArray("panels");
            for (ImPlotYoPlotPanel plotPanel : plotPanels)
            {
               ObjectNode node = arrayNode.addObject();
               node.put("name", plotPanel.getPanelName());
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
   }

   private void addPlotPanel(String name)
   {
      ImPlotYoPlotPanel plotPanel = new ImPlotYoPlotPanel(name);
      plotPanel.getIsShowing().set(true);
      panel.addChild(plotPanel);
      plotPanels.add(plotPanel);
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}
