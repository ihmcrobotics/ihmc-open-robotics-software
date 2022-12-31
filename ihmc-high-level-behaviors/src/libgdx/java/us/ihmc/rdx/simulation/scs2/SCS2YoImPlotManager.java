package us.ihmc.rdx.simulation.scs2;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.flag.ImGuiInputTextFlags;
import imgui.internal.ImGui;
import imgui.type.ImString;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXImGuiPerspectiveManager;
import us.ihmc.rdx.ui.ImGuiConfigurationLocation;
import us.ihmc.rdx.ui.yo.*;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.HybridDirectory;
import us.ihmc.tools.io.HybridFile;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.InputStream;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;

public class SCS2YoImPlotManager
{
   private RDXImGuiPerspectiveManager perspectiveManager;
   private final ArrayList<ImPlotModifiableYoPlotPanel> plotPanels = new ArrayList<>();
   private RDXYoManager yoManager;
   private ImGuiYoVariableSearchPanel yoVariableSearchPanel;
   private ImGuiPanel parentPanel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString panelToCreateName = new ImString("", 100);
   private HybridFile configurationFile;
   private boolean perspectiveReloadQueued = false;
   private int delayedPerspectiveReloadCounter = 0;

   public void create(RDXImGuiPerspectiveManager perspectiveManager, ImGuiPanel parentPanel)
   {
      this.perspectiveManager = perspectiveManager;
      this.parentPanel = parentPanel;

      updateConfigurationFile(perspectiveManager.getPerspectiveDirectory());
      perspectiveManager.getPerspectiveDirectoryUpdatedListeners().add(this::updateConfigurationFile);
      perspectiveManager.getLoadListeners().add(this::loadConfiguration);
      perspectiveManager.getSaveListeners().add(this::saveConfiguration);
   }

   public void update()
   {
      // This is because the panel changes get queued, so we need to wait a couple frames
      // to make sure we are ready to reload.
      if (perspectiveReloadQueued)
      {
         ++delayedPerspectiveReloadCounter;
         if (delayedPerspectiveReloadCounter == 2)
         {
            delayedPerspectiveReloadCounter = 0;
            perspectiveReloadQueued = false;
            perspectiveManager.reloadPerspective();
         }
      }
   }

   public void setupForSession(RDXYoManager yoManager)
   {
      this.yoManager = yoManager;

      if (yoVariableSearchPanel == null)
      {
         // We are using getRootRegistry which is the session's working copy; i.e. not linked
         yoVariableSearchPanel = new ImGuiYoVariableSearchPanel(yoManager.getRootRegistry());
         parentPanel.addChild(yoVariableSearchPanel.getPanel());
      }
      else
      {
         removeAllPlotPanels();
         yoVariableSearchPanel.changeYoRegistry(yoManager.getRootRegistry());

         perspectiveReloadQueued = true;
         delayedPerspectiveReloadCounter = 0;
      }

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
         LogTools.info("Loading {}", configurationFile.getLocationOfResourceForReading());
         plotPanels.clear();
         JSONFileTools.load(inputStream, node ->
         {
            JSONTools.forEachArrayElement(node, "panels", panelNode ->
            {
               String panelName = panelNode.get("name").asText();
               ImPlotModifiableYoPlotPanel plotPanel = addPlotPanel(panelName);
               JSONTools.forEachArrayElement(panelNode, "plots", plotNode ->
               {
                  ImPlotModifiableYoPlot imPlotModifiableYoPlot = plotPanel.addPlot();
                  JSONTools.forEachArrayElement(plotNode, "variables", variableNode ->
                  {
                     // We are using getRootRegistry which is the session's working copy; i.e. not linked
                     imPlotModifiableYoPlot.addVariable(yoManager.getRootRegistry().findVariable(variableNode.get("variableName").asText()), false);
                  });
               });
            });
         });
      }
      else
      {
         LogTools.error("Failed to load {}", configurationFile.getLocationOfResourceForReading());
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
      parentPanel.queueAddChild(plotPanel);
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
      removeAllPlotPanels();
      parentPanel.queueRemoveChild(yoVariableSearchPanel.getPanel());
   }

   private void removeAllPlotPanels()
   {
      ImPlotModifiableYoPlotPanel[] plotPanelsArray = plotPanels.toArray(new ImPlotModifiableYoPlotPanel[0]);
      for (ImPlotModifiableYoPlotPanel plotPanel : plotPanelsArray)
      {
         removePlotPanel(plotPanel);
      }
   }
}
