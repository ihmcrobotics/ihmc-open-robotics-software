package us.ihmc.rdx.simulation.scs2;

import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiInputTextFlags;
import imgui.flag.ImGuiMouseButton;
import imgui.ImGui;
import imgui.type.ImString;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXImGuiLayoutManager;
import us.ihmc.rdx.ui.ImGuiConfigurationLocation;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.*;
import us.ihmc.yoVariables.variable.YoVariable;

import java.nio.file.Path;
import java.util.ArrayList;

public class RDXSCS2PlottingManager
{
   private RDXImGuiLayoutManager layoutManager;
   private final ArrayList<RDXSCS2PlotPanel> plotPanels = new ArrayList<>();
   private final ArrayList<RDXLinkedYoBooleanWidget> variableWidgets = new ArrayList<>();
   private RDXYoManager yoManager;
   private RDXSCS2YoVariableSearchPanel searchPanel;
   private RDXPanel parentPanel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString panelToCreateName = new ImString("", 100);
   private HybridResourceFile configurationFile;
//   private boolean layoutReloadQueued = false;
//   private int delayedLayoutReloadCounter = 0;

   public void create(RDXImGuiLayoutManager layoutManager, RDXPanel parentPanel)
   {
      this.layoutManager = layoutManager;
      this.parentPanel = parentPanel;

      updateConfigurationFile(layoutManager.getLayoutDirectory());
      layoutManager.getLayoutDirectoryUpdatedListeners().add(this::updateConfigurationFile);
//      layoutManager.getLoadListeners().add(this::loadConfiguration);
//      layoutManager.getSaveListeners().add(this::saveConfiguration);
   }

   public void update()
   {
      // This is because the panel changes get queued, so we need to wait a couple frames
      // to make sure we are ready to reload.
//      if (layoutReloadQueued)
//      {
//         ++delayedLayoutReloadCounter;
//         if (delayedLayoutReloadCounter == 2)
//         {
//            delayedLayoutReloadCounter = 0;
//            layoutReloadQueued = false;
//            layoutManager.reloadLayout();
//         }
//      }
   }

   public void setupForSession(RDXYoManager yoManager)
   {
      this.yoManager = yoManager;

      if (searchPanel == null)
      {
         // We are using getRootRegistry which is the session's working copy; i.e. not linked
         searchPanel = new RDXSCS2YoVariableSearchPanel(yoManager.getRootRegistry());
         parentPanel.addChild(searchPanel.getPanel());
      }
      else
      {
         removeAllPlotPanels();
         searchPanel.changeYoRegistry(yoManager.getRootRegistry());
      }

//      loadConfiguration(layoutManager.getCurrentConfigurationLocation());
   }

   private void updateConfigurationFile(HybridResourceDirectory layoutDirectory)
   {
      configurationFile = new HybridResourceFile(layoutDirectory, getClass().getSimpleName() + ".json");
   }

   private boolean loadConfiguration(ImGuiConfigurationLocation configurationLocation)
   {
      configurationFile.setMode(configurationLocation.toHybridResourceMode());
      LogTools.info("Loading plot panels from {}", configurationFile.getLocationOfResourceForReading());
      return configurationFile.getInputStream(inputStream ->
      {
         plotPanels.clear();
         JSONFileTools.load(inputStream, node ->
         {
            JSONTools.forEachArrayElement(node, "panels", panelNode ->
            {
               String panelName = panelNode.get("name").asText();
               RDXSCS2PlotPanel plotPanel = addPlotPanel(panelName);
               JSONTools.forEachArrayElement(panelNode, "plots", plotNode ->
               {
//                  RDXSCS2Plot imPlotModifiableYoPlot = plotPanel.addPlot();
//                  JSONTools.forEachArrayElement(plotNode, "variables", variableNode ->
//                  {
//                     // We are using getRootRegistry which is the session's working copy; i.e. not linked
//                     imPlotModifiableYoPlot.addVariable(yoManager.getRootRegistry().findVariable(variableNode.get("variableName").asText()), false);
//                  });
               });
            });
         });
      });
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
            for (RDXSCS2PlotPanel plotPanel : plotPanels)
            {
               ObjectNode panelNode = panelArrayNode.addObject();
               panelNode.put("name", plotPanel.getPanelName());
               ArrayNode plotArrayNode = panelNode.putArray("plots");
               for (int column = 0; column < plotPanel.getColumns().get(); column++)
               {
                  for (int row = 0; row < plotPanel.getRows().get(); row++)
                  {
                     RDXSCS2Plot plot = plotPanel.getPlot(column, row);
                     ObjectNode plotNode = plotArrayNode.addObject();
                     ArrayNode variableArrayNode = plotNode.putArray("variables");
                     for (Pair<YoVariable, RDXSCS2PlotLine> yoVariableImPlotPlotLinePair : plot.getVariablePlotLinePairList())
                     {
                        ObjectNode variableNode = variableArrayNode.addObject();
                        variableNode.put("variableName", yoVariableImPlotPlotLinePair.getLeft().getFullNameString());
                     }
                  }
               }
            }
         });
      }
   }

   public void initializeLinkedVariables()
   {
      searchPanel.initializeYoVariablesAfterSessionStart();
      for (RDXSCS2PlotPanel plotPanel : plotPanels)
      {
         for (int column = 0; column < plotPanel.getColumns().get(); column++)
         {
            for (int row = 0; row < plotPanel.getRows().get(); row++)
            {
               RDXSCS2Plot plot = plotPanel.getPlot(column, row);
               for (Pair<YoVariable, RDXSCS2PlotLine> yoVariableImPlotPlotLinePair : plot.getVariablePlotLinePairList())
                  yoVariableImPlotPlotLinePair.getRight().setupLinkedVariable(yoManager);
            }
         }
      }
   }

   public void renderImGuiWidgets()
   {
      for (RDXLinkedYoBooleanWidget variableWidget : variableWidgets)
      {
         variableWidget.renderImGuiWidgets();
      }

      ImGui.pushItemWidth(150);
      int flags = ImGuiInputTextFlags.None;
      flags += ImGuiInputTextFlags.CallbackResize;
      ImGui.inputTextWithHint(labels.get("Panel name"), "Plot Panel " + plotPanels.size(), panelToCreateName, flags);
      ImGui.popItemWidth();
      ImGui.sameLine();
      if (ImGui.button("Add Plot Panel"))
         addPlotPanel(panelToCreateName.get().isEmpty() ? "Plot Panel " + plotPanels.size() : panelToCreateName.get());

      RDXSCS2PlotPanel remove = null;
      for (RDXSCS2PlotPanel plotPanel : plotPanels)
      {
         ImGui.checkbox(labels.get(plotPanel.getPanelName()), plotPanel.getIsShowing());
         ImGui.sameLine();
         ImGui.pushStyleColor(ImGuiCol.Text, ImGuiTools.DARK_RED);
         if (ImGuiTools.textWithUnderlineOnHover("X") && imgui.ImGui.isMouseClicked(ImGuiMouseButton.Left))
            remove = plotPanel;
         ImGui.popStyleColor();
      }
      if (remove != null)
         removePlotPanel(remove);
   }

   public RDXSCS2PlotPanel addPlotPanel(String name)
   {
      RDXSCS2PlotPanel plotPanel = new RDXSCS2PlotPanel(name, searchPanel, yoManager, this::removePlotPanel);
      plotPanel.getIsShowing().set(true);
      parentPanel.queueAddChild(plotPanel);
      plotPanels.add(plotPanel);
      return plotPanel;
   }

   private void removePlotPanel(RDXSCS2PlotPanel plotPanel)
   {
      plotPanels.remove(plotPanel);
      parentPanel.queueRemoveChild(plotPanel);
   }

   public void destroy()
   {
      removeAllPlotPanels();
      parentPanel.queueRemoveChild(searchPanel.getPanel());
   }

   private void removeAllPlotPanels()
   {
      RDXSCS2PlotPanel[] plotPanelsArray = plotPanels.toArray(new RDXSCS2PlotPanel[0]);
      for (RDXSCS2PlotPanel plotPanel : plotPanelsArray)
      {
         removePlotPanel(plotPanel);
      }
   }

   public void addVariableWidget(String variableName)
   {
      variableWidgets.add(new RDXLinkedYoBooleanWidget(yoManager, variableName));
   }

   public RDXSCS2PlotPanel getPlotPanel(String name)
   {
      for (RDXSCS2PlotPanel plotPanel : plotPanels)
         if (plotPanel.getPanelName().equals(name))
            return plotPanel;

      return null;
   }
}
