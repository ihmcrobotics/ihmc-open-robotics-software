package us.ihmc.gdx.ui.yo;

import imgui.ImGui;
import imgui.type.ImString;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;

public class ImGuiYoVariableSearchPanel
{
   private final ImGuiPanel panel = new ImGuiPanel("YoVariable Search", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString searchBar = new ImString();
   private final YoRegistry yoRegistry;
   private final ArrayList<String> variableNames;

   public ImGuiYoVariableSearchPanel(YoRegistry yoRegistry)
   {
      this.yoRegistry = yoRegistry;

      variableNames = getVariableNames();

      panel.setFirstTimeWidth(800);
      panel.setFirstTimeHeight(800);
   }

   private void renderImGuiWidgets()
   {
      ImGui.inputText(labels.get("Search"), searchBar);
      ImGui.text(variableNames.size() + " variables");
      ImGui.sameLine();
      if (ImGui.button("Cancel"))
      {

      }
   }

   public ArrayList<String> getVariableNames()
   {
      ArrayList<String> variableNames = new ArrayList<>();
      getAllVariableNamesRecursively(yoRegistry, variableNames);
      return variableNames;
   }

   private void getAllVariableNamesRecursively(YoRegistry registry, ArrayList<String> variableNames)
   {
      for (YoVariable variable : registry.getVariables())
      {
         variableNames.add(variable.getName());
      }

      for (YoRegistry child : registry.getChildren())
      {
         getAllVariableNamesRecursively(child, variableNames);
      }
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}
