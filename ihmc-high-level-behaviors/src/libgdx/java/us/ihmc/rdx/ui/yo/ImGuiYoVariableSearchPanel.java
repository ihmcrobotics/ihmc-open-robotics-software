package us.ihmc.rdx.ui.yo;

import imgui.ImGui;
import imgui.type.ImString;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.RegularExpression;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;

public class ImGuiYoVariableSearchPanel
{
   private final RDXPanel panel = new RDXPanel("YoVariable Search", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString searchBar = new ImString();
   private YoRegistry yoRegistry;
   private final ArrayList<YoVariable> allVariables = new ArrayList<>();
   private boolean searchRequested = false;
   private YoVariable selectedVariable = null;

   public ImGuiYoVariableSearchPanel(YoRegistry yoRegistry)
   {
      changeYoRegistry(yoRegistry);

      panel.setFirstTimeWidth(800);
      panel.setFirstTimeHeight(800);
   }

   /**
    * Used when switching between sessions.
    */
   public void changeYoRegistry(YoRegistry yoRegistry)
   {
      this.yoRegistry = yoRegistry;
   }

   private void renderImGuiWidgets()
   {
      ImGui.inputText(labels.get("Search"), searchBar);

      if (searchRequested)
      {
         ImGui.sameLine();
         if (ImGui.button("Cancel"))
         {
            searchRequested = false;
            panel.getIsShowing().set(false);
         }
      }

      ImGui.text("Registry contains " + allVariables.size() + " variables.");
      ImGui.separator();


      if (ImGui.beginListBox("##YoVariables", ImGui.getColumnWidth(), ImGui.getWindowSizeY() - 100))
      {
         for (YoVariable yoVariable : allVariables)
         {
            if (!RegularExpression.check(yoVariable.getFullNameString(), searchBar.get()))
               continue;

            ImGui.selectable(yoVariable.getFullNameString() + " (" + yoVariable.getClass().getSimpleName() + ": " + yoVariable.getValueAsString("%.5f") + ")");
            if (ImGui.isItemClicked())
            {
               selectedVariable = yoVariable;
               searchRequested = false;
               panel.getIsShowing().set(false);
            }

            if (ImGui.beginPopupContextItem())
            {
               if (ImGui.button("Copy name"))
               {
                  ImGui.setClipboardText(yoVariable.getName());
                  ImGui.closeCurrentPopup();
               }
               if (ImGui.button("Copy full name"))
               {
                  ImGui.setClipboardText(yoVariable.getFullNameString());
                  ImGui.closeCurrentPopup();
               }

               ImGui.endPopup();
            }
         }
         ImGui.endListBox();
      }
   }

   public void initializeYoVariablesAfterSessionStart()
   {
      addAllVariableNamesRecursively(yoRegistry);
   }

   private void addAllVariableNamesRecursively(YoRegistry registry)
   {
      for (YoVariable variable : registry.getVariables())
      {
         allVariables.add(variable);
      }

      for (YoRegistry child : registry.getChildren())
      {
         addAllVariableNamesRecursively(child);
      }
   }

   public RDXPanel getPanel()
   {
      return panel;
   }

   public void setSearchRequested(boolean searchRequested)
   {
      if (searchRequested)
         panel.getIsShowing().set(true);

      this.searchRequested = searchRequested;
   }

   public boolean getSearchRequested()
   {
      return searchRequested;
   }

   public YoVariable getSelectedVariable()
   {
      return selectedVariable;
   }

   public void setSelectedVariable(YoVariable selectedVariable)
   {
      this.selectedVariable = selectedVariable;
   }
}
