package us.ihmc.rdx.simulation.scs2;

import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.RegularExpression;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;

public class RDXSCS2YoVariableSearchPanel
{
   private final RDXPanel panel = new RDXPanel("YoVariable Search", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString searchBar = new ImString();
   private final ImBoolean showFullName = new ImBoolean(true);
   private YoRegistry yoRegistry;
   private final ArrayList<YoVariable> allVariables = new ArrayList<>();
   private YoVariable selectedVariable = null;

   public RDXSCS2YoVariableSearchPanel(YoRegistry yoRegistry)
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

      ImGui.text("Registry contains " + allVariables.size() + " variables.");
      ImGui.sameLine();
      ImGuiTools.smallCheckbox(labels.get("Show Full Name"), showFullName);
      ImGui.separator();

      if (!ImGui.isMouseDown(ImGuiMouseButton.Left))
         selectedVariable = null;

      if (ImGui.beginListBox("##YoVariables", ImGui.getColumnWidth(), ImGui.getWindowSizeY() - 100))
      {
         for (YoVariable yoVariable : allVariables)
         {
            if (!RegularExpression.check(yoVariable.getFullNameString(), searchBar.get()))
               continue;

            ImGui.selectable((showFullName.get() ? yoVariable.getFullNameString() : yoVariable.getName())
                             + " (" + yoVariable.getClass().getSimpleName() + ": " + yoVariable.getValueAsString("%.5f") + ")",
                             yoVariable == selectedVariable);
            if (ImGui.isItemClicked())
            {
               LogTools.info("Selected variable: " + yoVariable.getFullNameString());
               selectedVariable = yoVariable;
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

   public YoVariable getSelectedVariable()
   {
      return selectedVariable;
   }
}
