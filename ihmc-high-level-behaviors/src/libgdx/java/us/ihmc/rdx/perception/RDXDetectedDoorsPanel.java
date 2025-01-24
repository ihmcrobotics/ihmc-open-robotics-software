package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.flag.ImGuiTableFlags;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;

public class RDXDetectedDoorsPanel extends RDXPanel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public RDXDetectedDoorsPanel()
   {
      super("Doors");
      setRenderMethod(this::render);
   }

   public void render()
   {
      if (ImGui.beginTable(labels.get("Doors table"), 2, ImGuiTableFlags.None))
      {
         ImGui.tableNextRow();
         ImGui.tableSetColumnIndex(0);
         ImGui.checkbox(labels.get("Showing"), true);
         ImGui.tableSetColumnIndex(1);
         ImGui.text("Door name / ID");
         ImGui.endTable();
      }
   }
}
