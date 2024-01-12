package us.ihmc.rdx.ui.teleoperation;

import imgui.ImGui;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;

public class RDXIcarusOperation extends RDXPanel
{
   ImGuiUniqueLabelMap lables = new ImGuiUniqueLabelMap(getClass());

   public RDXIcarusOperation()
   {
      super("Operation");
      setRenderMethod(this::renderImGuiWidgets);
   }

   public void renderImGuiWidgets()
   {
      ImGui.button(lables.get("Spine Posture Command"));
      ImGui.button(lables.get("CoM Height Command"));
      ImGui.button(lables.get("Hand Position"));
      ImGui.button(lables.get("Hand Orientation"));
      ImGui.button(lables.get("Hand close"));
      ImGui.button(lables.get("Hand open"));
      ImGui.button(lables.get("Poser Command"));
      ImGui.button(lables.get("Mode Command"));
      ImGui.button(lables.get("Feet Position Command"));
   }
}
