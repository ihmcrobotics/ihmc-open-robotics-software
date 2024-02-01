package us.ihmc.rdx.imgui;

import imgui.flag.ImGuiDockNodeFlags;
import imgui.flag.ImGuiStyleVar;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.log.LogTools;

public class RDXDockspacePanel extends RDXPanelSizeHandler
{
   private final String name;
   private final ImBoolean isShowing = new ImBoolean(true);
   private int dockspaceID = -1;
   private boolean wasJustClosed = false;
   private boolean shownLastTick = false;
   private int windowViewportID;

   public RDXDockspacePanel(String name)
   {
      this.name = name;
   }

   public void renderPanel()
   {
      boolean shownThisTick = isShowing.get();
      if (shownThisTick)
      {
         ImGui.pushStyleVar(ImGuiStyleVar.WindowPadding, 0.0f, 0.0f);
         handleSizeBeforeBegin();
         ImGui.begin(name, isShowing);
         handleSizeAfterBegin();
         ImGui.popStyleVar();

         windowViewportID = ImGui.getWindowViewport().getID();

         // Info here: https://github.com/ocornut/imgui/blob/docking/imgui_demo.cpp#L7408
         int dockNodeFlags = ImGuiDockNodeFlags.None;
         dockNodeFlags += ImGuiDockNodeFlags.PassthruCentralNode;
         int id = ImGui.getID(name);
         if (dockspaceID != id)
         {
            LogTools.info("Dockspace ID changed. {}: {} -> {}", name, dockspaceID, id);
         }
         dockspaceID = id;
         ImGui.dockSpace(id, 0, 0, dockNodeFlags);

         ImGui.end();
      }

      wasJustClosed = !shownThisTick && shownLastTick;
      shownLastTick = shownThisTick;
   }

   public void renderMenuItem()
   {
      ImGui.checkbox(name + "###DockspacePanel" + name, isShowing);
   }

   public ImBoolean getIsShowing()
   {
      return isShowing;
   }

   public boolean getWasJustClosed()
   {
      return wasJustClosed;
   }

   public String getName()
   {
      return name;
   }

   public int getDockspaceID()
   {
      return dockspaceID;
   }

   public int getWindowViewportID()
   {
      return windowViewportID;
   }
}
