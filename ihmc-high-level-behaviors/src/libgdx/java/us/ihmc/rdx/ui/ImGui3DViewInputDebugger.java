package us.ihmc.rdx.ui;

import imgui.flag.ImGuiMouseButton;
import us.ihmc.rdx.imgui.ImGuiPlot;

public class ImGui3DViewInputDebugger
{
   private final int bufferSize = 1000;
   private ImGuiPlot screenXPlot;
   private ImGuiPlot screenYPlot;
   private ImGuiPlot deltaXPlot;
   private ImGuiPlot deltaYPlot;
   private ImGuiPlot scrolledYPlot;

   public void create(RDX3DPanel panel3D)
   {
      panel3D.addImGui3DViewInputProcessor(this, input ->
      {
         if (input.isWindowHovered())
         {
            scrolledYPlot.setValue(input.getMouseWheelDelta());
            screenXPlot.setValue(input.getMousePosX());
            screenYPlot.setValue(input.getMousePosY());
         }
         if (input.getMouseDragData(ImGuiMouseButton.Left).isDragging())
         {
            deltaXPlot.setValue(input.getMouseDragData(ImGuiMouseButton.Left).getMouseDraggedX());
            deltaYPlot.setValue(input.getMouseDragData(ImGuiMouseButton.Left).getMouseDraggedY());
         }
      });

      screenXPlot = new ImGuiPlot("screenX", bufferSize);
      screenYPlot = new ImGuiPlot("screenY", bufferSize);
      deltaXPlot = new ImGuiPlot("deltaX", bufferSize);
      deltaYPlot = new ImGuiPlot("deltaY", bufferSize);
      scrolledYPlot = new ImGuiPlot("scrolledY", bufferSize);
   }

   public void render()
   {
      screenXPlot.render();
      screenYPlot.render();
      deltaXPlot.render();
      deltaYPlot.render();
      scrolledYPlot.render();
   }

   public String getWindowName()
   {
      return "Input Debugger";
   }
}
