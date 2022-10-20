package us.ihmc.rdx.input;

import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.sceneManager.RDX2DOrthographicCamera;

import java.util.function.Supplier;

/**
 * This class is just to do the non-trivial or high computation stuff.
 * Simple things like ImGui.isKeyDown('W') should still just use the
 * ImGui API for now.
 */
public class ImGui2DViewInput
{
   private final RDX2DOrthographicCamera camera;
   private final Supplier<Float> viewportSizeXSupplier;
   private final Supplier<Float> viewportSizeYSupplier;
   private final ImGuiMouseDragData mouseDragDataLeft = new ImGuiMouseDragData(ImGuiMouseButton.Left);
   private final ImGuiMouseDragData mouseDragDataRight = new ImGuiMouseDragData(ImGuiMouseButton.Right);
   private final ImGuiMouseDragData mouseDragDataMiddle = new ImGuiMouseDragData(ImGuiMouseButton.Middle);
   private final ImGuiMouseDragData[] mouseDragData = new ImGuiMouseDragData[] {mouseDragDataLeft, mouseDragDataRight, mouseDragDataMiddle};
   private float mousePosX = 0.0f;
   private float mousePosY = 0.0f;
   private boolean isWindowHovered;
   private float mouseWheelDelta;
   private boolean initialized = false;

   public ImGui2DViewInput(RDX2DOrthographicCamera camera, Supplier<Float> viewportSizeXSupplier, Supplier<Float> viewportSizeYSupplier)
   {
      this.camera = camera;
      this.viewportSizeXSupplier = viewportSizeXSupplier;
      this.viewportSizeYSupplier = viewportSizeYSupplier;
   }

   public void compute()
   {
      if (!initialized)
      {
         initialized = true;
      }

      isWindowHovered = ImGui.isWindowHovered();
      mousePosX = (int) ImGui.getMousePosX() - (int) ImGui.getWindowPosX();
      mousePosY = (int) ImGui.getMousePosY() - (int) ImGui.getWindowPosY() - (int) ImGuiTools.TAB_BAR_HEIGHT;
      mouseWheelDelta = -ImGui.getIO().getMouseWheel();

      for (ImGuiMouseDragData mouseDragDatum : mouseDragData)
         mouseDragDatum.update();
   }

   /**
    * This is a better way to detect a singular mouse click than any of the provided methods.
    */
   public boolean mouseReleasedWithoutDrag(int button)
   {
      return ImGui.getMouseDragDeltaX() == 0.0f && ImGui.getMouseDragDeltaX() == 0.0f && ImGui.isMouseReleased(button);
   }

   public boolean isWindowHovered()
   {
      return isWindowHovered;
   }

   public boolean isDragging(int imGuiMouseButton)
   {
      return mouseDragData[imGuiMouseButton].isDragging();
   }

   public float getMouseDraggedX(int imGuiMouseButton)
   {
      return mouseDragData[imGuiMouseButton].getMouseDraggedX();
   }

   public float getMouseDraggedY(int imGuiMouseButton)
   {
      return mouseDragData[imGuiMouseButton].getMouseDraggedY();
   }

   public float getMousePosX()
   {
      return mousePosX;
   }

   public float getMousePosY()
   {
      return mousePosY;
   }

   public float getMouseWheelDelta()
   {
      return mouseWheelDelta;
   }
}
