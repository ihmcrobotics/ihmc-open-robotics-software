package us.ihmc.gdx.input;

import com.badlogic.gdx.math.Vector3;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.tools.GDXTools;

import java.util.function.Supplier;

/**
 * This class is just to do the non-trivial or high computation stuff.
 * Simple things like ImGui.isKeyDown('W') should still just use the
 * ImGui API for now.
 */
public class ImGui3DViewInput
{
   private final FocusBasedGDXCamera camera;
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
   private final Vector3 gdxOrigin = new Vector3();
   private final Vector3 gdxDirection = new Vector3();
   private final Line3D pickRayInWorld = new Line3D();
   private boolean computedPickRay = false;
   private boolean initialized = false;

   public ImGui3DViewInput(FocusBasedGDXCamera camera, Supplier<Float> viewportSizeXSupplier, Supplier<Float> viewportSizeYSupplier)
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

      computedPickRay = false;

      isWindowHovered = ImGui.isWindowHovered();
      mousePosX = (int) ImGui.getMousePosX() - (int) ImGui.getWindowPosX();
      mousePosY = (int) ImGui.getMousePosY() - (int) ImGui.getWindowPosY() - (int) ImGuiTools.TAB_BAR_HEIGHT;
      mouseWheelDelta = -ImGui.getIO().getMouseWheel();

      for (ImGuiMouseDragData mouseDragDatum : mouseDragData)
         mouseDragDatum.update(isWindowHovered);
   }

   public Line3DReadOnly getPickRayInWorld()
   {
      if (!computedPickRay)
      {
         computedPickRay = true;

         float viewportWidth = viewportSizeXSupplier.get();
         float viewportHeight = viewportSizeYSupplier.get();

         float viewportX = (2.0f * getMousePosX()) / viewportWidth - 1.0f;
         float viewportY = (2.0f * (viewportHeight - getMousePosY())) / viewportHeight - 1.0f;

         gdxOrigin.set(viewportX, viewportY, -1.0f);
         gdxOrigin.prj(camera.invProjectionView);

         gdxDirection.set(viewportX, viewportY, 1.0f);
         gdxDirection.prj(camera.invProjectionView);

         gdxDirection.sub(gdxOrigin).nor();

         GDXTools.toEuclid(gdxOrigin, pickRayInWorld.getPoint());
         GDXTools.toEuclid(gdxDirection, pickRayInWorld.getDirection());
      }

      return pickRayInWorld;
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
