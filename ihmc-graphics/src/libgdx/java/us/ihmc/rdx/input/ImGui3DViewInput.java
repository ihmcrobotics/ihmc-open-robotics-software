package us.ihmc.rdx.input;

import com.badlogic.gdx.math.Vector3;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDX3DPanel;

import java.nio.ByteBuffer;
import java.util.ArrayList;

/**
 * This class is just to do the non-trivial or high computation stuff.
 * Simple things like ImGui.isKeyDown('W') should still just use the
 * ImGui API for now.
 */
public class ImGui3DViewInput
{
   private final RDX3DPanel panel;
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
   private final ArrayList<ImGui3DViewPickResult> pickResults = new ArrayList<>();
   private ImGui3DViewPickResult closestPick = null;
   private final FramePoint3D tempCameraPose = new FramePoint3D();
   private final FramePoint3D pickPoint = new FramePoint3D();
   private double lastZCollision;

   public ImGui3DViewInput(RDX3DPanel panel)
   {
      this.panel = panel;
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
         mouseDragDatum.update();

      pickResults.clear();
   }

   public Line3DReadOnly getPickRayInWorld()
   {
      if (!computedPickRay)
      {
         computedPickRay = true;

         float viewportWidth = panel.getViewportSizeX();
         float viewportHeight = panel.getViewportSizeY();

         float viewportX = (2.0f * getMousePosX()) / viewportWidth - 1.0f;
         float viewportY = (2.0f * (viewportHeight - getMousePosY())) / viewportHeight - 1.0f;

         gdxOrigin.set(viewportX, viewportY, -1.0f);
         gdxOrigin.prj(panel.getCamera3D().invProjectionView);

         gdxDirection.set(viewportX, viewportY, 1.0f);
         gdxDirection.prj(panel.getCamera3D().invProjectionView);

         gdxDirection.sub(gdxOrigin).nor();

         LibGDXTools.toEuclid(gdxOrigin, pickRayInWorld.getPoint());
         LibGDXTools.toEuclid(gdxDirection, pickRayInWorld.getDirection());
      }

      return pickRayInWorld;
   }

   public Point3DReadOnly getPickPointInWorld()
   {
      return getPickPointInWorld(Double.NaN);
   }

   public Point3DReadOnly getPickPointInWorld(double fallbackXYPlaneIntersectionHeight)
   {
      ByteBuffer depthBuffer = panel.getNormalizedDeviceCoordinateDepthDirectByteBuffer();

      boolean mouseInBounds = true;
      mouseInBounds &= mousePosX >= 0.0f;
      mouseInBounds &= mousePosY >= 0.0f;
      mouseInBounds &= mousePosX < panel.getViewportSizeX();
      mouseInBounds &= mousePosY < panel.getViewportSizeY();

      if (mouseInBounds)
      {
         boolean fallbackToXYPlaneIntersection = true;

         if (depthBuffer != null)
         {
            int mousePosXInt = (int) mousePosX;
            int mousePosYInt = (int) mousePosY;
            int aliasedRenderedAreaWidth = (int) panel.getRenderSizeX();
            int aliasedRenderedAreaHeight = (int) panel.getRenderSizeY();
            int antiAliasing = panel.getAntiAliasing();
            int aliasedMouseY = mousePosYInt * antiAliasing;
            int aliasedMouseX = mousePosXInt * antiAliasing;
            int aliasedFlippedMouseY = aliasedRenderedAreaHeight - aliasedMouseY;

            int rowAdjustment = aliasedFlippedMouseY * aliasedRenderedAreaWidth * Float.BYTES;
            int columnAdjustment = aliasedMouseX * Float.BYTES;
            float normalizedDeviceCoordinateZ = depthBuffer.getFloat(rowAdjustment + columnAdjustment);

            if (normalizedDeviceCoordinateZ > 0.503)
            {
               fallbackToXYPlaneIntersection = false;

               float cameraNear = panel.getCamera3D().near;
               float cameraFar = panel.getCamera3D().far;
               float twoXCameraFarNear = 2.0f * cameraNear * cameraFar;
               float farPlusNear = cameraFar + cameraNear;
               float farMinusNear = cameraFar - cameraNear;
               float eyeDepth = (twoXCameraFarNear / (farPlusNear - normalizedDeviceCoordinateZ * farMinusNear));

               float principalOffsetXPixels = aliasedRenderedAreaWidth / 2.0f;
               float principalOffsetYPixels = aliasedRenderedAreaHeight / 2.0f;
               float fieldOfViewY = panel.getCamera3D().getVerticalFieldOfView();
               float focalLengthPixels = (float) ((aliasedRenderedAreaHeight / 2.0) / Math.tan(Math.toRadians((fieldOfViewY / 2.0))));
               float zUp3DX = eyeDepth;
               float zUp3DY = -(aliasedMouseX - principalOffsetXPixels) / focalLengthPixels * eyeDepth;
               float zUp3DZ = -(aliasedMouseY - principalOffsetYPixels) / focalLengthPixels * eyeDepth;

               tempCameraPose.setToZero(panel.getCamera3D().getCameraFrame());
               tempCameraPose.changeFrame(ReferenceFrame.getWorldFrame());

               pickPoint.setIncludingFrame(panel.getCamera3D().getCameraFrame(), zUp3DX, zUp3DY, zUp3DZ);
               pickPoint.changeFrame(ReferenceFrame.getWorldFrame());
               lastZCollision = pickPoint.getZ();
            }
         }

         if (fallbackToXYPlaneIntersection)
         {
            double xyZHeight = Double.isNaN(fallbackXYPlaneIntersectionHeight) ? lastZCollision : fallbackXYPlaneIntersectionHeight;
            pickPoint.setIncludingFrame(ReferenceFrame.getWorldFrame(), EuclidCoreTools.origin3D);
            pickPoint.setZ(xyZHeight);
            getPickRayInWorld();
            EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pickPoint, Axis3D.Z, pickRayInWorld.getPoint(), pickRayInWorld.getDirection(), pickPoint);
         }
      }

      return pickPoint;
   }

   public void addPickResult(ImGui3DViewPickResult pickResult)
   {
      pickResults.add(pickResult);
   }

   public void calculateClosestPick()
   {
      closestPick = null;
      for (ImGui3DViewPickResult pickResult : pickResults)
      {
         if (closestPick == null)
         {
            closestPick = pickResult;
         }
         else if (pickResult.getDistanceToCamera() < closestPick.getDistanceToCamera())
         {
            closestPick = pickResult;
         }
      }
   }

   /**
    * This is a better way to detect a singular mouse click than any of the provided methods.
    */
   public boolean mouseReleasedWithoutDrag(int button)
   {
      return ImGui.getMouseDragDeltaX(button) == 0.0f && ImGui.getMouseDragDeltaY(button) == 0.0f && ImGui.isMouseReleased(button);
   }

   public boolean isWindowHovered()
   {
      return isWindowHovered;
   }

   public ImGuiMouseDragData getMouseDragData(int imGuiMouseButton)
   {
      return mouseDragData[imGuiMouseButton];
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

   /**
    * null if no collisions
    */
   public ImGui3DViewPickResult getClosestPick()
   {
      return closestPick;
   }
}
