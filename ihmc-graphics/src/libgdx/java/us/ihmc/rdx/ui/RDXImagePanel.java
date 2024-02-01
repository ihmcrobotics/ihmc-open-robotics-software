package us.ihmc.rdx.ui;

import com.badlogic.gdx.graphics.Texture;
import imgui.internal.ImGui;
import us.ihmc.rdx.imgui.RDXPanel;

public class RDXImagePanel extends RDXPanel
{
   public static final boolean FLIP_Y = true;
   public static final boolean DO_NOT_FLIP_Y = false;

   private Texture texture;
   private final boolean flipY;
   private float mouseXRightFromLeft;
   private float mouseYDownFromTop;
   private Runnable userImGuiImageInteraction;

   public RDXImagePanel(String name, boolean flipY)
   {
      super(name);
      setRenderMethod(this::renderImGuiWidgets);
      this.flipY = flipY;
   }

   public void setTexture(Texture texture)
   {
      this.texture = texture;
      setFirstTimeWidth(texture.getWidth());
      setFirstTimeHeight(texture.getHeight());
   }

   public void renderImGuiWidgets()
   {
      float rawWindowPosX = ImGui.getWindowPosX();
      float rawWindowPosY = ImGui.getWindowPosY();

      mouseXRightFromLeft = ImGui.getMousePosX() - rawWindowPosX;
      mouseYDownFromTop = ImGui.getMousePosY() - rawWindowPosY;

      if (userImGuiImageInteraction != null)
      {
         userImGuiImageInteraction.run();
      }

      if (texture != null)
      {
         //      float posX = ImGui.getWindowPosX() + ImGui.getWindowContentRegionMinX();
         //      float posY = ImGui.getWindowPosY() + ImGui.getWindowContentRegionMinY();
         //      float sizeX = ImGui.getWindowContentRegionMaxX();
         //      float sizeY = ImGui.getWindowContentRegionMaxY();
         float tableHeader = 22.0f;
         float posX = rawWindowPosX;
         float posY = rawWindowPosY + tableHeader;
         float sizeX = ImGui.getWindowSizeX();
         float sizeY = ImGui.getWindowSizeY() - tableHeader;

         float windowAspect = sizeX / sizeY;
         float cameraAspect = (float) texture.getWidth() / (float) texture.getHeight();
         float drawSizeX = sizeX;
         float drawSizeY = sizeY;
         float centeringX = 0.0f;
         float centeringY = 0.0f;
         if (windowAspect > cameraAspect)
         {
            drawSizeX = drawSizeY * cameraAspect;
            centeringX = (sizeX - drawSizeX) / 2.0f;
         }
         else
         {
            drawSizeY = drawSizeX / cameraAspect;
            centeringY = (sizeY - drawSizeY) / 2.0f;
         }
         float startX = posX + centeringX;
         float startY = flipY ? posY + centeringY + drawSizeY : posY + centeringY;
         float endX = posX + centeringX + drawSizeX;
         float endY = flipY ? posY + centeringY : posY + centeringY + drawSizeY;

         ImGui.getWindowDrawList().addImage(texture.getTextureObjectHandle(), startX, startY, endX, endY);
      }
   }

   public void setUserImGuiImageInteraction(Runnable userImGuiImageInteraction)
   {
      this.userImGuiImageInteraction = userImGuiImageInteraction;
   }

   public float getMouseXRightFromLeft()
   {
      return mouseXRightFromLeft;
   }

   public float getMouseYDownFromTop()
   {
      return mouseYDownFromTop;
   }
}
