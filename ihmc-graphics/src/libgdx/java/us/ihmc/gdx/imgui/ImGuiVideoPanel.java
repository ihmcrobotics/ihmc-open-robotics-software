package us.ihmc.gdx.imgui;

import com.badlogic.gdx.graphics.Texture;
import imgui.internal.ImGui;

public class ImGuiVideoPanel extends ImGuiPanel
{
   private Texture texture;
   private final boolean flipY;

   public ImGuiVideoPanel(String name, boolean flipY)
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
      if (texture != null)
      {
         //      float posX = ImGui.getWindowPosX() + ImGui.getWindowContentRegionMinX();
         //      float posY = ImGui.getWindowPosY() + ImGui.getWindowContentRegionMinY();
         //      float sizeX = ImGui.getWindowContentRegionMaxX();
         //      float sizeY = ImGui.getWindowContentRegionMaxY();
         float tableHeader = 22.0f;
         float posX = ImGui.getWindowPosX();
         float posY = ImGui.getWindowPosY() + tableHeader;
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
}
