package us.ihmc.gdx.perception;

import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;

public class GDXCVImagePanel
{
   private final ImGuiVideoPanel videoPanel;
   private GDXPixmapBytedecoImage pixmapImage;
   private Texture panelTexture;

   public GDXCVImagePanel(String name, int imageWidth, int imageHeight)
   {
      videoPanel = new ImGuiVideoPanel(name, false);
      pixmapImage = new GDXPixmapBytedecoImage(imageWidth, imageHeight);
      panelTexture = new Texture(new PixmapTextureData(pixmapImage.getPixmap(), null, false, false));
      videoPanel.setTexture(panelTexture);
   }

   /**
    * Texture must be drawn to before panel will display the image.
    * This is where the image gets transferred to the GPU.
    */
   public void draw()
   {
      panelTexture.draw(pixmapImage.getPixmap(), 0, 0);
   }

   public ImGuiVideoPanel getVideoPanel()
   {
      return videoPanel;
   }

   public GDXPixmapBytedecoImage getPixmapImage()
   {
      return pixmapImage;
   }
}
