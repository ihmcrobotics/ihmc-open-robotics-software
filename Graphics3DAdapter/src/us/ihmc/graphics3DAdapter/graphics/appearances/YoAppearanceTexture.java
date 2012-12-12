package us.ihmc.graphics3DAdapter.graphics.appearances;

import java.awt.Component;
import java.net.URL;

public class YoAppearanceTexture extends YoAppearanceTransparency
{
   private final URL fileURL;
   private final Component component;

   public YoAppearanceTexture(URL fileURL, Component component)
   {
      super();
      this.fileURL = fileURL;
      this.component = component;
   }

   public URL getFileURL()
   {
      return fileURL;
   }

   public Component getComponent()
   {
      return component;
   }

}
