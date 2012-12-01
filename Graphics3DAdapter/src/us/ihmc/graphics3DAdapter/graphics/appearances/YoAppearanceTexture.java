package us.ihmc.graphics3DAdapter.graphics.appearances;

import java.awt.Component;
import java.net.URL;

public class YoAppearanceTexture extends YoAppearanceTransparancy
{
   private final URL fileURL;
   private final Component comp;

   public YoAppearanceTexture(URL fileURL, Component comp)
   {
      super();
      this.fileURL = fileURL;
      this.comp = comp;
   }

   public URL getFileURL()
   {
      return fileURL;
   }

   public Component getComp()
   {
      return comp;
   }

}
