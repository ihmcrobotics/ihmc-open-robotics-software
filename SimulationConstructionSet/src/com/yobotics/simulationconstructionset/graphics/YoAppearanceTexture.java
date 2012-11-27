package com.yobotics.simulationconstructionset.graphics;

import java.awt.Component;
import java.net.URL;

public class YoAppearanceTexture implements YoAppearanceDefinition
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
