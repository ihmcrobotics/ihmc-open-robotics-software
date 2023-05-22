package us.ihmc.rdx.ui;

import us.ihmc.rdx.tools.RDXIconTexture;

public class RDXButtonIconTextures
{
   private final RDXIconTexture upIcon;
   private final RDXIconTexture hoverIcon;
   private final RDXIconTexture downIcon;

   public RDXButtonIconTextures(RDXIconTexture upIcon, RDXIconTexture hoverIcon, RDXIconTexture downIcon)
   {
      this.upIcon = upIcon;
      this.hoverIcon = hoverIcon;
      this.downIcon = downIcon;
   }

   public RDXIconTexture getUpIcon()
   {
      return upIcon;
   }

   public RDXIconTexture getHoverIcon()
   {
      return hoverIcon;
   }

   public RDXIconTexture getDownIcon()
   {
      return downIcon;
   }
}
