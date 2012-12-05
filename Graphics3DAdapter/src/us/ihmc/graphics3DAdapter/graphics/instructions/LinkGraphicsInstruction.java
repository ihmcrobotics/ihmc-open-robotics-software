package us.ihmc.graphics3DAdapter.graphics.instructions;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceDefinition;

public abstract class LinkGraphicsInstruction implements LinkGraphicsPrimitiveInstruction
{

   boolean hasChanged = false;
   protected YoAppearanceDefinition appearance = null;

   public final boolean hasChanged()
   {
      return hasChanged;
   }

   public final YoAppearanceDefinition getAppearance()
   {
      return appearance;
   }

   public final void setAppearance(YoAppearanceDefinition appearance)
   {
      this.appearance = appearance;
      hasChanged = true;
   }

   public final void reset()
   {
      hasChanged = false;
   }

}
