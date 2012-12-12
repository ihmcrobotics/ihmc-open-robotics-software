package us.ihmc.graphics3DAdapter.graphics.instructions;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceDefinition;

public abstract class LinkGraphicsInstruction implements LinkGraphicsPrimitiveInstruction
{

   private boolean hasChanged = false;
   private YoAppearanceDefinition appearance = null;

   public final boolean hasChanged()
   {
      return hasChanged;
   }
   
   public void setHasChanged()
   {
      this.hasChanged = true;
   }
   
   public final void resetHasChanged()
   {
      hasChanged = false;
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

   

}
