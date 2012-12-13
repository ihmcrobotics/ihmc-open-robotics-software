package us.ihmc.graphics3DAdapter.graphics.instructions;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;

public abstract class Graphics3DInstruction implements Graphics3DPrimitiveInstruction
{
   private boolean hasChanged = false;
   private AppearanceDefinition appearance = null;

   public final boolean hasChanged()
   {
      return hasChanged;
   }
   
   public final void setHasChanged()
   {
      this.hasChanged = true;
   }
   
   public final void resetHasChanged()
   {
      hasChanged = false;
   }

   public final AppearanceDefinition getAppearance()
   {
      return appearance;
   }

   public final void setAppearance(AppearanceDefinition appearance)
   {
      this.appearance = appearance;
      hasChanged = true;
   }

   

}
