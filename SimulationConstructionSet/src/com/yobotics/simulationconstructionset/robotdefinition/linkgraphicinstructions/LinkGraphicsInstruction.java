package com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions;

import com.yobotics.simulationconstructionset.graphics.YoAppearanceDefinition;

public abstract class LinkGraphicsInstruction implements LinkGraphicsPrimitiveInstruction
{

   boolean hasChanged = false;
   protected YoAppearanceDefinition appearance = null;

   public final boolean hasChangedSinceLastCalled()
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
   }

   public final void reset()
   {
      hasChanged = false;
   }

}
