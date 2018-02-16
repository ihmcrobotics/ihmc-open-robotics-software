package us.ihmc.parameterTuner.guiElements;

import us.ihmc.yoVariables.parameters.ParameterLoadStatus;

public enum GuiParameterStatus
{
   ANY,
   DEFAULT,
   LOADED,
   MODIFIED;

   public static GuiParameterStatus get(ParameterLoadStatus parameterLoadStatus)
   {
      switch (parameterLoadStatus)
      {
      case DEFAULT:
         return GuiParameterStatus.DEFAULT;
      case LOADED:
         return GuiParameterStatus.LOADED;
      default:
         throw new RuntimeException("Unknown parameter load status: " + parameterLoadStatus);
      }
   }

   public boolean matches(GuiParameter parameter)
   {
      return matches(parameter.getStatus());
   }

   public boolean matches(GuiParameterStatus other)
   {
      return this == ANY || other == this;
   }
}
