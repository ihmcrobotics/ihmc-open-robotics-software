package us.ihmc.parameterTuner.guiElements;

import us.ihmc.yoVariables.parameters.ParameterLoadStatus;

public enum GuiParameterStatus
{
   ANY,
   UNLOADED,
   DEFAULT,
   FILE_LOADED,
   MODIFIED;

   public static GuiParameterStatus get(ParameterLoadStatus parameterLoadStatus)
   {
      switch (parameterLoadStatus)
      {
      case UNLOADED:
         return GuiParameterStatus.UNLOADED;
      case DEFAULT:
         return GuiParameterStatus.DEFAULT;
      case FILE:
         return GuiParameterStatus.FILE_LOADED;
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
