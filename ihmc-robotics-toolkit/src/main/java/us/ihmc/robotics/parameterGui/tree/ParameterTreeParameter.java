package us.ihmc.robotics.parameterGui.tree;

import us.ihmc.robotics.parameterGui.GuiParameter;

public class ParameterTreeParameter extends ParameterTreeValue
{
   private final GuiParameter parameter;

   public ParameterTreeParameter(GuiParameter parameter)
   {
      this.parameter = parameter;
   }

   @Override
   public boolean isRegistry()
   {
      return false;
   }

   @Override
   public String getName()
   {
      return parameter.getName();
   }

   public GuiParameter getParameter()
   {
      return parameter;
   }
}
