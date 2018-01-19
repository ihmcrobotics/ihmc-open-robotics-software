package us.ihmc.robotics.parameterGui.tree;

import us.ihmc.yoVariables.parameters.xml.Parameter;

public class ParameterTreeParameter extends ParameterTreeValue
{
   private final Parameter parameter;

   public ParameterTreeParameter(Parameter parameter)
   {
      this.parameter = parameter;
   }

   @Override
   public boolean isRegistry()
   {
      return false;
   }

   @Override
   public String getDescription()
   {
      return parameter.getDescription();
   }

   @Override
   public String getName()
   {
      return parameter.getName();
   }

   public Parameter getParameter()
   {
      return parameter;
   }
}
