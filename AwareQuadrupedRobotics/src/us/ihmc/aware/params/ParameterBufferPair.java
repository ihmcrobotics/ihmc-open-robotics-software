package us.ihmc.aware.params;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class ParameterBufferPair
{
   public final DoubleYoVariable[] variables;
   public final double[] buffer;

   public ParameterBufferPair(DoubleYoVariable[] variables, double[] buffer)
   {
      this.variables = variables;
      this.buffer = buffer;
   }
}
