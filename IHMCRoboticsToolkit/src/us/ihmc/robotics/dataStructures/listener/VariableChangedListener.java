package us.ihmc.robotics.dataStructures.listener;

import us.ihmc.robotics.dataStructures.variable.YoVariable;

/**
 * @author Jerry Pratt
 */

public interface VariableChangedListener
{
   public void variableChanged(YoVariable<?> v);
}
