package us.ihmc.robotics.dataStructures.listener;

import us.ihmc.robotics.dataStructures.variable.YoVariable;

/**
 * Title:        SimulationConstructionSet
 * Description:
 * Copyright:    Copyright (c) 2000
 * Company:      Yobotics, Inc.
 * @author Jerry Pratt
 * @version 1.0
 */

public interface VariableChangedListener
{
   public void variableChanged(YoVariable<?> v);
}
