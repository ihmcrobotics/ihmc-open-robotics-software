package us.ihmc.quadrupedRobotics.estimator.footSwitch;

import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;

public interface WrenchCalculator
{
   void calculate();
   WrenchReadOnly getWrench();
}
