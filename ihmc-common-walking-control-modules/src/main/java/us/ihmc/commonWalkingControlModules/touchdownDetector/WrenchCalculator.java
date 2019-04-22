package us.ihmc.commonWalkingControlModules.touchdownDetector;

import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;

public interface WrenchCalculator
{
   void calculate();
   WrenchReadOnly getWrench();
   String getName();
   boolean isTorquingIntoJointLimit();
}
