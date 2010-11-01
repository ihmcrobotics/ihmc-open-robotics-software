package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.UpperBodyTorques;

public interface UpperBodySubController
{
   public abstract void doUpperBodyControl(UpperBodyTorques upperBodyTorquesToPack);

}
