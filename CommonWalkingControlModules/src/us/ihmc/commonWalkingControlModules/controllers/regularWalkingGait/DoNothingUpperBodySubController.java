package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.UpperBodyTorques;

public class DoNothingUpperBodySubController implements UpperBodySubController
{
   public void doUpperBodyControl(UpperBodyTorques upperBodyTorquesToPack)
   {
      upperBodyTorquesToPack.setTorquesToZero();
   }
}
