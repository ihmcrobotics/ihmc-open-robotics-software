package us.ihmc.commonWalkingControlModules.heightPlanning;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;

public interface CoMHeightTimeDerivativesDataReadOnly
{
   ReferenceFrame getReferenceFrame();

   double getComHeightInFrame();

   void getComHeight(FramePoint3DBasics framePointToPack);

   double getComHeightVelocity();

   double getComHeightAcceleration();

   double getComHeightJerk();
}
