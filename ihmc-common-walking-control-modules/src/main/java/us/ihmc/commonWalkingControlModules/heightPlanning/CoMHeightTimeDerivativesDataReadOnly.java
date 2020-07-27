package us.ihmc.commonWalkingControlModules.heightPlanning;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public interface CoMHeightTimeDerivativesDataReadOnly
{
   ReferenceFrame getReferenceFrame();

   double getComHeightInFrame();

   void getComHeight(FramePoint3DBasics framePointToPack);

   double getComHeightVelocity();

   double getComHeightAcceleration();

   double getComHeightJerk();
}
