package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;

public interface DesiredVelocityControlModule
{
   ReferenceFrame getReferenceFrame();

   FrameVector2DReadOnly getDesiredVelocity();
}
