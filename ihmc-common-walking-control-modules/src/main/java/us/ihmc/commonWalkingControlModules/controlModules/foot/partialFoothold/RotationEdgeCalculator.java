package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.robotics.SCS2YoGraphicHolder;

public interface RotationEdgeCalculator extends SCS2YoGraphicHolder
{
   boolean compute(FramePoint2DReadOnly measuredCoP);

   void reset();

   FrameLine2DReadOnly getLineOfRotation();

   boolean isRotationEdgeTrusted();
}
