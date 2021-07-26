package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;

public interface PlanForToeOffCalculator
{
   boolean shouldPutCMPOnToes(double stepHeight, FramePoint3DReadOnly currentCoPPositionInSole, FrameConvexPolygon2DReadOnly supportFootPolygon);
}
