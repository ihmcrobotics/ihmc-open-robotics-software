package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;

public interface ICPControlHeuristics
{
   boolean cropCoPControlArea(FramePoint2DReadOnly currentICP, FramePoint2DReadOnly icpAtEndOfState, FrameConvexPolygon2DBasics controlRegionToCrop);
}
