package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;

public interface CoPWaypointCalculator<T extends TimedContactInterval>
{
   void computeCoPWaypoint(FixedFramePoint3DBasics copWaypointToPack, T contactSequenceProvider);
}
