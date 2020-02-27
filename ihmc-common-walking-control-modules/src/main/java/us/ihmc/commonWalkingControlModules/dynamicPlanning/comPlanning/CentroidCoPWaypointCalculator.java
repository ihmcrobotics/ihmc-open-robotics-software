package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;

public class CentroidCoPWaypointCalculator implements CoPWaypointCalculator<TimedContactInterval>
{
   @Override
   public void computeCoPWaypoint(FixedFramePoint3DBasics copWaypointToPack, TimedContactInterval contactSequenceProvider)
   {
      copWaypointToPack.set(contactSequenceProvider.getSupportPolygon().getCentroid());
   }
}
