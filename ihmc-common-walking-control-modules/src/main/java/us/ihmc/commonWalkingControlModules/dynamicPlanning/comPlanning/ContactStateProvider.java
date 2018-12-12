package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.quadrupedBasics.gait.TimeIntervalProvider;

public interface ContactStateProvider extends TimeIntervalProvider
{
   FramePoint3DReadOnly getCopPosition();
   ContactState getContactState();
}
