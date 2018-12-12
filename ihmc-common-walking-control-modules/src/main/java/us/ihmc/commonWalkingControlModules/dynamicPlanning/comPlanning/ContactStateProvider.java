package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.time.TimeIntervalProvider;

public interface ContactStateProvider extends TimeIntervalProvider
{
   FramePoint3DReadOnly getCopStartPosition();
   FramePoint3DReadOnly getCopEndPosition();
   ContactState getContactState();
}
