package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.quadrupedBasics.gait.TimeIntervalProvider;
import us.ihmc.quadrupedRobotics.planning.ContactState;

public interface ContactStateProvider extends TimeIntervalProvider
{
   FramePoint3DReadOnly getCopPosition();
   ContactState getContactState();

}
