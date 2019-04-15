package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.time.TimeIntervalProvider;

/**
 * Provides the contact state that constitutes the contact sequence used by the {@link CoMTrajectoryPlanner}. This includes
 * the starting and ending CoP position, the time interval, and the contact state {@link ContactState}.
 */
public interface ContactStateProvider extends TimeIntervalProvider
{
   /**
    * Provides the starting CoP position for the current contact state.
    */
   FramePoint3DReadOnly getCopStartPosition();

   /**
    * Provides the starting CoP position for the current contact state.
    */
   FramePoint3DReadOnly getCopEndPosition();

   /**
    * Specifies whether the current state is in contact or not.
    */
   ContactState getContactState();
}
