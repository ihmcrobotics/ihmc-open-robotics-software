package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

/**
 * Provides the contact state that constitutes the contact sequence used by the {@link CoMTrajectoryPlannerInterface}. This includes
 * the starting and ending eCMP position, the time interval, and the contact state {@link ContactState}.
 */
public interface ContactStateBasics<T extends ContactStateBasics<T>> extends ContactStateProvider<T>
{
   default void set(ContactStateProvider<?> other)
   {
      reset();
      setStartECMPPosition(other.getECMPStartPosition());
      setEndECMPPosition(other.getECMPEndPosition());
      setStartECMPVelocity(other.getECMPStartVelocity());
      setEndECMPVelocity(other.getECMPEndVelocity());
      getTimeInterval().set(other.getTimeInterval());
      setContactState(other.getContactState());
   }

   void reset();

   void setStartECMPPosition(FramePoint3DReadOnly startPosition);

   void setEndECMPPosition(FramePoint3DReadOnly endPosition);

   void setStartECMPVelocity(FrameVector3DReadOnly startVelocity);

   void setEndECMPVelocity(FrameVector3DReadOnly endVelocity);

   void setContactState(ContactState contactState);
}
