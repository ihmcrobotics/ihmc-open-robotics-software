package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.time.TimeIntervalProvider;

import java.util.List;

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

   default boolean hasCopConstraintRegion()
   {
      return false;
   }

   default List<Point3DReadOnly> getCopConstraintRegion()
   {
      return null;
   }
}
