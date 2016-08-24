package us.ihmc.quadrupedRobotics.planning.stepInput;

import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.util.PreallocatedQueue;
import us.ihmc.robotics.geometry.FrameOrientation;

public interface QuadrupedStepInputPlanner
{
   void initialize();
   void compute();

   /**
    * @return the current reference orientation for the body link
    */
   void getBodyOrientation(FrameOrientation bodyOrientation);

   /**
    * @return the queue of upcoming steps
    */
   PreallocatedQueue<QuadrupedTimedStep> getStepQueue();
}
