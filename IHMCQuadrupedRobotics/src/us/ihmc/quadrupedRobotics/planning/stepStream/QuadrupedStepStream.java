package us.ihmc.quadrupedRobotics.planning.stepStream;

import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.util.PreallocatedQueue;
import us.ihmc.robotics.geometry.FrameOrientation;

public interface QuadrupedStepStream
{
   /**
    * Reinitialize the step stream.
    */
   void onEntry();

   /**
    * Update the step queue and body orientation at each time step.
    */
   void process();

   /**
    * Get the queue of ongoing and upcoming steps.
    * @return timed step queue
    */
   PreallocatedQueue<QuadrupedTimedStep> getSteps();

   /**
    * Get the nominal body orientation at the current time step.
    */
   void getBodyOrientation(FrameOrientation bodyOrientation);
}
