package us.ihmc.simulationconstructionset.robotController;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;


public interface MultiThreadedRobotControlElement extends Runnable
{
   /**
    * Initialize this controller. Called from the controller thread.
    */
   public void initialize();
   
   /**
    * Read data from the robot. This method is called from the simulation thread.
    * @param currentClockTime Current clock time in nanoseconds, could be used for thread synchronization in a realtime context
    */
   public void read(long currentClockTime);
   
   /**
    * Do control. This method is called from the controller thread.
    */
   @Override
   public void run();
   
   /**
    * Write data to robot. This method is called from the simulation thread.
    * @param timestamp TODO
    */
   public void write(long timestamp);
   
   /**
    * Get the controllers YoVariableRegistry.
    */
   public YoVariableRegistry getYoVariableRegistry();

   /**
    * @return Name of the controller, used to name the control thread.
    */
   public String getName();
   
   /**
    * Get the DynamicGraphicsObjectListRegistry for this controller and adds it to SCS
    */
   public YoGraphicsListRegistry getDynamicGraphicObjectsListRegistry();
   
   /**
    * Get the clock time when this controller should wake up next. This is used in a realtime context where locks are infeasible.
    * @return Next wakeup time based of clock time in nanoseconds.
    */
   public long nextWakeupTime();
   
   
}
