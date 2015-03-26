package us.ihmc.darpaRoboticsChallenge.robotController;

import us.ihmc.simulationconstructionset.robotController.AbstractThreadedRobotController;
import us.ihmc.simulationconstructionset.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationconstructionset.robotController.MultiThreadedRobotControllerExecutor;

public class DRCSimGazeboThreadedRobotController extends AbstractThreadedRobotController implements Runnable
{

   private volatile boolean running = true;
   
   public DRCSimGazeboThreadedRobotController()
   {
      super(DRCSimGazeboThreadedRobotController.class.getSimpleName());
   }

   @Override
   public void addController(MultiThreadedRobotControlElement controller, int executionsPerControlTick, boolean skipFirstControlCycle)
   {
      controllers.add(new MultiThreadedRobotControllerExecutor(null, controller, executionsPerControlTick, skipFirstControlCycle, registry));
   }

   @Override
   public void run()
   {
      while(running)
      {
         doControl();
      }
   }
   
   public void stop()
   {
      running = false;
   }

}
