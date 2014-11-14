package us.ihmc.atlas.drcsim;

import us.ihmc.simulationconstructionset.robotController.AbstractThreadedRobotController;
import us.ihmc.simulationconstructionset.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationconstructionset.robotController.MultiThreadedRobotControllerExecutor;

public class DRCSimThreadedRobotController extends AbstractThreadedRobotController implements Runnable
{

   private volatile boolean running = true;
   
   public DRCSimThreadedRobotController()
   {
      super(DRCSimThreadedRobotController.class.getSimpleName());
   }

   @Override
   public void addController(MultiThreadedRobotControlElement controller, int executionsPerControlTick, boolean skipFirstControlCycle)
   {
      controllers.add(new MultiThreadedRobotControllerExecutor(controller, executionsPerControlTick, skipFirstControlCycle, registry));
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
