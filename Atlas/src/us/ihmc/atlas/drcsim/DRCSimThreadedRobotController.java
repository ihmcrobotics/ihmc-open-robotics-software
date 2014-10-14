package us.ihmc.atlas.drcsim;

import com.yobotics.simulationconstructionset.robotController.AbstractThreadedRobotController;
import com.yobotics.simulationconstructionset.robotController.MultiThreadedRobotControlElement;
import com.yobotics.simulationconstructionset.robotController.MultiThreadedRobotControllerExecutor;

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
