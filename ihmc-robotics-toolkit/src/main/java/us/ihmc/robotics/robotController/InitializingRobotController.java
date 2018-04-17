package us.ihmc.robotics.robotController;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

/**
 * Simple robot controller that calls RobotController.initialize() on the first tick
 * 
 * A new variable called "isInitialized" will be added to the controllers registry. 
 * 
 * If initialize() is called on this class, a RuntimeException will be thrown.
 * 
 * @author jesper
 *
 */
public class InitializingRobotController implements RobotController
{
   private final RobotController robotController;
   private final YoBoolean isInitialized;
   
   public InitializingRobotController(RobotController robotController)
   {
      this.robotController = robotController;
      this.isInitialized = new YoBoolean("isInitialized", robotController.getYoVariableRegistry());
      this.isInitialized.set(false);
   }

   @Override
   public void initialize()
   {
      throw new RuntimeException("Initialize() is called by the runtime already. It does not make sense to use this class.");
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return robotController.getYoVariableRegistry();
   }

   @Override
   public String getName()
   {
      return robotController.getName();
   }

   @Override
   public String getDescription()
   {
      return robotController.getDescription();
   }

   @Override
   public void doControl()
   {
      if(!this.isInitialized.getValue())
      {
         robotController.initialize();
         this.isInitialized.set(true);
      }
      
      robotController.doControl();
   }
   

}
