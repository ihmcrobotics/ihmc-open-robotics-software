package us.ihmc.robotics.robotController;

public class ModularRobotController extends AbstractModularRobotController implements RobotController
{
   public ModularRobotController(String name)
   {
      super(name);
   }

   @Override
   public void doControl()
   {
      if (rawSensorReader != null)
         rawSensorReader.read();
      if (sensorProcessor != null)
         sensorProcessor.update();
   
      for (int i = 0; i < robotControllers.size(); i++)
      {
         robotControllers.get(i).doControl();
      }
   
      if (outputProcessor != null)
         outputProcessor.update();
      if (rawOutputWriter != null)
         rawOutputWriter.write();
   }
}
