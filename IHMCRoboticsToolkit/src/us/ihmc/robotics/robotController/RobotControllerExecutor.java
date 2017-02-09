package us.ihmc.robotics.robotController;

public interface RobotControllerExecutor
{

   void waitAndWriteData(long tick);

   void readData(long tick);

   public void executeForSimulationTick(long tick);

   void initialize();

   long getTicksPerSimulationTick();
   
   void stop();

   void updateDynamicGraphicObjectListRegistry();

}
