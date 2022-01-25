package us.ihmc.exampleSimulations.lipmWalker;

public class LIPMWalkerSimulation
{
   public LIPMWalkerSimulation()
   {
      LIPMWalkerController controller = new LIPMWalkerController();
      LIPMWalkerRobot robot = new LIPMWalkerRobot();
   }
   
   public static void main(String[] args)
   {
      LIPMWalkerSimulation simulation = new LIPMWalkerSimulation();
   }
}
