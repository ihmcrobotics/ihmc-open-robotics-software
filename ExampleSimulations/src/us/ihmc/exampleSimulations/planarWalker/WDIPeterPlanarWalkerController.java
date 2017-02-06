package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class WDIPeterPlanarWalkerController implements RobotController
{
   private YoVariableRegistry registry = new YoVariableRegistry("Controller");
   private double deltaT;
   SideDependentList<PeterPlanarWalkerStateMachine> walkerStateMachine;
   
   public WDIPeterPlanarWalkerController(PeterPlanarWalkerRobot robot, double deltaT)
   {
      this.deltaT = deltaT;
      walkerStateMachine = new SideDependentList<>();
            
      for(RobotSide robotSide : RobotSide.values)
      {
         walkerStateMachine.set(robotSide, new PeterPlanarWalkerStateMachine(robot, deltaT, robotSide, robot.getYoTime()));
      }
   }

   @Override
   public void initialize()
   {
      // TODO Auto-generated method stub
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return null; 
   }

   @Override
   public String getName()
   {
      return null;
   }

   @Override
   public String getDescription()
   {
      return null;
   }

   @Override
   public void doControl()
   {
   }
}
