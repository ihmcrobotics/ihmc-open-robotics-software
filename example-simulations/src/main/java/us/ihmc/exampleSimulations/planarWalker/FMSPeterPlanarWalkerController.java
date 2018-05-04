package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FMSPeterPlanarWalkerController implements RobotController
{
   private YoVariableRegistry registry = new YoVariableRegistry("Controller");
   SideDependentList<PeterPlanarWalkerStateMachine> walkerStateMachine;
   
   public FMSPeterPlanarWalkerController(PeterPlanarWalkerRobot robot, double deltaT)
   {
      walkerStateMachine = new SideDependentList<>();
            
      for(RobotSide robotSide : RobotSide.values)
      {
         walkerStateMachine.set(robotSide, new PeterPlanarWalkerStateMachine(robot, deltaT, robotSide, robot.getYoTime(), registry));
      }
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry; 
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
      for(RobotSide robotSide : RobotSide.values)
      {
         walkerStateMachine.get(robotSide).getStateMachine().doActionAndTransition();
      }
   }
}
