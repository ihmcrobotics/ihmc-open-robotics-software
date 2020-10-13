package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.RobotController;

public class FMSPeterPlanarWalkerController implements RobotController
{
   private YoRegistry registry = new YoRegistry("Controller");
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
   public YoRegistry getYoRegistry()
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
