package us.ihmc.avatar.reachabilityMap.example;

import us.ihmc.avatar.reachabilityMap.ReachabilitySphereMapCalculator;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ReachabilitySphereMapExample
{
   public ReachabilitySphereMapExample()
   {
      final RobotArm robot = new RobotArm();
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, 16000);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      Graphics3DObject coordinate = new Graphics3DObject();
      coordinate.transform(robot.getElevatorFrameTransformToWorld());
      coordinate.addCoordinateSystem(1.0);
      scs.addStaticLinkGraphics(coordinate);
      scs.startOnAThread();

      OneDoFJointBasics[] armJoints = MultiBodySystemTools.filterJoints(robot.getJacobian().getJointsInOrder(), OneDoFJointBasics.class);
      ReachabilitySphereMapCalculator reachabilitySphereMapCalculator = new ReachabilitySphereMapCalculator(armJoints, scs);
//      reachabilitySphereMapCalculator.setControlFrameFixedInEndEffector(robot.getControlFrame());
      robot.setController(reachabilitySphereMapCalculator);
      robot.setController(new RobotController()
      {
         @Override
         public void initialize()
         {
         }

         @Override
         public YoRegistry getYoRegistry()
         {
            return new YoRegistry("dummy");
         }

         @Override
         public void doControl()
         {
            robot.copyRevoluteJointConfigurationToPinJoints();
         }
      });

      scs.setSimulateDoneCriterion(() -> reachabilitySphereMapCalculator.isDone());
      scs.simulate();
   }

   public static void main(String[] args)
   {
      new ReachabilitySphereMapExample();
   }
}
