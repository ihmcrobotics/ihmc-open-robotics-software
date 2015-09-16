package us.ihmc.darpaRoboticsChallenge.reachabilityMapCalculator.example;

import us.ihmc.darpaRoboticsChallenge.reachabilityMapCalculator.ReachabilityMapListener;
import us.ihmc.darpaRoboticsChallenge.reachabilityMapCalculator.ReachabilitySphereMapCalculator;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

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

      OneDoFJoint[] armJoints = ScrewTools.filterJoints(robot.getJacobian().getJointsInOrder(), OneDoFJoint.class);
      ReachabilitySphereMapCalculator reachabilitySphereMapCalculator = new ReachabilitySphereMapCalculator(armJoints, scs);
      reachabilitySphereMapCalculator.setControlFrameFixedInEndEffector(robot.getControlFrame());
      ReachabilityMapListener listener = new ReachabilityMapListener()
      {
         @Override
         public void hasReachedNewConfiguration()
         {
            robot.copyRevoluteJointConfigurationToPinJoints();
         }
      };
      reachabilitySphereMapCalculator.attachReachabilityMapListener(listener);
      reachabilitySphereMapCalculator.buildReachabilitySpace();
   }

   public static void main(String[] args)
   {
      new ReachabilitySphereMapExample();
   }
}
