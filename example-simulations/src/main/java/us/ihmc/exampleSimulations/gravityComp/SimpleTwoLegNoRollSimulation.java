package us.ihmc.exampleSimulations.gravityComp;

import static us.ihmc.exampleSimulations.gravityComp.SimpleLegRobotDefinition.anklePitchJointName;
import static us.ihmc.exampleSimulations.gravityComp.SimpleLegRobotDefinition.hipPitchJointName;
import static us.ihmc.exampleSimulations.gravityComp.SimpleLegRobotDefinition.kneePitchJointName;
import static us.ihmc.exampleSimulations.gravityComp.SimpleLegRobotDefinition.rootJointName;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

public class SimpleTwoLegNoRollSimulation
{
   public static void main(String[] args)
   {
      double simulationDT = 1.0e-4;
      double controlDT = simulationDT;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      SimpleTwoLeggedNoRollRobotDefinition robotDefinition = new SimpleTwoLeggedNoRollRobotDefinition();
      RobotFromDescription scsRobot = new RobotFromDescription(robotDefinition);
      initializeRobot(scsRobot);
      scsRobot.setController(new SimpleTwoLegNoRollController(controlDT, new SimpleTwoLeggedNoRollRobot(robotDefinition, scsRobot), yoGraphicsListRegistry));

      LinearGroundContactModel groundContactModel = new LinearGroundContactModel(scsRobot, 1000.0, 200.0, 600.0, 250.0, scsRobot.getRobotsYoRegistry());
      scsRobot.setGroundContactModel(groundContactModel);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize((int) Math.pow(2, 16)); // => 65536
      SimulationConstructionSet scs = new SimulationConstructionSet(scsRobot, parameters);
      scs.setFastSimulate(true, 15);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);
      scs.setDT(simulationDT, 10);
      scs.startOnAThread();
   }

   private static void initializeRobot(Robot robot)
   {
      FloatingJoint rootJoint = (FloatingJoint) robot.getJoint(rootJointName);
      rootJoint.setPosition(0.0, 0.0, 0.9675);
      rootJoint.setYawPitchRoll(0, 0.15, 0);
      for (RobotSide robotSide : RobotSide.values)
      {
         OneDegreeOfFreedomJoint hipPitchJoint = (OneDegreeOfFreedomJoint) robot.getJoint(robotSide.getShortLowerCaseName() + "_" + hipPitchJointName);
         OneDegreeOfFreedomJoint kneePitchJoint = (OneDegreeOfFreedomJoint) robot.getJoint(robotSide.getShortLowerCaseName() + "_" + kneePitchJointName);
         OneDegreeOfFreedomJoint anklePitchJoint = (OneDegreeOfFreedomJoint) robot.getJoint(robotSide.getShortLowerCaseName() + "_" + anklePitchJointName);

         hipPitchJoint.setQ(-0.65);
         kneePitchJoint.setQ(1.0);
         anklePitchJoint.setQ(-0.5);
      }
   }
}
