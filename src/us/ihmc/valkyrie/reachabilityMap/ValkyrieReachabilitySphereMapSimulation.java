package us.ihmc.valkyrie.reachabilityMap;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.reachabilityMapCalculator.ReachabilityMapListener;
import us.ihmc.darpaRoboticsChallenge.reachabilityMapCalculator.ReachabilitySphereMapCalculator;
import us.ihmc.darpaRoboticsChallenge.wholeBodyInverseKinematicsSimulationController.JointAnglesWriter;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieReachabilitySphereMapSimulation
{
   public ValkyrieReachabilitySphereMapSimulation()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(false, false);
      SDFFullRobotModel fullRobotModel = robotModel.createFullRobotModel();
      SDFRobot sdfRobot = robotModel.createSdfRobot(false);
      final JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(sdfRobot, fullRobotModel);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, 16000);
      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot, parameters);

      OneDoFJoint[] armJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(fullRobotModel.getChest(), fullRobotModel.getHand(RobotSide.LEFT)),
            OneDoFJoint.class);
      ReachabilitySphereMapCalculator reachabilitySphereMapCalculator = new ReachabilitySphereMapCalculator(armJoints, scs);
      reachabilitySphereMapCalculator.setupCalculatorToRecordInFile("Valkyrie", getClass());

      ReachabilityMapListener listener = new ReachabilityMapListener()
      {
         @Override
         public void hasReachedNewConfiguration()
         {
            jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
         }
      };

      reachabilitySphereMapCalculator.attachReachabilityMapListener(listener);
      scs.startOnAThread();

      reachabilitySphereMapCalculator.buildReachabilitySpace();
   }

   public static void main(String[] args)
   {
      new ValkyrieReachabilitySphereMapSimulation();
   }
}
