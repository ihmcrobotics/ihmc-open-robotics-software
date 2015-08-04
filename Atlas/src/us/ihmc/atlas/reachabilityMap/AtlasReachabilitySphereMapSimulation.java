package us.ihmc.atlas.reachabilityMap;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.reachabilityMapCalculator.ReachabilityMapListener;
import us.ihmc.darpaRoboticsChallenge.reachabilityMapCalculator.ReachabilitySphereMapCalculator;
import us.ihmc.darpaRoboticsChallenge.wholeBodyInverseKinematicsSimulationController.JointAnglesWriter;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.robotics.FormattingTools;
import us.ihmc.robotics.humanoidRobot.partNames.LimbName;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class AtlasReachabilitySphereMapSimulation
{
   public AtlasReachabilitySphereMapSimulation()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, AtlasTarget.SIM, false);
      SDFFullRobotModel fullRobotModel = robotModel.createFullRobotModel();
      SDFRobot sdfRobot = robotModel.createSdfRobot(false);
      final JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(sdfRobot, fullRobotModel);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, 16000);
      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot, parameters);
      
      OneDoFJoint[] armJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(fullRobotModel.getChest(), fullRobotModel.getHand(RobotSide.LEFT)), OneDoFJoint.class);
      ReachabilitySphereMapCalculator reachabilitySphereMapCalculator = new ReachabilitySphereMapCalculator(armJoints, scs);
//      reachabilitySphereMapCalculator.setControlFrameFixedInEndEffector(fullRobotModel.getHandControlFrame(RobotSide.LEFT));
      FramePose palmCenter = new FramePose(fullRobotModel.getHandControlFrame(RobotSide.LEFT));
      palmCenter.setX(robotModel.getArmControllerParameters().getWristHandCenterOffset() - 0.02);
      palmCenter.changeFrame(fullRobotModel.getEndEffector(RobotSide.LEFT, LimbName.ARM).getBodyFixedFrame());
      RigidBodyTransform transformFromPalmCenterToHandBodyFixedFrame = new RigidBodyTransform();
      palmCenter.getPose(transformFromPalmCenterToHandBodyFixedFrame);
      reachabilitySphereMapCalculator.setTransformFromControlFrameToEndEffectorBodyFixedFrame(transformFromPalmCenterToHandBodyFixedFrame);
      String robotName = FormattingTools.underscoredToCamelCase(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ.toString(), true);
      reachabilitySphereMapCalculator.setupCalculatorToRecordInFile(robotName, getClass());

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
      new AtlasReachabilitySphereMapSimulation();
   }
}
