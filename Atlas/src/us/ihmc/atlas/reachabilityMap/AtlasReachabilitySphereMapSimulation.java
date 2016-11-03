package us.ihmc.atlas.reachabilityMap;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.commonWalkingControlModules.reachabilityMapCalculator.ReachabilityMapListener;
import us.ihmc.commonWalkingControlModules.reachabilityMapCalculator.ReachabilitySphereMapCalculator;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.tools.FormattingTools;

public class AtlasReachabilitySphereMapSimulation
{
   public AtlasReachabilitySphereMapSimulation()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, DRCRobotModel.RobotTarget.SCS, false);
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      FloatingRootJointRobot sdfRobot = robotModel.createHumanoidFloatingRootJointRobot(false);
      final JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(sdfRobot, fullRobotModel);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, 16000);
      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot, parameters);
      
      OneDoFJoint[] armJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(fullRobotModel.getChest(), fullRobotModel.getHand(RobotSide.LEFT)), OneDoFJoint.class);
      ReachabilitySphereMapCalculator reachabilitySphereMapCalculator = new ReachabilitySphereMapCalculator(armJoints, scs);
//      reachabilitySphereMapCalculator.setControlFrameFixedInEndEffector(fullRobotModel.getHandControlFrame(RobotSide.LEFT));
      FramePose palmCenter = new FramePose(fullRobotModel.getHandControlFrame(RobotSide.LEFT));
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
