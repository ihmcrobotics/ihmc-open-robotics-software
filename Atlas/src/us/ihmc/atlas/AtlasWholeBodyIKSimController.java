package us.ihmc.atlas;

import java.util.ArrayList;

import us.ihmc.SdfLoader.SDFBaseFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.wholeBodyInverseKinematicsSimulationController.WholeBodyIKSimController;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.humanoidRobotics.partNames.ArmJointName;
import us.ihmc.humanoidRobotics.partNames.LegJointName;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

public class AtlasWholeBodyIKSimController extends WholeBodyIKSimController
{

   protected final ArrayList<ImmutablePair<RobotSide, RigidBodyTransform>> queuedTargets =  new ArrayList<ImmutablePair<RobotSide, RigidBodyTransform>>();

   public AtlasWholeBodyIKSimController(SDFRobot robot, SDFFullHumanoidRobotModel fullRobotModel, DRCRobotModel atlasRobotModel) throws Exception
   {
      super(robot, fullRobotModel, atlasRobotModel);
   }

   @Override
   public void setInitialJointAngles(SDFRobot scsRobot, SDFBaseFullRobotModel fullRobotModel, DRCRobotModel atlasRobotModel)
   {
      // Avoid singularities at startup
      for (RobotSide robotSide : RobotSide.values)
      {
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(0.0); //leg_hpz
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_ROLL)).setQ(robotSide.negateIfRightSide(0.062)); //leg_hpx
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(-0.233); //leg_hpy
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.KNEE)).setQ(0.518); //leg_kny
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(-0.276); //leg_aky
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.ANKLE_ROLL)).setQ(robotSide.negateIfRightSide(-0.062)); //leg_akx
         
         fullRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(0.0); //leg_hpz
         fullRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_ROLL)).setQ(robotSide.negateIfRightSide(0.062)); //leg_hpx
         fullRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(-0.233); //leg_hpy
         fullRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.KNEE)).setQ(0.518); //leg_kny
         fullRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(-0.276); //leg_aky
         fullRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.ANKLE_ROLL)).setQ(robotSide.negateIfRightSide(-0.062)); //leg_akx
         
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_YAW)).setQ(0.300); //arm_shy
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)).setQ(robotSide.negateIfRightSide(-1.30)); //arm_shx
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(2.00); //arm_ely
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_ROLL)).setQ(robotSide.negateIfRightSide(0.498)); //arm_elx
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH)).setQ(0.000); //arm_wry
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.WRIST_ROLL)).setQ(robotSide.negateIfRightSide(-0.004)); //arm_wrx
         
         fullRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_YAW)).setQ(0.300); //arm_shy
         fullRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)).setQ(robotSide.negateIfRightSide(-1.30)); //arm_shx
         fullRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(2.00); //arm_ely
         fullRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_ROLL)).setQ(robotSide.negateIfRightSide(0.498)); //arm_elx
         fullRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH)).setQ(0.000); //arm_wry
         fullRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.WRIST_ROLL)).setQ(robotSide.negateIfRightSide(-0.004)); //arm_wrx
      }
   }



   @Override
   public ArrayList<ImmutablePair<RobotSide, RigidBodyTransform>> getTargetsQueue()
   {
      //Add RigidBodyTransforms that are from pelvis frame to desired end effector frame here.
      queuedTargets.add(new ImmutablePair<RobotSide, RigidBodyTransform>(RobotSide.RIGHT, new RigidBodyTransform()));
      return queuedTargets;
   }



}
