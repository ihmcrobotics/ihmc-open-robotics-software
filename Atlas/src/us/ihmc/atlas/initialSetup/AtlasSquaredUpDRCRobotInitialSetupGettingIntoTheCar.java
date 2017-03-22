package us.ihmc.atlas.initialSetup;

import java.util.LinkedHashMap;
import java.util.Map;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

// TODO: Temporary initial setup for testing the grasping control
public class AtlasSquaredUpDRCRobotInitialSetupGettingIntoTheCar implements DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>
{
   private enum Methods
   {
      FACING_LHAND_GRABBING, FACING_BOTH_HANDS_GRABBING, FACING_LHAND_GRABBING_RHAND_SEAT,
      SIDE_RHAND_GRABBING_FRONT_BAR, SIDE_BOTH_HANDS_GRABBING_FRONT_AND_ABOVE_HEAD_BARS, SIDE_RHAND_GRABBING_TOP_BAR, SIDE_BOTH_HANDS_GRABBING_TOP_AND_FRONT_BARS, SIDE_BOTH_HANDS_GRABBING_FRONT_BAR_AND_STEERING_WHEEL,
      SIDE_LHAND_GRABBING_FRONT_BAR_RHAND_ON_SEAT,
      BACK_BOTH_HANDS_GRABBING
   };

   private static final Methods SELECTED_METHOD = Methods.SIDE_BOTH_HANDS_GRABBING_TOP_AND_FRONT_BARS;

   private double groundZ;

   public AtlasSquaredUpDRCRobotInitialSetupGettingIntoTheCar()
   {
      this(0.0);
   }

   public AtlasSquaredUpDRCRobotInitialSetupGettingIntoTheCar(double groundZ)
   {
      this.groundZ = groundZ;
   }

   @Override
   public void initializeRobot(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String prefix = StringUtils.uncapitalize(robotSide.getSideNameFirstLetter());
         robot.getOneDegreeOfFreedomJoint(prefix + "_leg_hpy").setQ(legInitialJointPositions.get(LegJointName.HIP_PITCH));
         robot.getOneDegreeOfFreedomJoint(prefix + "_leg_kny").setQ(legInitialJointPositions.get(LegJointName.KNEE_PITCH));
         robot.getOneDegreeOfFreedomJoint(prefix + "_leg_aky").setQ(legInitialJointPositions.get(LegJointName.ANKLE_PITCH));

         robot.getOneDegreeOfFreedomJoint(prefix + "_arm_wry").setQ(armInitialJointPositions.get(robotSide).get(ArmJointName.FIRST_WRIST_PITCH));
         robot.getOneDegreeOfFreedomJoint(prefix + "_arm_wrx").setQ(armInitialJointPositions.get(robotSide).get(ArmJointName.WRIST_ROLL));
         robot.getOneDegreeOfFreedomJoint(prefix + "_arm_shz").setQ(armInitialJointPositions.get(robotSide).get(ArmJointName.SHOULDER_YAW));
         robot.getOneDegreeOfFreedomJoint(prefix + "_arm_shx").setQ(armInitialJointPositions.get(robotSide).get(ArmJointName.SHOULDER_ROLL));
         robot.getOneDegreeOfFreedomJoint(prefix + "_arm_ely").setQ(armInitialJointPositions.get(robotSide).get(ArmJointName.ELBOW_PITCH));
         robot.getOneDegreeOfFreedomJoint(prefix + "_arm_elx").setQ(armInitialJointPositions.get(robotSide).get(ArmJointName.ELBOW_ROLL));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         String prefix = robotSide.getCamelCaseNameForStartOfExpression();
         robot.getOneDegreeOfFreedomJoint(prefix + "_f0_j1").setQ(0.985);//1.46);
         robot.getOneDegreeOfFreedomJoint(prefix + "_f1_j1").setQ(1.066);//1.46);
         robot.getOneDegreeOfFreedomJoint(prefix + "_f2_j1").setQ(1.163);//1.46);
         robot.getOneDegreeOfFreedomJoint(prefix + "_f3_j1").setQ(0.660);

         robot.getOneDegreeOfFreedomJoint(prefix + "_f0_j2").setQ(1.002);//0.4);
         robot.getOneDegreeOfFreedomJoint(prefix + "_f1_j2").setQ(0.951);//0.4);
         robot.getOneDegreeOfFreedomJoint(prefix + "_f2_j2").setQ(0.883);//0.4);
         robot.getOneDegreeOfFreedomJoint(prefix + "_f3_j2").setQ(0.837);
      }

      switch(SELECTED_METHOD)
      {
      case FACING_LHAND_GRABBING_RHAND_SEAT:
      case SIDE_LHAND_GRABBING_FRONT_BAR_RHAND_ON_SEAT:
         robot.getOneDegreeOfFreedomJoint("right_f0_j1").setQ(-1.4672);
         robot.getOneDegreeOfFreedomJoint("right_f1_j1").setQ(-1.4672);
         robot.getOneDegreeOfFreedomJoint("right_f2_j1").setQ(-1.4672);
         robot.getOneDegreeOfFreedomJoint("right_f3_j1").setQ(0.0);

         robot.getOneDegreeOfFreedomJoint("right_f0_j2").setQ(-1.4706);
         robot.getOneDegreeOfFreedomJoint("right_f1_j2").setQ(-1.4706);
         robot.getOneDegreeOfFreedomJoint("right_f2_j2").setQ(-1.4706);
         robot.getOneDegreeOfFreedomJoint("right_f3_j2").setQ(0.0);
         break;

      default:
         break;
      }

      robot.update();

      robot.setPositionInWorld(new Vector3D(pelvisInitialPose.getX(), pelvisInitialPose.getY(), groundZ + pelvisInitialPose.getZ()));
      robot.setOrientation(pelvisInitialPose.getYaw(), pelvisInitialPose.getPitch(), pelvisInitialPose.getRoll());
   }

   private final static FramePose pelvisInitialPose = new FramePose(ReferenceFrame.getWorldFrame());
   private final static Map<LegJointName, Double> legInitialJointPositions = new LinkedHashMap<LegJointName, Double>();
   private final static SideDependentList<Map<ArmJointName, Double>> armInitialJointPositions = SideDependentList.createListOfHashMaps();

   static
   {
      FramePoint pelvisPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      FrameOrientation pelvisOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame());

      Map<ArmJointName, Double> leftArmInitialJointPositions = new LinkedHashMap<ArmJointName, Double>();
      Map<ArmJointName, Double> rightArmInitialJointPositions = new LinkedHashMap<ArmJointName, Double>();

      switch (SELECTED_METHOD)
      {
      case FACING_LHAND_GRABBING:
         pelvisPosition.set(-0.016, 0.21, 0.8413);
         pelvisOrientation.setYawPitchRoll(0, 0, 0);

         legInitialJointPositions.put(LegJointName.HIP_PITCH, -0.5958);
         legInitialJointPositions.put(LegJointName.KNEE_PITCH, 1.0931);
         legInitialJointPositions.put(LegJointName.ANKLE_PITCH, -0.4973);

         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_YAW, 0.0957);
         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, -0.5149);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 1.5455);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, 1.8304);
         leftArmInitialJointPositions.put(ArmJointName.FIRST_WRIST_PITCH, -0.58);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.55);

         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_YAW, 0.34);
         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, 1.36);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 1.94);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, -1.18);
         rightArmInitialJointPositions.put(ArmJointName.FIRST_WRIST_PITCH, -0.19);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.07);
         break;

      case FACING_LHAND_GRABBING_RHAND_SEAT:
         pelvisPosition.set(-0.016, 0.21, 0.8413);
         pelvisOrientation.setYawPitchRoll(0, 0, 0);

         legInitialJointPositions.put(LegJointName.HIP_PITCH, -0.5958);
         legInitialJointPositions.put(LegJointName.KNEE_PITCH, 1.0931);
         legInitialJointPositions.put(LegJointName.ANKLE_PITCH, -0.4973);

         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_YAW, 0.0957);
         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, -0.5149);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 1.5455);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, 1.8304);
         leftArmInitialJointPositions.put(ArmJointName.FIRST_WRIST_PITCH, -0.58);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.55);

         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_YAW, -0.7885);
         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, 1.028);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 1.979);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, -0.872);
         rightArmInitialJointPositions.put(ArmJointName.FIRST_WRIST_PITCH, 0.532);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.07);
         break;

      case FACING_BOTH_HANDS_GRABBING:
         pelvisPosition.set(-0.1, 0.2548, 0.7891);
         pelvisOrientation.setYawPitchRoll(0, 0, 0);

         legInitialJointPositions.put(LegJointName.HIP_PITCH, -0.84);
         legInitialJointPositions.put(LegJointName.KNEE_PITCH, 1.34);
         legInitialJointPositions.put(LegJointName.ANKLE_PITCH, -0.5);

         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_YAW, -0.5);
         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, -1.3);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.955);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, 2.075);
         leftArmInitialJointPositions.put(ArmJointName.FIRST_WRIST_PITCH, 0.5);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.0);

         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_YAW, -0.8634);
         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, 1.1138);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.09);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, -1.75);
         rightArmInitialJointPositions.put(ArmJointName.FIRST_WRIST_PITCH, 0.35);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.2);
         break;

      case SIDE_RHAND_GRABBING_FRONT_BAR:
         pelvisPosition.set(0.13, 0.21, 0.8326);
         pelvisOrientation.setYawPitchRoll(Math.PI / 2.0, 0, 0);

         legInitialJointPositions.put(LegJointName.HIP_PITCH, -0.6521);
         legInitialJointPositions.put(LegJointName.KNEE_PITCH, 1.1418);
         legInitialJointPositions.put(LegJointName.ANKLE_PITCH, -0.49);

         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_YAW, 0.3431);
         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, -1.3606);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 1.9403);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, 1.1807);
         leftArmInitialJointPositions.put(ArmJointName.FIRST_WRIST_PITCH, -0.1928);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, -0.0703);

         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_YAW, -0.6352);
         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, 0.7261);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.6198);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, -2.3613);
         rightArmInitialJointPositions.put(ArmJointName.FIRST_WRIST_PITCH, -1.1842);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.4331);
         break;

      case SIDE_RHAND_GRABBING_TOP_BAR:
         pelvisPosition.set(0.1, 0.15, 0.8326);
         pelvisOrientation.setYawPitchRoll(Math.PI / 2.0, 0, 0);

         legInitialJointPositions.put(LegJointName.HIP_PITCH, -0.6521);
         legInitialJointPositions.put(LegJointName.KNEE_PITCH, 1.1418);
         legInitialJointPositions.put(LegJointName.ANKLE_PITCH, -0.49);

         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_YAW, 0.34);
         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, -1.3599);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 1.9402);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, 1.1802);
         leftArmInitialJointPositions.put(ArmJointName.FIRST_WRIST_PITCH, -0.193);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, -0.0705);

         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_YAW, 0.0);
         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, -0.78);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.7);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, -0.3731);
         rightArmInitialJointPositions.put(ArmJointName.FIRST_WRIST_PITCH, -0.53);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, -1.15);
         break;

      case SIDE_BOTH_HANDS_GRABBING_TOP_AND_FRONT_BARS:
         pelvisPosition.set(0.1, 0.15, 0.8326);
         pelvisOrientation.setYawPitchRoll(Math.PI / 2.0, 0, 0);

         legInitialJointPositions.put(LegJointName.HIP_PITCH, -0.6521);
         legInitialJointPositions.put(LegJointName.KNEE_PITCH, 1.1418);
         legInitialJointPositions.put(LegJointName.ANKLE_PITCH, -0.49);

         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_YAW, -1.7913);
         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, -1.3976);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.9252);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, 0.8858);
         leftArmInitialJointPositions.put(ArmJointName.FIRST_WRIST_PITCH, 0.1378);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.4528);

         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_YAW, 0.0);
         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, -0.78);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.7);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, -0.3731);
         rightArmInitialJointPositions.put(ArmJointName.FIRST_WRIST_PITCH, -0.53);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, -1.15);
         break;

      case SIDE_BOTH_HANDS_GRABBING_FRONT_BAR_AND_STEERING_WHEEL:
         pelvisPosition.set(0.1, 0.15, 0.8326);
         pelvisOrientation.setYawPitchRoll(Math.PI / 2.0, 0, 0);

         legInitialJointPositions.put(LegJointName.HIP_PITCH, -0.6521);
         legInitialJointPositions.put(LegJointName.KNEE_PITCH, 1.1418);
         legInitialJointPositions.put(LegJointName.ANKLE_PITCH, -0.49);

         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_YAW, -1.7913);
         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, -1.3976);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.9252);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, 0.8858);
         leftArmInitialJointPositions.put(ArmJointName.FIRST_WRIST_PITCH, 0.1378);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.4528);

         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_YAW, 0.2559);
         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, 0.4528);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.7677);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, -1.6732);
         rightArmInitialJointPositions.put(ArmJointName.FIRST_WRIST_PITCH, 0.2165);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, -1.0039);
         break;

      case SIDE_BOTH_HANDS_GRABBING_FRONT_AND_ABOVE_HEAD_BARS:
         pelvisPosition.set(0.1, 0.15, 0.8326);
         pelvisOrientation.setYawPitchRoll(Math.PI / 2.0, 0, 0);

         legInitialJointPositions.put(LegJointName.HIP_PITCH, -0.6521);
         legInitialJointPositions.put(LegJointName.KNEE_PITCH, 1.1418);
         legInitialJointPositions.put(LegJointName.ANKLE_PITCH, -0.49);

         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_YAW, -1.7913);
         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, -1.3976);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.9252);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, 0.8858);
         leftArmInitialJointPositions.put(ArmJointName.FIRST_WRIST_PITCH, 0.1378);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.4528);

         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_YAW, -0.9122);
         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, -0.6542);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.5442);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, -1.206);
         rightArmInitialJointPositions.put(ArmJointName.FIRST_WRIST_PITCH, -1.1752);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, -0.4202);
         //         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_YAW, -0.1772);
//         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, -0.3346);
//         rightArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 1.2);
//         rightArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, -2.03);
//         rightArmInitialJointPositions.put(ArmJointName.FIRST_WRIST_PITCH, -2.03);
//         rightArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, -0.69);
         break;

      case BACK_BOTH_HANDS_GRABBING:
         pelvisPosition.set(0.1, 0.15, 0.8326);
         pelvisOrientation.setYawPitchRoll(Math.PI, 0, 0);

         legInitialJointPositions.put(LegJointName.HIP_PITCH, -0.6521);
         legInitialJointPositions.put(LegJointName.KNEE_PITCH, 1.1418);
         legInitialJointPositions.put(LegJointName.ANKLE_PITCH, -0.49);

         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_YAW, 0.0);
         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, 0.0);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.0);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, 0.0);
         leftArmInitialJointPositions.put(ArmJointName.FIRST_WRIST_PITCH, 0.0);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.0);

         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_YAW, 0.0);
         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, 0.0);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.0);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, 0.0);
         rightArmInitialJointPositions.put(ArmJointName.FIRST_WRIST_PITCH, 0.0);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.0);
         break;

      case SIDE_LHAND_GRABBING_FRONT_BAR_RHAND_ON_SEAT:
         pelvisPosition.set(0.1, 0.15, 0.8326);
         pelvisOrientation.setYawPitchRoll(Math.PI / 2.0, 0, 0);

         legInitialJointPositions.put(LegJointName.HIP_PITCH, -0.6521);
         legInitialJointPositions.put(LegJointName.KNEE_PITCH, 1.1418);
         legInitialJointPositions.put(LegJointName.ANKLE_PITCH, -0.49);

         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_YAW, -1.7913);
         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, -1.3976);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.9252);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, 0.8858);
         leftArmInitialJointPositions.put(ArmJointName.FIRST_WRIST_PITCH, 0.1378);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.4528);

         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_YAW, 0.48);
         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, 0.76);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 1.29);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, -0.48);
         rightArmInitialJointPositions.put(ArmJointName.FIRST_WRIST_PITCH, 0.33);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, -0.34);
         break;

      default:
         throw new RuntimeException("Not implemented yet!");
      }

      pelvisInitialPose.setPosition(pelvisPosition);
      pelvisInitialPose.setOrientation(pelvisOrientation);

      armInitialJointPositions.put(RobotSide.LEFT, leftArmInitialJointPositions);
      armInitialJointPositions.put(RobotSide.RIGHT, rightArmInitialJointPositions);
   }

   @Override
   public void setOffset(Vector3D offset)
   {
   }

   @Override
   public void setInitialYaw(double yaw)
   {
   }

   @Override
   public void setInitialGroundHeight(double groundHeight)
   {
      groundZ = groundHeight;
   }

   @Override
   public double getInitialYaw()
   {
      return 0;
   }

   @Override
   public double getInitialGroundHeight()
   {
      return groundZ;
   }

   @Override
   public void getOffset(Vector3D offsetToPack)
   {

   }
}
