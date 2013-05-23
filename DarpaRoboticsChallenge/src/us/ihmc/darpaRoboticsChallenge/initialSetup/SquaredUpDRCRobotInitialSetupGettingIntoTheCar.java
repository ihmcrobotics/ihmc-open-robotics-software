package us.ihmc.darpaRoboticsChallenge.initialSetup;

import java.util.LinkedHashMap;
import java.util.Map;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.darpaRoboticsChallenge.DRCRobotMultiContactControllerParameters;
import us.ihmc.projectM.R2Sim02.initialSetup.RobotInitialSetup;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

// TODO: Temporary initial setup for testing the grasping control
public class SquaredUpDRCRobotInitialSetupGettingIntoTheCar implements RobotInitialSetup<SDFRobot>
{
   private enum Methods
   {
      FACING_LHAND_GRABBING, FACING_BOTH_HANDS_GRABBING,
      SIDE_RHAND_GRABBING_FRONT_BAR, SIDE_RHAND_GRABBING_TOP_BAR, SIDE_BOTH_HANDS_GRABBING_TOP_AND_FRONT_BARS, SIDE_BOTH_HANDS_GRABBING_FRONT_BAR_AND_WHEEL,
      BACK_BOTH_HANDS_GRABBING
   };

   private static final Methods SELECTED_METHOD = Methods.SIDE_BOTH_HANDS_GRABBING_FRONT_BAR_AND_WHEEL;

   private final double groundZ;

   public SquaredUpDRCRobotInitialSetupGettingIntoTheCar()
   {
      this(0.0);
   }

   public SquaredUpDRCRobotInitialSetupGettingIntoTheCar(double groundZ)
   {
      this.groundZ = groundZ;
   }

   public void initializeRobot(SDFRobot robot)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String prefix = FormattingTools.lowerCaseFirstLetter(robotSide.getSideNameFirstLetter());
         robot.getOneDoFJoint(prefix + "_leg_lhy").setQ(legInitialJointPositions.get(LegJointName.HIP_PITCH));
         robot.getOneDoFJoint(prefix + "_leg_kny").setQ(legInitialJointPositions.get(LegJointName.KNEE));
         robot.getOneDoFJoint(prefix + "_leg_uay").setQ(legInitialJointPositions.get(LegJointName.ANKLE_PITCH));

         robot.getOneDoFJoint(prefix + "_arm_uwy").setQ(armInitialJointPositions.get(robotSide).get(ArmJointName.WRIST_PITCH));
         robot.getOneDoFJoint(prefix + "_arm_mwx").setQ(armInitialJointPositions.get(robotSide).get(ArmJointName.WRIST_ROLL));
         robot.getOneDoFJoint(prefix + "_arm_usy").setQ(armInitialJointPositions.get(robotSide).get(ArmJointName.SHOULDER_PITCH));
         robot.getOneDoFJoint(prefix + "_arm_shx").setQ(armInitialJointPositions.get(robotSide).get(ArmJointName.SHOULDER_ROLL));
         robot.getOneDoFJoint(prefix + "_arm_ely").setQ(armInitialJointPositions.get(robotSide).get(ArmJointName.ELBOW_PITCH));
         robot.getOneDoFJoint(prefix + "_arm_elx").setQ(armInitialJointPositions.get(robotSide).get(ArmJointName.ELBOW_ROLL));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         String prefix = robotSide.getCamelCaseNameForStartOfExpression();
         robot.getOneDoFJoint(prefix + "_f0_j1").setQ(0.985);//1.46);
         robot.getOneDoFJoint(prefix + "_f1_j1").setQ(1.066);//1.46);
         robot.getOneDoFJoint(prefix + "_f2_j1").setQ(1.163);//1.46);
         robot.getOneDoFJoint(prefix + "_f3_j1").setQ(0.660);

         robot.getOneDoFJoint(prefix + "_f0_j2").setQ(1.002);//0.4);
         robot.getOneDoFJoint(prefix + "_f1_j2").setQ(0.951);//0.4);
         robot.getOneDoFJoint(prefix + "_f2_j2").setQ(0.883);//0.4);
         robot.getOneDoFJoint(prefix + "_f3_j2").setQ(0.837);
      }
      robot.update();
      
      robot.setPositionInWorld(new Vector3d(pelvisInitialPose.getX(), pelvisInitialPose.getY(), groundZ + pelvisInitialPose.getZ()));
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
         legInitialJointPositions.put(LegJointName.KNEE, 1.0931);
         legInitialJointPositions.put(LegJointName.ANKLE_PITCH, -0.4973);

         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_PITCH, 0.0957);
         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, -0.5149);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 1.5455);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, 1.8304);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_PITCH, -0.58);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.55);

         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_PITCH, 0.34);
         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, 1.36);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 1.94);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, -1.18);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_PITCH, -0.19);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.07);
         break;

      case FACING_BOTH_HANDS_GRABBING:
         pelvisPosition.set(-0.1, 0.2548, 0.7891);
         pelvisOrientation.setYawPitchRoll(0, 0, 0);
         
         legInitialJointPositions.put(LegJointName.HIP_PITCH, -0.84);
         legInitialJointPositions.put(LegJointName.KNEE, 1.34);
         legInitialJointPositions.put(LegJointName.ANKLE_PITCH, -0.5);

         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_PITCH, -0.5);
         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, -1.3);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.955);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, 2.075);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_PITCH, 0.5);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.0);

         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_PITCH, -0.8634);
         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, 1.1138);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.09);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, -1.75);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_PITCH, 0.35);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.2);
         break;

      case SIDE_RHAND_GRABBING_FRONT_BAR:
         pelvisPosition.set(-0.554, 0.0, 0.8326);
         pelvisOrientation.setYawPitchRoll(Math.PI / 2.0, 0, 0);
         
         legInitialJointPositions.put(LegJointName.HIP_PITCH, -0.6521);
         legInitialJointPositions.put(LegJointName.KNEE, 1.1418);
         legInitialJointPositions.put(LegJointName.ANKLE_PITCH, -0.49);

         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_PITCH, 0.3431);
         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, -1.3606);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 1.9403);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, 1.1807);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_PITCH, -0.1928);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, -0.0703);

         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_PITCH, -0.6352);
         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, 0.7261);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.6198);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, -2.3613);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_PITCH, -1.1842);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.4331);
         break;

      case SIDE_RHAND_GRABBING_TOP_BAR:
         pelvisPosition.set(0.1, 0.15, 0.8326);
         pelvisOrientation.setYawPitchRoll(Math.PI / 2.0, 0, 0);

         legInitialJointPositions.put(LegJointName.HIP_PITCH, -0.6521);
         legInitialJointPositions.put(LegJointName.KNEE, 1.1418);
         legInitialJointPositions.put(LegJointName.ANKLE_PITCH, -0.49);

         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_PITCH, 0.34);
         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, -1.3599);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 1.9402);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, 1.1802);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_PITCH, -0.193);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, -0.0705);

         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_PITCH, 0.0);
         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, -0.78);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.7);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, -0.3731);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_PITCH, -0.53);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, -1.15);
         break;

      case SIDE_BOTH_HANDS_GRABBING_TOP_AND_FRONT_BARS:
         pelvisPosition.set(0.1, 0.15, 0.8326);
         pelvisOrientation.setYawPitchRoll(Math.PI / 2.0, 0, 0);

         legInitialJointPositions.put(LegJointName.HIP_PITCH, -0.6521);
         legInitialJointPositions.put(LegJointName.KNEE, 1.1418);
         legInitialJointPositions.put(LegJointName.ANKLE_PITCH, -0.49);

         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_PITCH, -1.7913);
         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, -1.3976);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.9252);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, 0.8858);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_PITCH, 0.1378);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.4528);

         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_PITCH, 0.0);
         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, -0.78);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.7);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, -0.3731);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_PITCH, -0.53);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, -1.15);
         break;

      case SIDE_BOTH_HANDS_GRABBING_FRONT_BAR_AND_WHEEL:
         pelvisPosition.set(0.1, 0.15, 0.8326);
         pelvisOrientation.setYawPitchRoll(Math.PI / 2.0, 0, 0);

         legInitialJointPositions.put(LegJointName.HIP_PITCH, -0.6521);
         legInitialJointPositions.put(LegJointName.KNEE, 1.1418);
         legInitialJointPositions.put(LegJointName.ANKLE_PITCH, -0.49);

         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_PITCH, -1.7913);
         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, -1.3976);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.9252);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, 0.8858);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_PITCH, 0.1378);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.4528);

         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_PITCH, 0.2559);
         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, 0.4528);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.7677);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, -1.6732);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_PITCH, 0.2165);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, -1.0039);
         break;

      case BACK_BOTH_HANDS_GRABBING:
         pelvisPosition.set(0.1, 0.15, 0.8326);
         pelvisOrientation.setYawPitchRoll(Math.PI, 0, 0);

         legInitialJointPositions.put(LegJointName.HIP_PITCH, -0.6521);
         legInitialJointPositions.put(LegJointName.KNEE, 1.1418);
         legInitialJointPositions.put(LegJointName.ANKLE_PITCH, -0.49);

         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_PITCH, 0.0);
         leftArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, 0.0);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.0);
         leftArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, 0.0);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_PITCH, 0.0);
         leftArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.0);

         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_PITCH, 0.0);
         rightArmInitialJointPositions.put(ArmJointName.SHOULDER_ROLL, 0.0);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_PITCH, 0.0);
         rightArmInitialJointPositions.put(ArmJointName.ELBOW_ROLL, 0.0);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_PITCH, 0.0);
         rightArmInitialJointPositions.put(ArmJointName.WRIST_ROLL, 0.0);
         break;

      default:
         throw new RuntimeException("Not implemented yet!");
      }
      
      pelvisInitialPose.setPosition(pelvisPosition);
      pelvisInitialPose.setOrientation(pelvisOrientation);

      armInitialJointPositions.put(RobotSide.LEFT, leftArmInitialJointPositions);
      armInitialJointPositions.put(RobotSide.RIGHT, rightArmInitialJointPositions);
   }

   public static class DRCRobotControllerParametersGettingIntoTheCar extends DRCRobotMultiContactControllerParameters
   {
      public Map<OneDoFJoint, Double> getDefaultArmJointPositions(FullRobotModel fullRobotModel, RobotSide robotSide)
      {
         Map<OneDoFJoint, Double> jointPositions = new LinkedHashMap<OneDoFJoint, Double>();
         Map<ArmJointName, Double> armInitialJointPositions = SquaredUpDRCRobotInitialSetupGettingIntoTheCar.armInitialJointPositions.get(robotSide);
         for (ArmJointName armJointName : armInitialJointPositions.keySet())
         {
            jointPositions.put(fullRobotModel.getArmJoint(robotSide, armJointName), armInitialJointPositions.get(armJointName));
         }
         return jointPositions;
      }
   }

   public void getOffset(Vector3d offsetToPack)
   {
      offsetToPack.set(new Vector3d(pelvisInitialPose.getX(), pelvisInitialPose.getY(), pelvisInitialPose.getZ()));
   }
}
