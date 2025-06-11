package us.ihmc.alexander;

import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * This class contains the arm configurations for Nadia; this allows for the Cycloid arms, the TMotor arms, and the Cycloid arms
 * with the TMotor forearms with the SAKE hands.
 */
public class AlexanderPresetArmConfigurations
{
   private static final SideDependentList<double[]> STAND_PREP_CYCLOID_4DOF = new SideDependentList<>();
   private static final SideDependentList<double[]> HOME_CYCLOID_4DOF = new SideDependentList<>();
   private static final SideDependentList<double[]> TUCKED_UP_ARMS_CYCLOID_4DOF = new SideDependentList<>();

   private static final SideDependentList<double[]> STAND_PREP_CYCLOID_7DOF = new SideDependentList<>();
   private static final SideDependentList<double[]> HOME_CYCLOID_7DOF = new SideDependentList<>();
   private static final SideDependentList<double[]> TUCKED_UP_ARMS_CYCLOID_7DOF = new SideDependentList<>();

   // To tune these values, run NadiaModelViewer as it allows you to tune joint angles with sliders
   static
   {
      for (RobotSide side : RobotSide.values)
      {
         HOME_CYCLOID_4DOF.put(side, new double[] {0.018, side.negateIfRightSide(0.146), side.negateIfRightSide(-0.202), -0.312});
         STAND_PREP_CYCLOID_4DOF.put(side, HOME_CYCLOID_4DOF.get(side));
         TUCKED_UP_ARMS_CYCLOID_4DOF.put(side, new double[] {0.104, side.negateIfRightSide(0.347), side.negateIfRightSide(-0.094), -1.2});

         HOME_CYCLOID_7DOF.put(side, new double[]{0.018, side.negateIfRightSide(0.146), side.negateIfRightSide(-0.202), -0.312, side.negateIfRightSide(0.0), side.negateIfRightSide(0.0), side.negateIfRightSide(0.0)});
         STAND_PREP_CYCLOID_7DOF.put(side, HOME_CYCLOID_7DOF.get(side));
         TUCKED_UP_ARMS_CYCLOID_7DOF.put(side, new double[]{0.104, side.negateIfRightSide(0.347), side.negateIfRightSide(-0.094), -1.2, side.negateIfRightSide(0.0), side.negateIfRightSide(0.0), side.negateIfRightSide(0.0)});
      }
   }

   /**
    * @return a copy so the original values don't get modified.
    */
   public static double[] getPresetArmConfiguration(AlexanderVersionInterface robotVersion, RobotSide side, PresetArmConfiguration presetArmConfiguration)
   {
      double[] jointAngles;

      if (robotVersion.getJointMap().hasCycloidForearm(side))
      {
         jointAngles = new double[HOME_CYCLOID_7DOF.get(RobotSide.LEFT).length];
         getCycloid7DoFArmConfigurations(side, presetArmConfiguration, jointAngles);
      }
      else
      {
         jointAngles = new double[HOME_CYCLOID_4DOF.get(RobotSide.LEFT).length];
         getCycloid4DoFArmConfigurations(side, presetArmConfiguration, jointAngles);
      }

      return jointAngles;
   }

   public static void getCycloid4DoFArmConfigurations(RobotSide side, PresetArmConfiguration presetArmConfiguration, double[] jointAnglesToPack)
   {
      switch (presetArmConfiguration)
      {
         case STAND_PREP -> System.arraycopy(STAND_PREP_CYCLOID_4DOF.get(side), 0, jointAnglesToPack, 0, STAND_PREP_CYCLOID_4DOF.get(side).length);
         case HOME -> System.arraycopy(HOME_CYCLOID_4DOF.get(side), 0, jointAnglesToPack, 0, HOME_CYCLOID_4DOF.get(side).length);
         case TUCKED_UP_ARMS -> System.arraycopy(TUCKED_UP_ARMS_CYCLOID_4DOF.get(side), 0, jointAnglesToPack, 0, TUCKED_UP_ARMS_CYCLOID_4DOF.get(side).length);
      }
   }

   public static void getCycloid7DoFArmConfigurations(RobotSide side, PresetArmConfiguration presetArmConfiguration, double[] jointAnglesToPack)
   {
      switch (presetArmConfiguration)
      {
         case STAND_PREP -> System.arraycopy(STAND_PREP_CYCLOID_7DOF.get(side), 0, jointAnglesToPack, 0, STAND_PREP_CYCLOID_7DOF.get(side).length);
         case HOME -> System.arraycopy(HOME_CYCLOID_7DOF.get(side), 0, jointAnglesToPack, 0, HOME_CYCLOID_7DOF.get(side).length);
         case TUCKED_UP_ARMS -> System.arraycopy(TUCKED_UP_ARMS_CYCLOID_7DOF.get(side), 0, jointAnglesToPack, 0, TUCKED_UP_ARMS_CYCLOID_7DOF.get(side).length);
      }
   }
}
