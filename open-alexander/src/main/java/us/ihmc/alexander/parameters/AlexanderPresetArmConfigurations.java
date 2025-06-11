package us.ihmc.alexander.parameters;

import us.ihmc.alexander.AlexanderVersionInterface;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * This class contains the arm configurations for Alexander; this allows for the Cycloid arms, the TMotor arms, and the Cycloid arms
 * with the TMotor forearms with the SAKE hands.
 */
public class AlexanderPresetArmConfigurations
{
   private static final SideDependentList<double[]> INITIAL_SETUP_CYCLOID_4DOF = new SideDependentList<>();
   private static final SideDependentList<double[]> STAND_PREP_CYCLOID_4DOF = new SideDependentList<>();
   private static final SideDependentList<double[]> HOME_CYCLOID_4DOF = new SideDependentList<>();
   private static final SideDependentList<double[]> WIDE_ARMS_CYCLOID_4DOF = new SideDependentList<>();
   private static final SideDependentList<double[]> TUCKED_UP_ARMS_CYCLOID_4DOF = new SideDependentList<>();
   private static final SideDependentList<double[]> DOOR_AVOIDANCE_CYCLOID_4DOF = new SideDependentList<>();

   private static final SideDependentList<double[]> INITIAL_SETUP_CYCLOID_7DOF = new SideDependentList<>();
   private static final SideDependentList<double[]> STAND_PREP_CYCLOID_7DOF = new SideDependentList<>();
   private static final SideDependentList<double[]> HOME_CYCLOID_7DOF = new SideDependentList<>();
   private static final SideDependentList<double[]> TUCKED_UP_ARMS_CYCLOID_7DOF = new SideDependentList<>();

   // To tune these values, run AlexanderModelViewer as it allows you to tune joint angles with sliders
   static
   {
      for (RobotSide side : RobotSide.values)
      {
         INITIAL_SETUP_CYCLOID_4DOF.put(side, new double[] {0.4, side.negateIfRightSide(0.3), side.negateIfRightSide(0.0), -1.4});
         STAND_PREP_CYCLOID_4DOF.put(side, new double[] {0.3, side.negateIfRightSide(0.1), side.negateIfRightSide(0.2), -0.4});
         HOME_CYCLOID_4DOF.put(side, new double[] {0.6, side.negateIfRightSide(-0.1), side.negateIfRightSide(0.4), -0.8});
         WIDE_ARMS_CYCLOID_4DOF.put(side, new double[] {0.6, side.negateIfRightSide(0.3), side.negateIfRightSide(0.4), -0.8});
         TUCKED_UP_ARMS_CYCLOID_4DOF.put(side, new double[] {0.6, side.negateIfRightSide(0.0), side.negateIfRightSide(0.25), -1.3});
         DOOR_AVOIDANCE_CYCLOID_4DOF.put(side, new double[] {-0.15, side.negateIfRightSide(0.0), side.negateIfRightSide(0.4), -2.0});

         INITIAL_SETUP_CYCLOID_7DOF.put(side, new double[]{0.7, side.negateIfRightSide(0.2), side.negateIfRightSide(0.0), -1.4, side.negateIfRightSide(0.0), side.negateIfRightSide(0.0), side.negateIfRightSide(0.0)});
         STAND_PREP_CYCLOID_7DOF.put(side, new double[]{0.3, side.negateIfRightSide(0.1), side.negateIfRightSide(0.2), -0.4, side.negateIfRightSide(0.0), side.negateIfRightSide(0.0), side.negateIfRightSide(0.0)});
         HOME_CYCLOID_7DOF.put(side, new double[]{0.7, side.negateIfRightSide(0.0), side.negateIfRightSide(0.0), -1.2, side.negateIfRightSide(0.0), side.negateIfRightSide(0.0), side.negateIfRightSide(0.0)});
         TUCKED_UP_ARMS_CYCLOID_7DOF.put(side, new double[]{0.6, side.negateIfRightSide(0.0), side.negateIfRightSide(0.25), -1.3, side.negateIfRightSide(0.0), side.negateIfRightSide(0.0), side.negateIfRightSide(0.0)});
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
         case INITIAL_SETUP -> System.arraycopy(INITIAL_SETUP_CYCLOID_4DOF.get(side), 0, jointAnglesToPack, 0, INITIAL_SETUP_CYCLOID_4DOF.get(side).length);
         case STAND_PREP -> System.arraycopy(STAND_PREP_CYCLOID_4DOF.get(side), 0, jointAnglesToPack, 0, STAND_PREP_CYCLOID_4DOF.get(side).length);
         case HOME -> System.arraycopy(HOME_CYCLOID_4DOF.get(side), 0, jointAnglesToPack, 0, HOME_CYCLOID_4DOF.get(side).length);
         case WIDE_ARMS -> System.arraycopy(WIDE_ARMS_CYCLOID_4DOF.get(side), 0, jointAnglesToPack, 0, WIDE_ARMS_CYCLOID_4DOF.get(side).length);
         case TUCKED_UP_ARMS -> System.arraycopy(TUCKED_UP_ARMS_CYCLOID_4DOF.get(side), 0, jointAnglesToPack, 0, TUCKED_UP_ARMS_CYCLOID_4DOF.get(side).length);
         case DOOR_AVOIDANCE -> System.arraycopy(DOOR_AVOIDANCE_CYCLOID_4DOF.get(side), 0, jointAnglesToPack, 0, DOOR_AVOIDANCE_CYCLOID_4DOF.get(side).length);
      }
   }

   public static void getCycloid7DoFArmConfigurations(RobotSide side, PresetArmConfiguration presetArmConfiguration, double[] jointAnglesToPack)
   {
      switch (presetArmConfiguration)
      {
         case INITIAL_SETUP -> System.arraycopy(INITIAL_SETUP_CYCLOID_7DOF.get(side), 0, jointAnglesToPack, 0, INITIAL_SETUP_CYCLOID_7DOF.get(side).length);
         case STAND_PREP -> System.arraycopy(STAND_PREP_CYCLOID_7DOF.get(side), 0, jointAnglesToPack, 0, STAND_PREP_CYCLOID_7DOF.get(side).length);
         case HOME -> System.arraycopy(HOME_CYCLOID_7DOF.get(side), 0, jointAnglesToPack, 0, HOME_CYCLOID_7DOF.get(side).length);
         case TUCKED_UP_ARMS -> System.arraycopy(TUCKED_UP_ARMS_CYCLOID_7DOF.get(side), 0, jointAnglesToPack, 0, TUCKED_UP_ARMS_CYCLOID_7DOF.get(side).length);
      }
   }
}
