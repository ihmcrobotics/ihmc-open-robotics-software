package us.ihmc.openAlexander;

import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * This class contains the arm configurations for Alexander
 */
public class AlexanderPresetArmConfigurations
{
   //TODO Set configurations, only stand prep and n-pose are correct
   private static final SideDependentList<double[]> STAND_PREP_4DOF = new SideDependentList<>();
   private static final SideDependentList<double[]> HOME_4DOF = new SideDependentList<>();
   private static final SideDependentList<double[]> TUCKED_UP_ARMS_4DOF = new SideDependentList<>();

   private static final SideDependentList<double[]> STAND_PREP_7DOF = new SideDependentList<>();
   private static final SideDependentList<double[]> HOME_7DOF = new SideDependentList<>();
   private static final SideDependentList<double[]> TUCKED_UP_ARMS_7DOF = new SideDependentList<>();

   static
   {
      for (RobotSide side : RobotSide.values)
      {
         HOME_4DOF.put(side, new double[] {0.018, side.negateIfRightSide(0.146), side.negateIfRightSide(-0.202), -0.312});
         STAND_PREP_4DOF.put(side, HOME_4DOF.get(side));
         TUCKED_UP_ARMS_4DOF.put(side, new double[] {0.104, side.negateIfRightSide(0.347), side.negateIfRightSide(-0.094), -1.2});

         HOME_7DOF.put(side, new double[]{0.018, side.negateIfRightSide(0.146), side.negateIfRightSide(-0.202), -0.312, side.negateIfRightSide(0.0), side.negateIfRightSide(0.0), side.negateIfRightSide(0.0)});
         STAND_PREP_7DOF.put(side, HOME_7DOF.get(side));
         TUCKED_UP_ARMS_7DOF.put(side, new double[]{0.104, side.negateIfRightSide(0.347), side.negateIfRightSide(-0.094), -1.2, side.negateIfRightSide(0.0), side.negateIfRightSide(0.0), side.negateIfRightSide(0.0)});
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
         jointAngles = new double[HOME_7DOF.get(RobotSide.LEFT).length];
         LogTools.warn("ASSUMING 7DOF");
         getCycloid7DoFArmConfigurations(side, presetArmConfiguration, jointAngles);
      }
      else
      {
         LogTools.warn("ASSUMING 4DOF");
         jointAngles = new double[HOME_4DOF.get(RobotSide.LEFT).length];
         getCycloid4DoFArmConfigurations(side, presetArmConfiguration, jointAngles);
      }

      return jointAngles;
   }

   public static void getCycloid4DoFArmConfigurations(RobotSide side, PresetArmConfiguration presetArmConfiguration, double[] jointAnglesToPack)
   {
      switch (presetArmConfiguration)
      {
         case STAND_PREP -> System.arraycopy(STAND_PREP_4DOF.get(side), 0, jointAnglesToPack, 0, STAND_PREP_4DOF.get(side).length);
         case HOME, N_POSE -> System.arraycopy(HOME_4DOF.get(side), 0, jointAnglesToPack, 0, HOME_4DOF.get(side).length);
         case TUCKED_UP_ARMS -> System.arraycopy(TUCKED_UP_ARMS_4DOF.get(side), 0, jointAnglesToPack, 0, TUCKED_UP_ARMS_4DOF.get(side).length);
      }
   }

   public static void getCycloid7DoFArmConfigurations(RobotSide side, PresetArmConfiguration presetArmConfiguration, double[] jointAnglesToPack)
   {
      switch (presetArmConfiguration)
      {
         case STAND_PREP -> System.arraycopy(STAND_PREP_7DOF.get(side), 0, jointAnglesToPack, 0, STAND_PREP_7DOF.get(side).length);
         case HOME, N_POSE -> System.arraycopy(HOME_7DOF.get(side), 0, jointAnglesToPack, 0, HOME_7DOF.get(side).length);
         case TUCKED_UP_ARMS -> System.arraycopy(TUCKED_UP_ARMS_7DOF.get(side), 0, jointAnglesToPack, 0, TUCKED_UP_ARMS_7DOF.get(side).length);
      }
   }
}
