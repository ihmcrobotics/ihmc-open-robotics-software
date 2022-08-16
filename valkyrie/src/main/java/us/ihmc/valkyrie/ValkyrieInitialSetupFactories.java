package us.ihmc.valkyrie;

import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public class ValkyrieInitialSetupFactories
{
   private static final double halfPi = 0.5 * Math.PI;

   public static ValkyrieMutableInitialSetup newCrawl1(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.5, 0.0, -1.0, 1.2, 0.2, 0.0);
      initialSetup.setArmJointQs(-2.0, -1.2, 0.0, -1.2);
      initialSetup.setRootJointPose(0.0, 0.0, 0.39, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newCrawl2(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.25, 1.75, -0.8, 0.0);
      initialSetup.setArmJointQs(-1.5, -0.65, 1.5, -1.4);
      initialSetup.setSpineJointQs(0.0, 0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.56, 0.0, 0.4, 0.0, 0.5);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newKneel1(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -0.3, 1.9, 0.8, 0.0);
      initialSetup.setArmJointQs(0.7, -1.2, 0.0, -1.7);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.76, 0.0, 0.0, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newKneel2(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, 0.75, 2.05, -0.8, 0.0);
      initialSetup.setArmJointQs(0.7, -1.2, 0.0, -1.7);
      initialSetup.setSpineJointQs(0.0, 0.4, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.64, 0.0, -0.4, 0.0, 0.8);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newSit1(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.9, 1.525, 0.775, 0.0);
      initialSetup.setArmJointQs(1.1, -1.0, 1.1, -0.9);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.33, 0.0, -0.2, 0.0, 1.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newSit2(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.2, 1.525, 0.775, 0.0);
      initialSetup.setArmJointQs(1.4, 0.2, 1.5, -1.8);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.23, 0.0, -0.6, 0.0, 1.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newSit3(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -0.875, 1.425, 0.8, 0.0);
      initialSetup.setArmJointQs(-0.5, 1.266, -3.1, -2.0);
      initialSetup.setSpineJointQs(0.0, 0.4, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.17, 0.0, -0.8, 0.0, 1.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newQuadToKneel1(HumanoidJointNameMap jointMap)
   {
      // TODO Needs joint limits to be removed
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.15, 2.7, -0.8, 0.0);
      initialSetup.setArmJointQs(-1.35, -1.519, 1.2, -0.2);
      initialSetup.setSpineJointQs(0.0, 0.9, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.6, 0.0, 0.35, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newQuadToKneel2(HumanoidJointNameMap jointMap)
   {
      // TODO Needs joint limits to be removed
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.15, 2.4, 0.8, 0.0);
      initialSetup.setArmJointQs(-1.35, -1.519, 1.2, -0.2);
      initialSetup.setSpineJointQs(0.0, 0.9, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.61, 0.0, 0.35, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newQuadToKneel3(HumanoidJointNameMap jointMap)
   {
      // TODO Needs joint limits to be removed
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.5, -1.15, 2.375, 0.8, 0.0);
      initialSetup.setArmJointQs(-1.35, -1.519, 1.2, -0.2);
      initialSetup.setSpineJointQs(0.0, 0.8, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.565, 0.0, 0.35, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newQuadToKneel4(HumanoidJointNameMap jointMap)
   {
      // TODO Needs joint limits to be removed
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.5, -1.15, 2.25, 0.8, 0.0);
      initialSetup.setArmJointQs(-1.35, -1.519, 1.2, -0.2);
      initialSetup.setSpineJointQs(0.0, 0.75, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.59, 0.0, 0.45, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newTriToKneel1(HumanoidJointNameMap jointMap)
   {
      // TODO Needs joint limits to be removed
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.15, 2.7, -0.8, 0.0);
      initialSetup.setArmJointQs(-1.35, -1.519, 1.2, -0.2);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -0.35, 1.3, 0.0, 0.2);
      initialSetup.setSpineJointQs(0.0, 0.9, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.6, 0.0, 0.35, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newTriToKneel2(HumanoidJointNameMap jointMap)
   {
      // TODO Needs joint limits to be removed
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.15, 2.4, 0.8, 0.0);
      initialSetup.setArmJointQs(-1.35, -1.519, 1.2, -0.2);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -0.35, 1.3, 0.0, 0.2);
      initialSetup.setSpineJointQs(0.0, 0.9, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.61, 0.0, 0.35, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newTriToKneel3(HumanoidJointNameMap jointMap)
   {
      // TODO Needs joint limits to be removed
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.5, -1.15, 2.375, 0.8, 0.0);
      initialSetup.setArmJointQs(-1.35, -1.519, 1.2, -0.2);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -0.35, 1.3, 0.0, 0.2);
      initialSetup.setSpineJointQs(0.0, 0.8, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.565, 0.0, 0.35, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newTriToKneel4(HumanoidJointNameMap jointMap)
   {
      // TODO Needs joint limits to be removed
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.5, -1.15, 2.25, 0.8, 0.0);
      initialSetup.setArmJointQs(-1.35, -1.519, 1.2, -0.2);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -0.35, 1.3, 0.0, 0.2);
      initialSetup.setSpineJointQs(0.0, 0.75, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.585, 0.0, 0.45, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newToKneel1(HumanoidJointNameMap jointMap)
   {
      // TODO Needs joint limits to be removed
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.15, 2.7, -0.8, 0.0);
      initialSetup.setArmJointQs(-1.25, -1.519, 1.2, -0.2);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -0.35, 1.3, 0.0, 0.2);
      initialSetup.setSpineJointQs(0.0, 0.8, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.6, 0.0, 0.35, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newToKneel2(HumanoidJointNameMap jointMap)
   {
      // TODO Needs joint limits to be removed
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.15, 2.4, 0.8, 0.0);
      initialSetup.setArmJointQs(-1.25, -1.519, 1.2, -0.2);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -0.35, 1.3, 0.0, 0.2);
      initialSetup.setSpineJointQs(0.0, 0.8, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.61, 0.0, 0.35, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newToKneel3(HumanoidJointNameMap jointMap)
   {
      // TODO Needs joint limits to be removed
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.5, -1.15, 2.375, 0.8, 0.0);
      initialSetup.setArmJointQs(-1.25, -1.519, 1.2, -0.2);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -0.35, 1.3, 0.0, 0.2);
      initialSetup.setSpineJointQs(0.0, 0.7, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.565, 0.0, 0.35, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newToKneel4(HumanoidJointNameMap jointMap)
   {
      // TODO Needs joint limits to be removed
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.5, -1.15, 2.25, 0.8, 0.0);
      initialSetup.setArmJointQs(-1.25, -1.519, 1.2, -0.2);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -0.35, 1.3, 0.0, 0.2);
      initialSetup.setSpineJointQs(0.0, 0.65, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.585, 0.0, 0.45, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newToKneel5(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.5, -0.75, 2.05, 0.8, 0.0);
      initialSetup.setArmJointQs(-0.8, -1.519, 1.2, -0.2);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -0.35, 1.3, 0.0, 0.2);
      initialSetup.setSpineJointQs(0.0, 0.4, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.635, 0.0, 0.30, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingA1(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.8, 0.2, -0.33, 1.6, 0.3, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, 1.4, 0.26, 1.5, -1.8);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.8, -0.1, -0.6, 1.4, 0.0, 0.0);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 1.7, 1.3, 0.0, 1.95);
      initialSetup.setSpineJointQs(0.85, 0.34, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.26, halfPi, -0.785, halfPi);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingA2(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.8, 0.2, -0.33, 1.6, 0.3, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -0.15, 1.266, -3.1, -2.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.8, -0.1, -0.6, 1.4, 0.0, 0.0);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 1.7, 1.3, 0.0, 1.95);
      initialSetup.setSpineJointQs(0.85, 0.34, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.26, halfPi, -0.785, halfPi);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingA3(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.8, 0.2, -0.45, 1.4, 0.425, -0.2);
      initialSetup.setArmJointQs(RobotSide.LEFT, 1.7, -1.5, 0.0, -1.2);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.55, 0.13, -0.6, 1.4, 0.0, 0.0);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 1.6, 1.3, 0.0, 2.0);
      initialSetup.setSpineJointQs(0.60, 0.36, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.31, halfPi, -0.585, 1.3);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingA4(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.975, 0.5, -0.45, 1.4, 0.425, -0.3);
      initialSetup.setArmJointQs(RobotSide.LEFT, 1.7, -1.5, 0.0, -1.2);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.41, 0.25, -0.6, 1.4, 0.0, 0.0);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 1.2, 1.1, 0.0, 2.0);
      initialSetup.setSpineJointQs(0.30, 0.36, -0.2);
      initialSetup.setRootJointPose(0.0, 0.0, 0.34, halfPi, -0.385, 1.2);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingA5(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 1.075, 0.5, -0.25, 1.4, 0.225, -0.3);
      initialSetup.setArmJointQs(RobotSide.LEFT, 1.7, -1.5, 0.0, -1.2);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.01, 0.3, -0.6, 1.4, 0.0, 0.0);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 1.2, 0.9, 0.3, 2.0);
      initialSetup.setSpineJointQs(0.10, 0.36, -0.2);
      initialSetup.setRootJointPose(0.0, 0.0, 0.34, halfPi, 0.0, 1.2);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingA6(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.875, 0.5, -0.55, 1.4, 0.225, -0.3);
      initialSetup.setArmJointQs(RobotSide.LEFT, 1.7, -1.5, 0.0, -1.2);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.01, 0.3, -0.6, 1.4, 0.0, 0.0);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 1.2, 0.9, 0.3, 2.0);
      initialSetup.setSpineJointQs(0.10, 0.36, -0.2);
      initialSetup.setRootJointPose(0.0, 0.0, 0.34, halfPi, 0.0, 1.2);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingA7(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, -0.414, 0.35, -0.95, 0.4, 0.225, -0.3);
      initialSetup.setArmJointQs(RobotSide.LEFT, 1.7, -1.5, 0.0, -1.2);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.01, 0.3, -0.6, 1.4, 0.0, 0.0);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 1.2, 0.9, 0.3, 2.0);
      initialSetup.setSpineJointQs(0.10, 0.36, -0.2);
      initialSetup.setRootJointPose(0.0, 0.0, 0.34, halfPi, 0.0, 1.2);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newLieDown1(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -0.325, 1.1, 0.8, 0.0);
      initialSetup.setArmJointQs(0.0, -1.5, 0.0, -2.0);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.245, 0.707, 0.0, 0.707, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newLieDown2(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -0.325, 1.1, 0.8, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, halfPi, 0.4, halfPi, -1.75);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 0.0, 1.5, 0.0, 2.0);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.245, 0.707, 0.0, 0.707, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingB1(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.5, 0.1, -0.25, 1.2, 0.625, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, halfPi, -0.5, halfPi, -1.45);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.2, 0.0, -0.375, 1.1, 0.82, 0.0);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 0.0, 1.5, 0.0, 2.0);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.275, halfPi, -1.07, halfPi);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingB2(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.5, 0.1, -0.22, 1.2, 0.625, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, halfPi, -0.75, halfPi, -1.35);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.2, 0.0, -0.375, 1.1, 0.82, 0.0);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 0.0, 1.5, 0.0, 2.0);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.26, halfPi, -0.9, halfPi);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingB3(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.7, 0.3, -0.05, 1.2, 0.5, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, halfPi, -1.25, halfPi, -1.35);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.65, -0.12, -0.375, 1.1, 0.82, 0.0);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 0.0, 1.5, 0.0, 2.0);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.3, halfPi, -0.6, halfPi);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingB4(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 1.0, 0.4, 0.025, 1.2, 0.45, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, halfPi, -1.25, halfPi, -1.35);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.35, -0.13, -0.375, 1.1, 0.82, 0.0);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 0.0, 1.519, -0.3, 2.0);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.32, halfPi, -0.3, halfPi);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingB5(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 1.1, 0.4, 0.2, 1.2, 0.4, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -halfPi, -1.519, halfPi, -0.7);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.075, -0.15, -0.375, 1.1, 0.82, 0.0);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 0.0, 1.519, 0.0, 2.0);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.335, halfPi, 0.0, halfPi);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingB6(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 1.1, 0.4, 0.175, 1.2, 0.4, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -halfPi, -1.519, halfPi, -1.3);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.075, -0.13, -0.375, 1.1, 0.82, 0.0);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 0.0, 1.519, 0.4, 2.0);
      initialSetup.setSpineJointQs(-0.4, 0.0, 0.075);
      initialSetup.setRootJointPose(0.0, 0.0, 0.32, halfPi, 0.0, halfPi);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingB7(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, -0.4, -0.2, -0.825, 0.5, 0.4, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -halfPi, -1.519, halfPi, -1.3);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.075, -0.13, -0.375, 1.1, 0.82, 0.0);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 0.0, 1.519, 0.4, 2.0);
      initialSetup.setSpineJointQs(-0.4, 0.0, 0.075);
      initialSetup.setRootJointPose(0.0, 0.0, 0.32, halfPi, 0.0, halfPi);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingB8(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, -0.4, 0.0, -1.425, 1.3, -0.8, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -halfPi, -1.519, 1.3, -1.3);
      initialSetup.setLegJointQs(RobotSide.RIGHT, 0.05, -0.13, -0.475, 0.8, 0.82, 0.0);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 0.0, 1.45, 0.45, 2.0);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.1);
      initialSetup.setRootJointPose(0.0, 0.0, 0.35, halfPi, 0.3, halfPi);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingB9(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, -0.4, 0.0, -1.625, 2.0, -0.8, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -halfPi, -1.519, 1.35, -1.5);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.65, -0.2, -0.425, 0.775, -0.8, 0.0);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 0.0, 1.45, 0.45, 2.0);
      initialSetup.setSpineJointQs(0.25, 0.0, 0.15);
      initialSetup.setRootJointPose(0.0, 0.0, 0.36, halfPi, 0.7, halfPi);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingB10(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.1, 0.0, -1.625, 2.057, -0.8, 0.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.8, -0.2, -0.825, 1.175, -0.8, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -halfPi, -1.519, 1.35, -1.5);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 0.0, 1.45, 0.45, 2.0);
      initialSetup.setSpineJointQs(0.65, 0.0, 0.15);
      initialSetup.setRootJointPose(0.0, 0.0, 0.36, halfPi, 1.1, halfPi);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingB11(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.7, 0.0, -1.625, 2.057, -0.8, 0.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.1, -0.2, -0.825, 1.175, -0.8, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -halfPi, -1.519, 1.35, -1.5);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 0.0, 1.45, 0.45, 2.0);
      initialSetup.setSpineJointQs(1.15, 0.0, 0.15);
      initialSetup.setRootJointPose(0.0, 0.0, 0.36, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingB12(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.7, 0.0, -1.625, 2.057, -0.8, 0.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.5, -0.2, -1.325, 1.725, -0.8, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -halfPi, -1.519, 1.35, -1.5);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 0.0, 1.45, 0.45, 2.0);
      initialSetup.setSpineJointQs(1.15, 0.0, 0.15);
      initialSetup.setRootJointPose(0.0, 0.0, 0.36, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingB13(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.7, 0.0, -1.625, 2.057, -0.8, 0.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.5, -0.2, -1.325, 1.725, -0.8, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -halfPi, -1.519, 1.35, -1.5);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 0.0, 1.275, 0.65, 2.0);
      initialSetup.setSpineJointQs(0.95, -0.1, 0.05);
      initialSetup.setRootJointPose(0.0, 0.0, 0.36, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingB14(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.7, 0.0, -1.625, 2.057, -0.8, 0.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.5, -0.2, -1.325, 1.725, -0.8, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -halfPi, -1.0, 1.35, -1.5);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 1.3, 1.375, -0.55, 2.0);
      initialSetup.setSpineJointQs(0.65, -0.1, 0.05);
      initialSetup.setRootJointPose(0.0, 0.0, 0.36, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingB15(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.7, 0.0, -1.625, 2.057, -0.8, 0.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.5, -0.2, -1.325, 1.725, -0.8, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -halfPi, -1.0, 1.35, -1.5);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 1.5, 1.275, -1.55, 2.0);
      initialSetup.setSpineJointQs(0.65, -0.1, 0.05);
      initialSetup.setRootJointPose(0.0, 0.0, 0.36, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA1(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -0.25, 0.5, -0.25, 0.0);
      initialSetup.setArmJointQs(0.7, -1.5, 0.0, -2.0);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.185, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA2a(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -0.5, 0.75, -0.25, 0.0);
      initialSetup.setArmJointQs(0.45, -1.5, 0.0, -2.0);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.285, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA2b(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -0.5, 0.75, -0.25, 0.0);
      initialSetup.setArmJointQs(-1.55, 0.35, 1.5, -2.0);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.285, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA2c(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -0.5, 0.75, -0.25, 0.0);
      initialSetup.setArmJointQs(-0.25, 1.266, 0.0, -2.0);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.285, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA3(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.3, 1.625, -0.65, 0.0);
      initialSetup.setArmJointQs(-2.25, -1.334, 1.5, -0.5);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.485, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA4(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.3, 1.625, -0.65, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -1.45, -0.8, 1.5, -2.0);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -2.4, 1.45, 1.5, 0.5);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.485, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA5a(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.3, 1.625, -0.65, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -1.45, -0.3, 1.5, -1.7);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -2.25, 1.334, 1.5, 0.5);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.485, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA5b(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.3, 1.625, -0.65, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -0.25, -1.4, 0.2, -1.7);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -2.25, 1.334, 1.5, 0.5);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.485, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA5c(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.3, 1.625, -0.65, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -2.05, -1.4, 1.5, -1.7);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -2.75, 1.334, 1.5, 0.5);
      initialSetup.setSpineJointQs(0.0, 0.2, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.485, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA6a(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.6, 0.0, -1.5, 1.925, -0.65, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -1.5, -0.6, 1.1, -2.0);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -2.0, 0.7, 0.7, 1.7);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.4, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA6b(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.6, 0.0, -1.5, 1.925, -0.65, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -1.5, -1.5, 1.5, -1.7);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -2.0, 0.7, 0.7, 1.7);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.4, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA7a(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.6, 0.0, -1.5, 1.925, -0.65, 0.0);
      initialSetup.setArmJointQs(-1.5, -0.6, 1.1, -2.0);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.4, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA7b(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.6, 0.25, -1.8, 2.05, -0.8, 0.0);
      initialSetup.setArmJointQs(-1.8, -0.775, 1.1, -1.7);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.4, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA7c(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.45, 0.55, -1.25, 2.05, -0.8, 0.0);
      initialSetup.setArmJointQs(-1.55, -1.519, 1.1, -0.2);
      initialSetup.setSpineJointQs(0.0, 0.075, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.5, 0.0, 1.0, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA7d(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.45, 0.55, -0.9, 2.05, -0.8, 0.3);
      initialSetup.setArmJointQs(-1.45, -1.519, 1.1, -0.2);
      initialSetup.setSpineJointQs(0.0, 0.4, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.53, 0.0, 0.7, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA7e(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -0.875, 2.05, -0.8, 0.0);
      initialSetup.setArmJointQs(-1.35, -1.519, 1.2, -0.2);
      initialSetup.setSpineJointQs(0.0, 0.666, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.67, 0.0, 0.75, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA10(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -0.15, 2.05, -0.8, 0.0);
      initialSetup.setArmJointQs(0.0, -1.3, 0.4, -0.7);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.74, 0.0, 0.0, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA11(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.0, 0.5, -0.15, 2.05, -0.8, 0.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, 0.0, 0.4, -0.15, 2.05, -0.8, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, 0.0, -0.6, 0.4, -0.7);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 0.0, 1.5, 0.4, 1.3);
      initialSetup.setSpineJointQs(0.0, 0.0, -0.23);
      initialSetup.setRootJointPose(0.0, 0.0, 0.755, 0.0, 0.0, -0.5);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA12(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.0, 0.5, -0.15, 2.05, -0.8, 0.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, 0.0, 0.4, -1.4, 1.35, 0.05, 0.1);
      initialSetup.setArmJointQs(RobotSide.LEFT, 0.0, -0.6, 0.4, -0.7);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 0.0, 1.5, 0.4, 1.3);
      initialSetup.setSpineJointQs(0.0, 0.0, -0.23);
      initialSetup.setRootJointPose(0.0, 0.0, 0.755, 0.0, 0.0, -0.5);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA13a(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.0, 0.5, -0.15, 2.05, -0.8, 0.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, 0.0, 0.4, -1.35, 1.35, 0.0, 0.1);
      initialSetup.setArmJointQs(RobotSide.LEFT, 0.0, -0.6, 0.4, -0.7);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 0.0, 1.5, 0.4, 1.3);
      initialSetup.setSpineJointQs(0.0, 0.0, -0.23);
      initialSetup.setRootJointPose(0.0, 0.0, 0.755, 0.0, 0.0, -0.5);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA13b(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.0, 0.0, -0.15, 2.05, -0.8, 0.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, 0.0, 0.0, -1.55, 1.35, 0.2, 0.0);
      initialSetup.setArmJointQs(0.0, -1.2, 0.4, -0.7);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.735, 0.0, 0.0, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA15a(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.0, 0.0, 0.05, 1.6, -0.5, 0.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, 0.0, 0.0, -1.31, 1.35, -0.04, 0.0);
      initialSetup.setArmJointQs(0.0, -1.2, 0.4, -0.7);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.85, 0.0, 0.0, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA15b(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.0, 0.0, -0.25, 1.59, -0.5, 0.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, 0.0, 0.0, -1.62, 1.35, -0.03, 0.0);
      initialSetup.setArmJointQs(0.0, -1.2, 0.4, -0.7);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.835, 0.0, 0.3, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA16(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.0, 0.06, 0.365, 0.3, 0.5, 0.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, 0.0, 0.1, -1.1, 1.2, -0.4, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, 0.0, -1.4, 0.4, -0.7);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 0.0, 1.2, 0.4, 0.7);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.2);
      initialSetup.setRootJointPose(0.0, 0.0, 0.995, 0.0, 0.3, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceC5(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, -0.4, 0.0, -1.3, 1.625, -0.65, 0.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.3, 0.0, -1.3, 1.625, -0.65, 0.0);
      initialSetup.setArmJointQs(-2.25, -1.334, 1.5, -0.5);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.485, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceC6(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, -0.4, 0.0, -1.3, 1.625, -0.65, 0.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.3, 0.0, -1.3, 1.625, -0.65, 0.0);
      initialSetup.setArmJointQs(-2.25, -1.334, 1.5, -0.5);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.485, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   /**
    * Order of joint angles in JSON file:
    * 1. Left leg (6 values): 7.919233E-5, -0.03010657, -1.5434167, 2.0578954, -0.80010957, -2.4982874E-5,
    * 2. Right leg (6 values): 7.073794E-5, -0.03009036, -1.5432106, 2.0563831, -0.8001095, -2.4546394E-5,
    * 3. Spine (3 values): 2.5291948E-4, 0.025048098, -3.908084E-5,
    * 4. Left arm (4 values): -1.5521886, -1.1981736, 1.3492163, -1.1037868,
    * 5. Neck (4 values): 0.009494689, -5.318392E-5, 0.00210515, 1.1967057,
    * 6. Right arm (4 values): -1.5522051, 1.1981682, 1.3492138, 1.1036909
    */
   public static ValkyrieMutableInitialSetup newAllFoursBellyDown(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.0, -0.03, -1.54, 2.057, -0.8, 0.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, 0.0, -0.03, -1.54, 2.057, -0.8, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -1.55, -1.2, 1.35, -1.1);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -1.55, 1.2, 1.35, 1.1);
      initialSetup.setSpineJointQs(0.0, 0.025, 0.0);
      initialSetup.setRootJointPose(-0.13, -0.006, 0.57, 0.0, 0.6181, 0.0, 0.7861);
      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newAllFoursPelvisLevel(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.065160476, 0.3997581, -1.6374844, 1.8006114, -0.8154475, 0.016774062);
      initialSetup.setLegJointQs(RobotSide.RIGHT, 0.10306259, 0.25142053, -1.9145228, 1.6526313, -1.1029923, -0.013788947);
      initialSetup.setArmJointQs(RobotSide.LEFT, -1.5840142, -1.5527437, 1.3523333, -0.56639177);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -1.5347092, 1.1301415, 1.3192228, 0.55924076);
      initialSetup.setSpineJointQs(0.11051914, 0.03585892, 0.06934927);
      initialSetup.setRootJointPose(-0.16067322295514036, 0.092, 0.664, -0.19280733812854542, 0.6214101134020131, 0.1106679724037626, 0.7512838353175995);
      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newAllFoursGoingDownStepsPartial(HumanoidJointNameMap jointMap)
   {
      // , , , , -0.006579297, 2.136231E-4, 0.011883598, 100.0,
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, -0.20121925, 0.010199763, -2.0094466, 1.6984907, -0.9972875, -0.0010149578);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.28244376, -0.010222228, -2.3696282, 1.7484268, -0.7049025, 0.10972377);
      initialSetup.setArmJointQs(RobotSide.LEFT, -1.0121833, -0.92566574, 1.1405622, -1.0445135);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -0.8424884, 0.40751427, 1.1402806, 1.2626526);
      initialSetup.setSpineJointQs(-0.030311223, -0.13197364, -0.06452885);
      initialSetup.setRootJointPose(0.28508451545003705,
                                    0.002176475927506658,
                                    0.6792942943495437,
                                    0.056949298162032154,
                                    0.6050423985719524,
                                    0.08034868401159971,
                                    0.7900788329950086);
      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newStand(HumanoidJointNameMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      for (RobotSide robotSide : RobotSide.values)
      {
         initialSetup.setJoint(robotSide, LegJointName.HIP_ROLL, 0.0);
         initialSetup.setJoint(robotSide, LegJointName.HIP_PITCH, -0.6);
         initialSetup.setJoint(robotSide, LegJointName.KNEE_PITCH, 1.3);
         initialSetup.setJoint(robotSide, LegJointName.ANKLE_PITCH, -0.7);
         initialSetup.setJoint(robotSide, LegJointName.ANKLE_ROLL, 0.0);

         initialSetup.setJoint(robotSide, ArmJointName.SHOULDER_ROLL, robotSide.negateIfRightSide(-1.2));
         initialSetup.setJoint(robotSide, ArmJointName.SHOULDER_PITCH, -0.2);
         initialSetup.setJoint(robotSide, ArmJointName.ELBOW_PITCH, robotSide.negateIfRightSide(-1.5));
         initialSetup.setJoint(robotSide, ArmJointName.ELBOW_ROLL, 1.3);
      }

      initialSetup.getRootJointPosition().setZ(0.995);
      return initialSetup;
   }
}