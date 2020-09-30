package us.ihmc.valkyrie;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class ValkyrieInitialSetupFactories
{
   private static final double halfPi = 0.5 * Math.PI;

   public static ValkyrieMutableInitialSetup newCrawl1(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.5, 0.0, -1.0, 1.2, 0.2, 0.0);
      initialSetup.setArmJointQs(-2.0, -1.2, 0.0, -1.2);
      initialSetup.setRootJointPose(0.0, 0.0, 0.39, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newCrawl2(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.25, 1.75, -0.8, 0.0);
      initialSetup.setArmJointQs(-1.5, -0.65, 1.5, -1.4);
      initialSetup.setSpineJointQs(0.0, 0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.56, 0.0, 0.4, 0.0, 0.5);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newKneel1(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -0.3, 1.9, 0.8, 0.0);
      initialSetup.setArmJointQs(0.7, -1.2, 0.0, -1.7);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.74, 0.0, 0.0, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newKneel2(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, 0.75, 2.05, -0.8, 0.0);
      initialSetup.setArmJointQs(0.7, -1.2, 0.0, -1.7);
      initialSetup.setSpineJointQs(0.0, 0.4, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.64, 0.0, -0.4, 0.0, 0.8);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newSit1(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.9, 1.525, 0.775, 0.0);
      initialSetup.setArmJointQs(1.1, -1.0, 1.1, -0.9);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.33, 0.0, -0.2, 0.0, 1.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newSit2(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.2, 1.525, 0.775, 0.0);
      initialSetup.setArmJointQs(1.4, 0.2, 1.5, -1.8);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.23, 0.0, -0.6, 0.0, 1.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newSit3(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -0.875, 1.425, 0.8, 0.0);
      initialSetup.setArmJointQs(-0.5, 1.266, -3.1, -2.0);
      initialSetup.setSpineJointQs(0.0, 0.4, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.17, 0.0, -0.8, 0.0, 1.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newQuadToKneel1(DRCRobotJointMap jointMap)
   {
      // TODO Needs joint limits to be removed
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.15, 2.7, -0.8, 0.0);
      initialSetup.setArmJointQs(-1.35, -1.519, 1.2, -0.2);
      initialSetup.setSpineJointQs(0.0, 0.9, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.6, 0.0, 0.35, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newQuadToKneel2(DRCRobotJointMap jointMap)
   {
      // TODO Needs joint limits to be removed
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.15, 2.4, 0.8, 0.0);
      initialSetup.setArmJointQs(-1.35, -1.519, 1.2, -0.2);
      initialSetup.setSpineJointQs(0.0, 0.9, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.61, 0.0, 0.35, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newQuadToKneel3(DRCRobotJointMap jointMap)
   {
      // TODO Needs joint limits to be removed
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.5, -1.15, 2.375, 0.8, 0.0);
      initialSetup.setArmJointQs(-1.35, -1.519, 1.2, -0.2);
      initialSetup.setSpineJointQs(0.0, 0.8, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.565, 0.0, 0.35, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newQuadToKneel4(DRCRobotJointMap jointMap)
   {
      // TODO Needs joint limits to be removed
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.5, -1.15, 2.25, 0.8, 0.0);
      initialSetup.setArmJointQs(-1.35, -1.519, 1.2, -0.2);
      initialSetup.setSpineJointQs(0.0, 0.75, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.59, 0.0, 0.45, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newTriToKneel1(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newTriToKneel2(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newTriToKneel3(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newTriToKneel4(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newToKneel1(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newToKneel2(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newToKneel3(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newToKneel4(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newToKneel5(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.5, -0.75, 2.05, 0.8, 0.0);
      initialSetup.setArmJointQs(-0.8, -1.519, 1.2, -0.2);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -0.35, 1.3, 0.0, 0.2);
      initialSetup.setSpineJointQs(0.0, 0.4, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.635, 0.0, 0.30, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingA1(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newRollingA2(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newRollingA3(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newRollingA4(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newRollingA5(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newRollingA6(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newRollingA7(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newLieDown1(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -0.325, 1.1, 0.8, 0.0);
      initialSetup.setArmJointQs(0.0, -1.5, 0.0, -2.0);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.245, 0.707, 0.0, 0.707, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newLieDown2(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -0.325, 1.1, 0.8, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, halfPi, 0.4, halfPi, -1.75);
      initialSetup.setArmJointQs(RobotSide.RIGHT, 0.0, 1.5, 0.0, 2.0);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.245, 0.707, 0.0, 0.707, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newRollingB1(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newRollingB2(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newRollingB3(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newRollingB4(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newRollingB5(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newRollingB6(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newRollingB7(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newRollingB8(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newRollingB9(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newRollingB10(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newRollingB11(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newRollingB12(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newRollingB13(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newRollingB14(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newRollingB15(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newJSCSequenceA1(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -0.25, 0.5, -0.25, 0.0);
      initialSetup.setArmJointQs(0.7, -1.5, 0.0, -2.0);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.185, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA2a(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -0.5, 0.75, -0.25, 0.0);
      initialSetup.setArmJointQs(0.45, -1.5, 0.0, -2.0);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.285, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA2b(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -0.5, 0.75, -0.25, 0.0);
      initialSetup.setArmJointQs(-1.55, 0.35, 1.5, -2.0);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.285, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA2c(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -0.5, 0.75, -0.25, 0.0);
      initialSetup.setArmJointQs(-0.25, 1.266, 0.0, -2.0);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.285, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA3(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.3, 1.625, -0.65, 0.0);
      initialSetup.setArmJointQs(-2.25, -1.334, 1.5, -0.5);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.485, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA4(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.3, 1.625, -0.65, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -1.45, -0.8, 1.5, -2.0);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -2.4, 1.45, 1.5, 0.5);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.485, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA5a(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.3, 1.625, -0.65, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -1.45, -0.3, 1.5, -1.7);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -2.25, 1.334, 1.5, 0.5);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.485, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA5b(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.3, 1.625, -0.65, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -0.25, -1.4, 0.2, -1.7);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -2.25, 1.334, 1.5, 0.5);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.485, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA5c(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.3, 1.625, -0.65, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -2.05, -1.4, 1.5, -1.7);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -2.75, 1.334, 1.5, 0.5);
      initialSetup.setSpineJointQs(0.0, 0.2, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.485, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA6a(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.6, 0.0, -1.5, 1.925, -0.65, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -1.5, -0.6, 1.1, -2.0);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -2.0, 0.7, 0.7, 1.7);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.4, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA6b(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.6, 0.0, -1.5, 1.925, -0.65, 0.0);
      initialSetup.setArmJointQs(RobotSide.LEFT, -1.5, -1.5, 1.5, -1.7);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -2.0, 0.7, 0.7, 1.7);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.4, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA7a(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.6, 0.0, -1.5, 1.925, -0.65, 0.0);
      initialSetup.setArmJointQs(-1.5, -0.6, 1.1, -2.0);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.4, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA7b(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.6, 0.25, -1.8, 2.05, -0.8, 0.0);
      initialSetup.setArmJointQs(-1.8, -0.775, 1.1, -1.7);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.4, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA7c(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.45, 0.55, -1.25, 2.05, -0.8, 0.0);
      initialSetup.setArmJointQs(-1.55, -1.519, 1.1, -0.2);
      initialSetup.setSpineJointQs(0.0, 0.075, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.5, 0.0, 1.0, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA7d(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.45, 0.55, -0.9, 2.05, -0.8, 0.3);
      initialSetup.setArmJointQs(-1.45, -1.519, 1.1, -0.2);
      initialSetup.setSpineJointQs(0.0, 0.4, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.53, 0.0, 0.7, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA7e(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -0.875, 2.05, -0.8, 0.0);
      initialSetup.setArmJointQs(-1.35, -1.519, 1.2, -0.2);
      initialSetup.setSpineJointQs(0.0, 0.666, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.67, 0.0, 0.75, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA10(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -0.15, 2.05, -0.8, 0.0);
      initialSetup.setArmJointQs(0.0, -1.3, 0.4, -0.7);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.74, 0.0, 0.0, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA11(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newJSCSequenceA12(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newJSCSequenceA13a(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newJSCSequenceA13b(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.0, 0.0, -0.15, 2.05, -0.8, 0.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, 0.0, 0.0, -1.55, 1.35, 0.2, 0.0);
      initialSetup.setArmJointQs(0.0, -1.2, 0.4, -0.7);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.735, 0.0, 0.0, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA15a(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.0, 0.0, 0.05, 1.6, -0.5, 0.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, 0.0, 0.0, -1.31, 1.35, -0.04, 0.0);
      initialSetup.setArmJointQs(0.0, -1.2, 0.4, -0.7);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.85, 0.0, 0.0, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA15b(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.0, 0.0, -0.25, 1.59, -0.5, 0.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, 0.0, 0.0, -1.62, 1.35, -0.03, 0.0);
      initialSetup.setArmJointQs(0.0, -1.2, 0.4, -0.7);
      initialSetup.setSpineJointQs(0.0, 0.0, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.835, 0.0, 0.3, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceA16(DRCRobotJointMap jointMap)
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

   public static ValkyrieMutableInitialSetup newJSCSequenceC5(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, -0.4, 0.0, -1.3, 1.625, -0.65, 0.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.3, 0.0, -1.3, 1.625, -0.65, 0.0);
      initialSetup.setArmJointQs(-2.25, -1.334, 1.5, -0.5);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.485, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newJSCSequenceC6(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(RobotSide.LEFT, -0.4, 0.0, -1.3, 1.625, -0.65, 0.0);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.3, 0.0, -1.3, 1.625, -0.65, 0.0);
      initialSetup.setArmJointQs(-2.25, -1.334, 1.5, -0.5);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.485, 0.0, halfPi, 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newDownwardDog1(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setLegJointQs(0.0, 0.0, -1.1, 0.925, -0.35, 0.0);
      initialSetup.setArmJointQs(-2.0, -0.8, 1.5, -1.0);
      initialSetup.setSpineJointQs(0.0, -0.1, 0.0);
      initialSetup.setRootJointPose(0.0, 0.0, 0.59, 0.0, 0.5 * Math.PI + Math.toRadians(4.0), 0.0);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newDownwardDog2(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setRootJointPose(0.123, -0.017,  0.555 , 0.007,  0.666,  0.003,  0.746 );
      initialSetup.setArmJointQs(RobotSide.LEFT, -1.8023768663406372, -0.6600321531295776, 1.6847652196884155, -1.1806418895721436);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -1.808201551437378, 0.6778274774551392, 1.6804845333099365, 1.2342568635940552);
      initialSetup.setSpineJointQs(-0.010324880480766296, -0.04249017685651779, -0.0017525835428386927);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.005049135535955429, 0.003498799167573452, -0.7198069095611572, 0.7784734964370728, -0.13401946425437927, 9.574534487910569E-4);
      initialSetup.setLegJointQs(RobotSide.RIGHT, 0.003915309440344572, 0.004298515152186155, -0.7215321660041809, 0.7923046946525574, -0.15062116086483002, -5.203679320402443E-4);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newDownwardDog3(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setRootJointPose(-0.201,  0.008,  0.569 ,  0.001,  0.874, -0.005,  0.486 );
      initialSetup.setArmJointQs(RobotSide.LEFT, -2.6978771686553955, -0.883269190788269, 1.159483790397644, -0.9043288230895996);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -2.749500274658203, 0.8935198783874512, 1.1485799551010132, 0.8796213269233704);
      initialSetup.setSpineJointQs(0.04522188752889633, -0.12999999523162842, -3.2741372706368566E-5);
      initialSetup.setLegJointQs(RobotSide.LEFT, -0.00942451786249876, 6.895464612171054E-4, -1.7987622022628784, 0.9737935066223145, -0.5250318050384521, -7.837577722966671E-4);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.009312592446804047, -0.0015030786162242293, -1.7993817329406738, 0.9674890637397766, -0.5198957324028015, 3.357457753736526E-4);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newDownwardDog4(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setRootJointPose(-0.137,  0.081,  0.595 ,  -0.006,  0.811, -0.070,  0.580 );
      initialSetup.setArmJointQs(RobotSide.LEFT, -2.278846263885498, -1.0112913846969604, 1.3825160264968872, -0.8207610249519348);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -2.529759407043457, 0.9063995480537415, 1.2799235582351685, 0.7467198967933655);
      initialSetup.setSpineJointQs(0.3059770464897156, -0.12999999523162842, 0.004892634693533182);
      initialSetup.setLegJointQs(RobotSide.LEFT, -0.07278738170862198, -0.012698269449174404, -1.5313224792480469, 1.0129259824752808, -0.5027531385421753, -0.002264829818159342);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.06796674430370331, -0.01171040814369917, -1.542181134223938, 0.9300077557563782, -0.43473777174949646, -0.0013247140450403094);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newDownwardDog5(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setRootJointPose(-0.018,  0.024,  0.519,  -0.010,  0.739, -0.023,  0.673 );
      initialSetup.setArmJointQs(RobotSide.LEFT, -2.0328853130340576, -0.6315305829048157, 1.4743789434432983, -1.2933447360992432);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -2.055750846862793, 0.5933356881141663, 1.4505459070205688, 1.26048743724823);
      initialSetup.setSpineJointQs(0.06431251764297485, -0.12999999523162842, 0.001056957757100463);
      initialSetup.setLegJointQs(RobotSide.LEFT, -0.013619190081954002, -0.007051494438201189, -1.1692461967468262, 1.171615719795227, -0.5346041321754456, -0.0011694349814206362);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.014346427284181118, -0.005746279843151569, -1.1726155281066895, 1.142091155052185, -0.5201836824417114, 2.2127181000541896E-4);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newDownwardDog6(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setRootJointPose(-0.009,  0.020,  0.615 ,  -0.007,  0.700, -0.018,  0.714 );
      initialSetup.setArmJointQs(RobotSide.LEFT, -1.864648461341858, -1.1557345390319824, 1.5107604265213013, -0.4732952117919922);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -1.8913699388504028, 1.1152821779251099, 1.5061705112457275, 0.44274628162384033);
      initialSetup.setSpineJointQs(0.05381539463996887, -0.11364592611789703, 0.0031673461198806763);
      initialSetup.setLegJointQs(RobotSide.LEFT, -0.012595400214195251, -0.006910591386258602, -1.0284287929534912, 0.9588997960090637, -0.36863067746162415, -4.8766008694656193E-4);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.012869426980614662, -0.006223239004611969, -1.0329811573028564, 0.9392527341842651, -0.3519395887851715, -3.7425383925437927E-4);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newDownwardDog7(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setRootJointPose(0.006, -0.128,  0.582,  0.048,  0.702,  0.005,  0.711);
      initialSetup.setArmJointQs(RobotSide.LEFT, -1.8425549268722534, -0.968360185623169, 1.5199047327041626, -0.3387618362903595);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -1.9097520112991333, 1.0771688222885132, 1.5249048471450806, 0.9470992088317871);
      initialSetup.setSpineJointQs(-0.0810769721865654, -0.12153621017932892, -0.00599761214107275);
      initialSetup.setLegJointQs(RobotSide.LEFT, 0.030850788578391075, 0.016274211928248405, -0.9926846027374268, 0.9103361368179321, -0.31820449233055115, 0.001071992446668446);
      initialSetup.setLegJointQs(RobotSide.RIGHT, 0.03241691365838051, 0.018180493265390396, -1.0288697481155396, 1.0234824419021606, -0.42063140869140625, 0.0013632606714963913);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newDownwardDog8(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setRootJointPose( 0.000,  0.113,  0.594, -0.042,  0.700, -0.032,  0.712 );
      initialSetup.setArmJointQs(RobotSide.LEFT, -1.883097767829895, -1.1128853559494019, 1.52692711353302, -0.8211992383003235);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -1.8842524290084839, 0.9998580813407898, 1.5112547874450684, 0.3537876605987549);
      initialSetup.setSpineJointQs(0.1375928819179535, -0.11772836744785309, 0.008991878479719162);
      initialSetup.setLegJointQs(RobotSide.LEFT, -0.041541725397109985, -0.022845974192023277, -1.0262222290039062, 1.0043795108795166, -0.40567538142204285, -0.0015900044236332178);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.039717718958854675, -0.020319132134318352, -1.011014699935913, 0.9027884006500244, -0.31383952498435974, -0.0013461336493492126);

      return initialSetup;
   }

   public static ValkyrieMutableInitialSetup newDownwardDog9(DRCRobotJointMap jointMap)
   {
      ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);
      initialSetup.setRootJointPose( -0.173,  0.030,  0.564, -0.011,  0.756, -0.010,  0.654);
      initialSetup.setArmJointQs(RobotSide.LEFT, -2.262970447540283, -1.2187447547912598, 1.4327235221862793, -0.41919344663619995);
      initialSetup.setArmJointQs(RobotSide.RIGHT, -2.2887258529663086, 1.1344482898712158, 1.4381821155548096, 0.3499889671802521);
      initialSetup.setSpineJointQs(0.059888582676649094, -0.12999999523162842, -0.0015916757984086871);
      initialSetup.setLegJointQs(RobotSide.LEFT, -0.012916137464344501, -0.004163815174251795, -1.5134780406951904, 1.4132899045944214, -0.6693117022514343, -0.0017147266771644354);
      initialSetup.setLegJointQs(RobotSide.RIGHT, -0.01341082714498043, -0.003379943547770381, -1.502829670906067, 1.3760894536972046, -0.6603738069534302, 8.854138432070613E-4);

      return initialSetup;
   }

   public static void main(String[] args)
   {
      String input =
            "leftHipYaw (-0.00942451786249876), leftHipRoll (6.895464612171054E-4), leftHipPitch (-1.7987622022628784), leftKneePitch (0.9737935066223145), leftAnklePitch (-0.5250318050384521), leftAnkleRoll (-7.837577722966671E-4), rightHipYaw (-0.009312592446804047), rightHipRoll (-0.0015030786162242293), rightHipPitch (-1.7993817329406738), rightKneePitch (0.9674890637397766), rightAnklePitch (-0.5198957324028015), rightAnkleRoll (3.357457753736526E-4), torsoYaw (0.04522188752889633), torsoPitch (-0.12999999523162842), torsoRoll (-3.2741372706368566E-5), leftShoulderPitch (-2.6978771686553955), leftShoulderRoll (-0.883269190788269), leftShoulderYaw (1.159483790397644), leftElbowPitch (-0.9043288230895996), leftForearmYaw (0.08031015843153), leftWristRoll (0.0050920145586133), leftWristPitch (0.006010639481246471), lowerNeckPitch (0.0), neckYaw (0.0), upperNeckPitch (0.0), hokuyo_joint (0.0), rightShoulderPitch (-2.749500274658203), rightShoulderRoll (0.8935198783874512), rightShoulderYaw (1.1485799551010132), rightElbowPitch (0.8796213269233704), rightForearmYaw (0.09620540589094162), rightWristRoll (-0.005838884972035885), rightWristPitch (-0.007548563182353973)\n";
      String[] subStrings = input.split(",");
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

      OneDoFJointBasics[] joints = fullRobotModel.getControllableOneDoFJoints();
      for (int i = 0; i < joints.length; i++)
      {
         double jointAngle = Double.parseDouble(subStrings[i].split("\\(")[1].replace(")", ""));
         joints[i].setQ(jointAngle);
      }

      System.out.print("initialSetup.setArmJointQs(RobotSide.LEFT, ");
      System.out.print(fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.SHOULDER_PITCH).getQ() + ", ");
      System.out.print(fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.SHOULDER_ROLL).getQ() + ", ");
      System.out.print(fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.SHOULDER_YAW).getQ() + ", ");
      System.out.print(fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.ELBOW_PITCH).getQ());
      System.out.println(");");

      System.out.print("initialSetup.setArmJointQs(RobotSide.RIGHT, ");
      System.out.print(fullRobotModel.getArmJoint(RobotSide.RIGHT, ArmJointName.SHOULDER_PITCH).getQ() + ", ");
      System.out.print(fullRobotModel.getArmJoint(RobotSide.RIGHT, ArmJointName.SHOULDER_ROLL).getQ() + ", ");
      System.out.print(fullRobotModel.getArmJoint(RobotSide.RIGHT, ArmJointName.SHOULDER_YAW).getQ() + ", ");
      System.out.print(fullRobotModel.getArmJoint(RobotSide.RIGHT, ArmJointName.ELBOW_PITCH).getQ());
      System.out.println(");");

      System.out.print("initialSetup.setSpineJointQs(");
      System.out.print(fullRobotModel.getSpineJoint(SpineJointName.SPINE_YAW).getQ() + ", ");
      System.out.print(fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH).getQ() + ", ");
      System.out.print(fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL).getQ());
      System.out.println(");");

      System.out.print("initialSetup.setLegJointQs(RobotSide.LEFT, ");
      System.out.print(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.HIP_YAW).getQ() + ", ");
      System.out.print(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.HIP_ROLL).getQ() + ", ");
      System.out.print(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.HIP_PITCH).getQ() + ", ");
      System.out.print(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getQ() + ", ");
      System.out.print(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.ANKLE_PITCH).getQ() + ", ");
      System.out.print(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.ANKLE_ROLL).getQ());
      System.out.println(");");

      System.out.print("initialSetup.setLegJointQs(RobotSide.RIGHT, ");
      System.out.print(fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.HIP_YAW).getQ() + ", ");
      System.out.print(fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.HIP_ROLL).getQ() + ", ");
      System.out.print(fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.HIP_PITCH).getQ() + ", ");
      System.out.print(fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.KNEE_PITCH).getQ() + ", ");
      System.out.print(fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.ANKLE_PITCH).getQ() + ", ");
      System.out.print(fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.ANKLE_ROLL).getQ());
      System.out.println(");");
   }
}
