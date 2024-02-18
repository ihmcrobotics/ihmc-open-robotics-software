package us.ihmc.avatar.inverseKinematics;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Used for finding IK solutions for various sets of end effectors.
 * Can solve:
 * - Chest to one hand
 * - Chest to both hands
 * - Pelvis to one hand
 * - Pelvis to both hands
 * - Pelvis to one foot
 * - Pelvis to two feet
 *
 * For example, solving chest to one arm, pelvis to one
 *
 * Uses the WholeBodyControllerCore for solving.
 */
public class HumanoidIKSolver
{
   /** The gains of the solver depend on this dt, it shouldn't change based on the actual thread scheduled period. */
   public static final double CONTROL_DT = 0.001;
   public static final double GRAVITY = 9.81;
   public static final double GOOD_QUALITY_MAX = 1.0;
   private static final int INVERSE_KINEMATICS_CALCULATIONS_PER_UPDATE = 70;
   public static final double DEFAULT_POSITION_GAIN = 1200.0;
   public static final double DEFAULT_POSITION_WEIGHT = 20.0;
   public static final double DEFAULT_ORIENTATION_GAIN = 100.0;
   public static final double DEFAULT_ORIENTATION_WEIGHT = 1.0;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final HumanoidJointNameMap jointNameMap;
   private final FullHumanoidRobotModel sourceFullRobotModel;

   public HumanoidIKSolver(HumanoidJointNameMap jointNameMap, FullHumanoidRobotModel sourceFullRobotModel)
   {
      this.jointNameMap = jointNameMap;
      this.sourceFullRobotModel = sourceFullRobotModel;
   }

   public void buildSolver(boolean controlLeftHand,
                           boolean includeRightHand,
                           boolean includePelvis,
                           boolean includeLeftFoot,
                           boolean includeRightFoot)
   {

   }
}
