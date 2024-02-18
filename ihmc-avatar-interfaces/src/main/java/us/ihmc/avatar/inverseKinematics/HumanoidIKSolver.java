package us.ihmc.avatar.inverseKinematics;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
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
 *
 * This class won't deal with stance feet i.e. controlling bodies to stay in the same place.
 * That's for the user to handle.
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

   private RigidBodyBasics workRootBody;

   private final SideDependentList<HumanoidIKSolverControlledBody> hands = new SideDependentList<>();
   private HumanoidIKSolverControlledBody chest;
   private HumanoidIKSolverControlledBody pelvis;
   private final SideDependentList<HumanoidIKSolverControlledBody> feet = new SideDependentList<>();

   public HumanoidIKSolver(HumanoidJointNameMap jointNameMap, FullHumanoidRobotModel sourceFullRobotModel)
   {
      this.jointNameMap = jointNameMap;
      this.sourceFullRobotModel = sourceFullRobotModel;
   }

   public void buildSolver(boolean controlLeftHand,
                           boolean controlRightHand,
                           boolean controlChest,
                           boolean controlPelvis,
                           boolean controlLeftFoot,
                           boolean controlRightFoot,
                           boolean unlockChest)
   {
      // accept controlled bodies, source full robot model, root body
      // connect the dots, build list of oneDoFjoints
      // unlock chest
      // unlock pelvis uses feet in contact

      for (RobotSide side : RobotSide.values)
      {
         hands.clear();
         chest = null;
         pelvis = null;
         feet.clear();
      }

      // We need to determine
      // - Which is the root body
      // - The set of OneDoFJoints
      // - If we are solving with a controlled stance foot
      //   - Need to pass in root joint to WBCC toolbox
      // - If there is a foot and a hand
      //   - Also need to pass in root joint to WBCC toolbox?

      // We make assumptions that the user passed in a set of values that implies a kinematic chain
      boolean controllingAFoot = controlLeftFoot || controlRightFoot;
      boolean pelvisIsIncludedInSystem = controllingAFoot || unlockChest || controlPelvis;

      // It's always either pelvis or chest
      RigidBodyBasics sourceRootBody = pelvisIsIncludedInSystem ? sourceFullRobotModel.getPelvis() : sourceFullRobotModel.getChest();

      // We clone a detached chest and single arm for the WBCC to work with. We just want to find arm joint angles.
      workRootBody = MultiBodySystemMissingTools.getDetachedCopyOfSubtreeWithElevator(sourceRootBody);

      if (controlLeftHand)
         hands.set(RobotSide.LEFT, HumanoidIKSolverControlledBody.createHand(workRootBody, sourceFullRobotModel, jointNameMap, RobotSide.LEFT));
      if (controlRightHand)
         hands.set(RobotSide.RIGHT, HumanoidIKSolverControlledBody.createHand(workRootBody, sourceFullRobotModel, jointNameMap, RobotSide.RIGHT));


   }
}
