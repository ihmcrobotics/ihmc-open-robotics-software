package us.ihmc.valkyrie.stepReachability;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.reachabilityMap.footstep.HumanoidStepReachabilityCalculator;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.OneDoFJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.ValkyrieSimulationCollisionModel;

import java.util.HashMap;
import java.util.Map;

public class ValkyrieStepReachabilityCalculator extends HumanoidStepReachabilityCalculator
{
   /**
    * Note on naming convention:
    * {Joint}LimitReduction means reduce the range by that amount.
    * So if jointA normally can be between 0 rad and 1 rad and has a reduction
    * of 0.2, it now can only go 0.1 rad to 0.9 rad
    */
   private static final double kneePitchLimitReduction = 0.3;
   private static final double anklePitchLimitReduction = 0.3;
   private static final double ankleRollLimitReduction = 0.3;
   private static final double hipPitchLimitReduction = 0.3;
   private static final double hipRollLimitReduction = 0.3;
   private static final double hipYawLimitReduction = 0.3;

   public ValkyrieStepReachabilityCalculator() throws Exception
   {
      super();
   }

   public static void main(String[] args) throws Exception
   {
      new ValkyrieStepReachabilityCalculator();
   }

   @Override
   protected DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS);
   }

   @Override
   protected void imposeJointLimitRestrictions(DRCRobotModel robotModel)
   {
      HumanoidJointNameMap jointMap = robotModel.getJointMap();
      RobotDescription robotDescription = robotModel.getRobotDescription();

      for (RobotSide robotSide : RobotSide.values)
      {
         String kneePitchName = jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH);
         String anklePitchName = jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH);
         String ankleRollName = jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL);
         String hipPitchName = jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH);
         String hipRollName = jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL);
         String hipYawName = jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW);

         OneDoFJointDescription kneePitch = (OneDoFJointDescription) robotDescription.getJointDescription(kneePitchName);
         OneDoFJointDescription anklePitch = (OneDoFJointDescription) robotDescription.getJointDescription(anklePitchName);
         OneDoFJointDescription ankleRoll = (OneDoFJointDescription) robotDescription.getJointDescription(ankleRollName);
         OneDoFJointDescription hipPitch = (OneDoFJointDescription) robotDescription.getJointDescription(hipPitchName);
         OneDoFJointDescription hipRoll = (OneDoFJointDescription) robotDescription.getJointDescription(hipRollName);
         OneDoFJointDescription hipYaw = (OneDoFJointDescription) robotDescription.getJointDescription(hipYawName);

         restrictJointLimits(kneePitch, kneePitchLimitReduction);
         restrictJointLimits(anklePitch, anklePitchLimitReduction);
         restrictJointLimits(ankleRoll, ankleRollLimitReduction);
         restrictJointLimits(hipPitch, hipPitchLimitReduction);
         restrictJointLimits(hipRoll, hipRollLimitReduction);
         restrictJointLimits(hipYaw, hipYawLimitReduction);
      }
   }

   private static void restrictJointLimits(OneDoFJointDescription jointDescription, double jointLimitReduction)
   {
      double nominalMin = jointDescription.getLowerLimit();
      double nominalMax = jointDescription.getUpperLimit();
      double reductionAmount = (nominalMax - nominalMin) * jointLimitReduction;

      double modifiedMin = nominalMin + 0.5 * reductionAmount;
      double modifiedMax = nominalMax - 0.5 * reductionAmount;
      jointDescription.setLimitStops(modifiedMin, modifiedMax, jointDescription.getLimitStopParameters()[2], jointDescription.getLimitStopParameters()[3]);
   }

   @Override
   protected RobotCollisionModel getRobotCollisionModel(HumanoidJointNameMap jointMap)
   {
      ValkyrieSimulationCollisionModel collisionModel = new ValkyrieSimulationCollisionModel(jointMap);
      collisionModel.setCollidableHelper(new CollidableHelper(), "robot", "ground");
      return collisionModel;
   }
}
