package us.ihmc.valkyrie.stepReachability;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.reachabilityMap.footstep.HumanoidStepReachabilityCalculator;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.valkyrie.ValkyrieFootstepPlannerCollisionModel;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieStepReachabilityCalculator extends HumanoidStepReachabilityCalculator
{
   /**
    * Note on naming convention: {Joint}LimitReduction means reduce the range by that amount. So if
    * jointA normally can be between 0 rad and 1 rad and has a reduction of 0.2, it now can only go 0.1
    * rad to 0.9 rad
    */
   private static final double kneePitchLimitReduction = 0.2;
   private static final double anklePitchLimitReduction = 0.2;
   private static final double ankleRollLimitReduction = 0.2;
   private static final double hipPitchLimitReduction = 0.2;
   private static final double hipRollLimitReduction = 0.2;
   private static final double hipYawLimitReduction = 0.2;

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
      RobotDefinition robotDefinition = robotModel.getRobotDefinition();

      for (RobotSide robotSide : RobotSide.values)
      {
         String kneePitchName = jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH);
         String anklePitchName = jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH);
         String ankleRollName = jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL);
         String hipPitchName = jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH);
         String hipRollName = jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL);
         String hipYawName = jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW);

         OneDoFJointDefinition kneePitch  = (OneDoFJointDefinition) robotDefinition.getJointDefinition(kneePitchName);
         OneDoFJointDefinition anklePitch = (OneDoFJointDefinition) robotDefinition.getJointDefinition(anklePitchName);
         OneDoFJointDefinition ankleRoll  = (OneDoFJointDefinition) robotDefinition.getJointDefinition(ankleRollName);
         OneDoFJointDefinition hipPitch   = (OneDoFJointDefinition) robotDefinition.getJointDefinition(hipPitchName);
         OneDoFJointDefinition hipRoll    = (OneDoFJointDefinition) robotDefinition.getJointDefinition(hipRollName);
         OneDoFJointDefinition hipYaw     = (OneDoFJointDefinition) robotDefinition.getJointDefinition(hipYawName);

         restrictJointLimits(kneePitch, kneePitchLimitReduction);
         restrictJointLimits(anklePitch, anklePitchLimitReduction);
         restrictJointLimits(ankleRoll, ankleRollLimitReduction);
         restrictJointLimits(hipPitch, hipPitchLimitReduction);
         restrictJointLimits(hipRoll, hipRollLimitReduction);
         restrictJointLimits(hipYaw, hipYawLimitReduction);
      }
   }

   private static void restrictJointLimits(OneDoFJointDefinition jointDefinition, double jointLimitReduction)
   {
      double nominalMin = jointDefinition.getPositionLowerLimit();
      double nominalMax = jointDefinition.getPositionUpperLimit();
      double reductionAmount = (nominalMax - nominalMin) * jointLimitReduction;

      double modifiedMin = nominalMin + 0.5 * reductionAmount;
      double modifiedMax = nominalMax - 0.5 * reductionAmount;
      jointDefinition.setPositionLimits(modifiedMin, modifiedMax);
   }

   @Override
   protected RobotCollisionModel getRobotCollisionModel(HumanoidJointNameMap jointMap)
   {
      return new ValkyrieFootstepPlannerCollisionModel(jointMap);
   }

   @Override
   protected String getResourcesDirectory()
   {
      return "ihmc-open-robotics-software/valkyrie/src/main/resources";
   }
}
