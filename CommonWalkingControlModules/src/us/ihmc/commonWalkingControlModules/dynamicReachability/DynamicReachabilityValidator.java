package us.ihmc.commonWalkingControlModules.dynamicReachability;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlanner;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class DynamicReachabilityValidator
{
   private final ICPPlanner icpPlanner;
   private final FullHumanoidRobotModel fullRobotModel;

   private final ReferenceFrame centerOfMassFrame;
   private final FramePoint centerOfMassPosition = new FramePoint();

   private final SideDependentList<FrameVector> centerOfMassOffsetFromHips = new SideDependentList<>();
   private final SideDependentList<FrameVector> soleFrameOffsetFromAnkles = new SideDependentList<>();

   public DynamicReachabilityValidator(ICPPlanner icpPlanner, FullHumanoidRobotModel fullRobotModel, ReferenceFrame centerOfMassFrame)
   {
      this.icpPlanner = icpPlanner;
      this.centerOfMassFrame = centerOfMassFrame;

      this.fullRobotModel = fullRobotModel;

      for (RobotSide side : RobotSide.values)
      {
         centerOfMassOffsetFromHips.put(side, new FrameVector());
         soleFrameOffsetFromAnkles.put(side, new FrameVector());
      }
   }

   public void checkReachabilityOfStep(Footstep footstep)
   {
      RobotSide supportSide = footstep.getRobotSide().getOppositeSide();

      updateOffsets();

      double requiredStanceLegLength = computeRequiredStanceLegLength(supportSide);
      double requiredSwingLegLength = computeRequiredSwingLegLength(footstep);
   }

   private final FramePoint tempHipPoint = new FramePoint();
   private final FramePoint tempAnklePoint = new FramePoint();
   private void updateOffsets()
   {
      centerOfMassPosition.setToZero(centerOfMassFrame);

      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJoint hipPitchJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_PITCH);
         tempHipPoint.setToZero(hipPitchJoint.getFrameBeforeJoint());
         tempHipPoint.changeFrame(centerOfMassFrame);

         FrameVector centerOfMassOffsetFromHip = centerOfMassOffsetFromHips.get(robotSide);
         centerOfMassOffsetFromHip.setToZero(centerOfMassFrame);
         centerOfMassOffsetFromHip.set(centerOfMassPosition);
         centerOfMassOffsetFromHip.sub(tempHipPoint);

         ReferenceFrame soleFrame = fullRobotModel.getSoleFrame(robotSide);
         OneDoFJoint anklePitchJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_PITCH);
         tempAnklePoint.setToZero(anklePitchJoint.getFrameAfterJoint());
         tempAnklePoint.changeFrame(soleFrame);

         FrameVector soleFrameOffsetFromAnkle = soleFrameOffsetFromAnkles.get(robotSide);
         soleFrameOffsetFromAnkle.setToZero(soleFrame);
         soleFrameOffsetFromAnkle.sub(tempAnklePoint);
      }
   }

   private double computeRequiredStanceLegLength(RobotSide supportSide)
   {
      OneDoFJoint supportAnklePitchJoint = fullRobotModel.getLegJoint(supportSide, LegJointName.ANKLE_PITCH);
      tempAnklePoint.setToZero(supportAnklePitchJoint.getFrameAfterJoint());
      tempAnklePoint.changeFrame(centerOfMassFrame);

      icpPlanner.getFinalDesiredCenterOfMassPosition(tempHipPoint);
      tempHipPoint.changeFrame(centerOfMassFrame);
      tempHipPoint.sub(centerOfMassOffsetFromHips.get(supportSide));

      return tempHipPoint.distance(tempAnklePoint);
   }

   private double computeRequiredSwingLegLength(Footstep nextFootstep)
   {
      RobotSide swingSide = nextFootstep.getRobotSide();

      ReferenceFrame upcomingSoleFrame = nextFootstep.getSoleReferenceFrame();
      tempAnklePoint.setToZero(upcomingSoleFrame);
      tempAnklePoint.changeFrame(fullRobotModel.getSoleFrame(swingSide));
      tempAnklePoint.add(soleFrameOffsetFromAnkles.get(swingSide));
      tempAnklePoint.changeFrame(centerOfMassFrame);

      icpPlanner.getFinalDesiredCenterOfMassPosition(tempHipPoint);
      tempHipPoint.changeFrame(centerOfMassFrame);
      tempHipPoint.sub(centerOfMassOffsetFromHips.get(swingSide));

      return tempHipPoint.distance(tempAnklePoint);
   }
}
