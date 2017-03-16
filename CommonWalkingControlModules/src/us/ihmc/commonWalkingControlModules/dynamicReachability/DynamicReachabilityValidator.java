package us.ihmc.commonWalkingControlModules.dynamicReachability;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlanner;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class DynamicReachabilityValidator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable minimumLegLength = new DoubleYoVariable("minimumLegLength", registry);
   private final DoubleYoVariable maximumLegLength = new DoubleYoVariable("maximumLegLength", registry);

   private final DoubleYoVariable maximumDesiredKneeBend = new DoubleYoVariable("maximumDesiredKneeBend", registry);

   private final BooleanYoVariable reachableWRTStanceFoot = new BooleanYoVariable("reachableWRTStanceFoot", registry);
   private final BooleanYoVariable reachableWRTFootstep = new BooleanYoVariable("reachableWRTFootstep", registry);


   private final SideDependentList<FrameVector> centerOfMassOffsetFromHips = new SideDependentList<>();
   private final SideDependentList<FrameVector> soleFrameOffsetFromAnkles = new SideDependentList<>();

   private final ICPPlanner icpPlanner;
   private final FullHumanoidRobotModel fullRobotModel;

   private Footstep nextFootstep;

   private final ReferenceFrame centerOfMassFrame;

   private final FramePoint centerOfMassPosition = new FramePoint();
   private final FramePoint tempHipPoint = new FramePoint();
   private final FramePoint tempAnklePoint = new FramePoint();
   private final FramePoint2d tempFinalCoM = new FramePoint2d();

   private final double thighLength;
   private final double shinLength;

   public DynamicReachabilityValidator(ICPPlanner icpPlanner, FullHumanoidRobotModel fullRobotModel, ReferenceFrame centerOfMassFrame,
         YoVariableRegistry parentRegistry)
   {
      this.icpPlanner = icpPlanner;
      this.fullRobotModel = fullRobotModel;
      this.centerOfMassFrame = centerOfMassFrame;

      maximumDesiredKneeBend.set(0.4);

      for (RobotSide side : RobotSide.values)
      {
         centerOfMassOffsetFromHips.put(side, new FrameVector());
         soleFrameOffsetFromAnkles.put(side, new FrameVector());
      }

      // compute leg lengths
      ReferenceFrame hipPitchFrame = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.HIP_PITCH).getFrameAfterJoint();
      tempHipPoint.setToZero(hipPitchFrame);
      FramePoint tempKneePoint = new FramePoint(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getFrameBeforeJoint());
      tempKneePoint.changeFrame(hipPitchFrame);

      thighLength = tempHipPoint.distance(tempKneePoint);

      ReferenceFrame kneePitchFrame = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getFrameAfterJoint();
      tempKneePoint.setToZero(kneePitchFrame);
      tempAnklePoint.setToZero(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.ANKLE_PITCH).getFrameBeforeJoint());
      tempAnklePoint.changeFrame(kneePitchFrame);

      shinLength = tempKneePoint.distance(tempAnklePoint);

      parentRegistry.addChild(registry);
   }

   public void setUpcomingFootstep(Footstep nextFootstep)
   {
      this.nextFootstep = nextFootstep;
   }

   /**
    * Checks whether the current footstep is reachable given the desired footstep timing.
    *
    * @return reachable or not
    */
   public boolean checkReachabilityOfStep()
   {
      RobotSide supportSide = nextFootstep.getRobotSide().getOppositeSide();

      updateOffsets();
      updateLegLengthLimits();

      double requiredStanceLegLength = computeRequiredStanceLegLength(supportSide);
      double requiredSwingLegLength = computeRequiredSwingLegLength(nextFootstep);

      boolean reachableWRTStanceFoot = MathTools.intervalContains(requiredStanceLegLength, minimumLegLength.getDoubleValue(), maximumLegLength.getDoubleValue());
      boolean reachableWRTFootstep = MathTools.intervalContains(requiredSwingLegLength, minimumLegLength.getDoubleValue(), maximumLegLength.getDoubleValue());

      this.reachableWRTStanceFoot.set(reachableWRTStanceFoot);
      this.reachableWRTFootstep.set(reachableWRTFootstep);

      return reachableWRTStanceFoot && reachableWRTFootstep;
   }

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

   private void updateLegLengthLimits()
   {
      this.maximumLegLength.set(thighLength + shinLength);

      double minimumLegLength = Math.pow(thighLength, 2.0) + Math.pow(shinLength, 2.0) +
            2 * thighLength * shinLength * Math.sin(maximumDesiredKneeBend.getDoubleValue());
      minimumLegLength = Math.sqrt(minimumLegLength);
      this.minimumLegLength.set(minimumLegLength);
   }

   private double computeRequiredStanceLegLength(RobotSide supportSide)
   {
      OneDoFJoint supportAnklePitchJoint = fullRobotModel.getLegJoint(supportSide, LegJointName.ANKLE_PITCH);
      tempAnklePoint.setToZero(supportAnklePitchJoint.getFrameAfterJoint());
      tempAnklePoint.changeFrame(centerOfMassFrame);

      icpPlanner.getFinalDesiredCenterOfMassPosition(tempFinalCoM); // // FIXME: 3/16/17  doesn't have the height correct
      // // FIXME: 3/16/17 include the height
      tempHipPoint.setXY(tempFinalCoM);
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

      icpPlanner.getFinalDesiredCenterOfMassPosition(tempFinalCoM); // // FIXME: 3/16/17  doesn't have the height correct
      // // FIXME: 3/16/17 include the height
      tempHipPoint.setXY(tempFinalCoM);
      tempHipPoint.changeFrame(centerOfMassFrame);
      tempHipPoint.sub(centerOfMassOffsetFromHips.get(swingSide));

      return tempHipPoint.distance(tempAnklePoint);
   }
}
