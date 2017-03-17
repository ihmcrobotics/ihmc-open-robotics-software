package us.ihmc.commonWalkingControlModules.dynamicReachability;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlanner;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.*;
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
   private final FramePoint2d tempPoint2d = new FramePoint2d();

   private final double thighLength;
   private final double shinLength;

   private final LineSegment1d stanceHeightLine = new LineSegment1d();
   private final LineSegment1d stepHeightLine = new LineSegment1d();

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

      computeHeightLineFromStance(supportSide);
      computeHeightLineFromStep(nextFootstep);

      this.reachableWRTStanceFoot.set(stanceHeightLine.length() > 0.0);
      this.reachableWRTFootstep.set(stepHeightLine.length() > 0.0);

      return stanceHeightLine.isOverlappingInclusive(stepHeightLine);
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

   private void computeHeightLineFromStance(RobotSide supportSide)
   {
      OneDoFJoint supportAnklePitchJoint = fullRobotModel.getLegJoint(supportSide, LegJointName.ANKLE_PITCH);
      tempAnklePoint.setToZero(supportAnklePitchJoint.getFrameAfterJoint());
      tempAnklePoint.changeFrame(centerOfMassFrame);
      tempAnklePoint.getFrameTuple2d(tempPoint2d);

      // get the hip location in XY
      FrameVector offsetFromHips = centerOfMassOffsetFromHips.get(supportSide);
      icpPlanner.getFinalDesiredCenterOfMassPosition(tempFinalCoM);
      tempFinalCoM.changeFrame(offsetFromHips.getReferenceFrame());
      tempFinalCoM.sub(offsetFromHips.getX(), offsetFromHips.getY());

      double planarDistance = tempFinalCoM.distance(tempPoint2d);

      double minimumHeight, maximumHeight;
      if (planarDistance <= minimumLegLength.getDoubleValue())
         minimumHeight = 0.0;
      else
         minimumHeight = Math.sqrt(Math.pow(minimumLegLength.getDoubleValue(), 2.0) - Math.pow(planarDistance, 2.0));
      if (planarDistance <= maximumLegLength.getDoubleValue())
         maximumHeight = 0.0;
      else
         maximumHeight = Math.sqrt(Math.pow(maximumLegLength.getDoubleValue(), 2.0) - Math.pow(planarDistance, 2.0));

      stanceHeightLine.set(minimumHeight, maximumHeight);
   }

   private void computeHeightLineFromStep(Footstep nextFootstep)
   {
      RobotSide swingSide = nextFootstep.getRobotSide();

      ReferenceFrame upcomingSoleFrame = nextFootstep.getSoleReferenceFrame();
      tempAnklePoint.setToZero(upcomingSoleFrame);
      tempAnklePoint.changeFrame(fullRobotModel.getSoleFrame(swingSide));
      tempAnklePoint.add(soleFrameOffsetFromAnkles.get(swingSide));
      tempAnklePoint.changeFrame(centerOfMassFrame);
      tempAnklePoint.getFrameTuple2d(tempPoint2d);

      // get the hip location in XY
      FrameVector offsetFromHips = centerOfMassOffsetFromHips.get(swingSide);
      icpPlanner.getFinalDesiredCenterOfMassPosition(tempFinalCoM);
      tempFinalCoM.changeFrame(offsetFromHips.getReferenceFrame());
      tempFinalCoM.sub(offsetFromHips.getX(), offsetFromHips.getY());

      double planarDistance = tempFinalCoM.distance(tempPoint2d);

      double minimumHeight, maximumHeight;
      if (planarDistance <= minimumLegLength.getDoubleValue())
         minimumHeight = 0.0;
      else
         minimumHeight = Math.sqrt(Math.pow(minimumLegLength.getDoubleValue(), 2.0) - Math.pow(planarDistance, 2.0));
      if (planarDistance <= maximumLegLength.getDoubleValue())
         maximumHeight = 0.0;
      else
         maximumHeight = Math.sqrt(Math.pow(maximumLegLength.getDoubleValue(), 2.0) - Math.pow(planarDistance, 2.0));

      stepHeightLine.set(minimumHeight, maximumHeight);
   }
}
