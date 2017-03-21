package us.ihmc.commonWalkingControlModules.dynamicReachability;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlanner;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class DynamicReachabilityCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable minimumLegLength = new DoubleYoVariable("minimumLegLength", registry);
   private final DoubleYoVariable maximumLegLength = new DoubleYoVariable("maximumLegLength", registry);

   private final DoubleYoVariable stanceMaximumLengthCheck = new DoubleYoVariable("stanceMaximumLengthCheck", registry);
   private final DoubleYoVariable stanceMinimumLengthCheck = new DoubleYoVariable("stanceMinimumLengthCheck", registry);
   private final DoubleYoVariable swingMaximumLengthCheck = new DoubleYoVariable("swingMaximumLengthCheck", registry);
   private final DoubleYoVariable swingMinimumLengthCheck = new DoubleYoVariable("swingMinimumLengthCheck", registry);

   private final DoubleYoVariable maximumDesiredKneeBend = new DoubleYoVariable("maximumDesiredKneeBend", registry);

   private final DoubleYoVariable stanceLegMinimumHeight = new DoubleYoVariable("stanceLegMinimumHeight", registry);
   private final DoubleYoVariable stanceLegMaximumHeight = new DoubleYoVariable("stanceLegMaximumHeight", registry);
   private final DoubleYoVariable swingLegMinimumHeight = new DoubleYoVariable("swingLegMinimumHeight", registry);
   private final DoubleYoVariable swingLegMaximumHeight = new DoubleYoVariable("swingLegMaximumHeight", registry);

   private final BooleanYoVariable reachableWRTStanceFoot = new BooleanYoVariable("reachableWRTStanceFoot", registry);
   private final BooleanYoVariable reachableWRTFootstep = new BooleanYoVariable("reachableWRTFootstep", registry);
   private final BooleanYoVariable isStepReachable = new BooleanYoVariable("isStepReachable", registry);

   private final SideDependentList<YoFramePoint> ankleLocations = new SideDependentList<>();
   private final SideDependentList<YoFramePoint> hipMinimumLocations = new SideDependentList<>();
   private final SideDependentList<YoFramePoint> hipMaximumLocations = new SideDependentList<>();

   private final YoFramePoint finalCoM;

   private final ICPPlanner icpPlanner;
   private final FullHumanoidRobotModel fullRobotModel;

   private Footstep nextFootstep;

   private final FramePoint tempHipPoint = new FramePoint();
   private final FramePoint tempAnklePoint = new FramePoint();

   private final FramePoint2d tempFinalCoM = new FramePoint2d();
   private final FramePoint2d tempPoint2d = new FramePoint2d();

   private final double thighLength;
   private final double shinLength;

   private final LineSegment1d stanceHeightLine = new LineSegment1d();
   private final LineSegment1d stepHeightLine = new LineSegment1d();

   private final ReferenceFrame predictedCoMFrame;
   private final TranslationReferenceFrame predictedPelvisFrame;
   private final SideDependentList<ReferenceFrame> predictedHipFrames = new SideDependentList<>();

   private final FramePoint predictedCoMPosition = new FramePoint();
   private final FrameOrientation predictedPelvisOrientation = new FrameOrientation();

   public DynamicReachabilityCalculator(ICPPlanner icpPlanner, FullHumanoidRobotModel fullRobotModel, ReferenceFrame centerOfMassFrame,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.icpPlanner = icpPlanner;
      this.fullRobotModel = fullRobotModel;

      maximumDesiredKneeBend.set(1.0);

      String listName = "Dynamic Reachability";
      for (RobotSide side : RobotSide.values)
      {
         YoFramePoint ankleLocation = new YoFramePoint(side.getShortLowerCaseName() + "PredictedAnklePoint", worldFrame, registry);
         YoFramePoint hipMaximumLocation = new YoFramePoint(side.getShortLowerCaseName() + "PredictedHipMaximumPoint", worldFrame, registry);
         YoFramePoint hipMinimumLocation = new YoFramePoint(side.getShortLowerCaseName() + "PredictedHipMinimumPoint", worldFrame, registry);
         ankleLocations.put(side, ankleLocation);
         hipMaximumLocations.put(side, hipMaximumLocation);
         hipMinimumLocations.put(side, hipMinimumLocation);

         YoGraphicPosition ankleLocationViz = new YoGraphicPosition(side.getSideNameFirstLetter() + "Predicted Ankle Point", ankleLocation, 0.05, YoAppearance.AliceBlue());
         YoGraphicPosition hipMaximumLocationViz = new YoGraphicPosition(side.getSideNameFirstLetter() + "Predicted Maximum Hip Point", hipMaximumLocation, 0.05, YoAppearance.ForestGreen());
         YoGraphicPosition hipMinimumLocationViz = new YoGraphicPosition(side.getSideNameFirstLetter() + "Predicted Minimum Hip Point", hipMinimumLocation, 0.05, YoAppearance.Blue());

         yoGraphicsListRegistry.registerYoGraphic(listName, ankleLocationViz);
         yoGraphicsListRegistry.registerYoGraphic(listName, hipMaximumLocationViz);
         yoGraphicsListRegistry.registerYoGraphic(listName, hipMinimumLocationViz);
      }

      finalCoM = new YoFramePoint("PredictedFinalCoM", worldFrame, registry);
      YoGraphicPosition finalCoMViz = new YoGraphicPosition("Predicted Final CoM", finalCoM, 0.01, YoAppearance.Black(), YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);
      yoGraphicsListRegistry.registerYoGraphic(listName, finalCoMViz);
      yoGraphicsListRegistry.registerArtifact(listName, finalCoMViz.createArtifact());

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

      ReferenceFrame pelvisFrame = fullRobotModel.getPelvis().getBodyFixedFrame();
      FramePoint pelvis = new FramePoint(pelvisFrame);
      FramePoint com = new FramePoint(centerOfMassFrame);
      pelvis.changeFrame(centerOfMassFrame);
      FrameVector translationToCoM = new FrameVector(centerOfMassFrame);
      translationToCoM.set(com);
      translationToCoM.sub(pelvis);
      translationToCoM.changeFrame(pelvisFrame);

      predictedCoMFrame = new ReferenceFrame("Predicted CoM Position", worldFrame, false, false, false)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            predictedCoMPosition.changeFrame(worldFrame);
            predictedPelvisOrientation.changeFrame(worldFrame);
            transformToParent.setTranslation(predictedCoMPosition.getPoint());
            transformToParent.setRotation(predictedPelvisOrientation.getQuaternion());
         }
      };

      predictedPelvisFrame = new TranslationReferenceFrame("Predicted Pelvis Frame", predictedCoMFrame);
      predictedPelvisFrame.updateTranslation(translationToCoM.getVector());

      for (RobotSide robotSide : RobotSide.values)
      {
         FrameVector translationToPelvis = new FrameVector(pelvisFrame);
         FramePoint pelvisCenter = new FramePoint(pelvisFrame);
         FramePoint hipJoint = new FramePoint(fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_PITCH).getFrameAfterJoint());
         hipJoint.changeFrame(pelvisFrame);
         translationToPelvis.set(hipJoint);
         translationToPelvis.sub(pelvisCenter);
         TranslationReferenceFrame predictedHipFrame = new TranslationReferenceFrame(robotSide.getShortLowerCaseName() + " Predicted Hip Frame", predictedPelvisFrame);
         predictedHipFrame.updateTranslation(translationToPelvis.getVector());
         predictedHipFrames.put(robotSide, predictedHipFrame);
      }

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

      updateFrames(nextFootstep);
      updateLegLengthLimits();

      computeHeightLineFromStance(supportSide);
      computeHeightLineFromStep(nextFootstep);

      this.reachableWRTStanceFoot.set(stanceHeightLine.length() > 0.0);
      this.reachableWRTFootstep.set(stepHeightLine.length() > 0.0);
      this.isStepReachable.set(stanceHeightLine.isOverlappingExclusive(stepHeightLine));

      return isStepReachable.getBooleanValue();
   }

   private final FrameOrientation stanceFootOrientation = new FrameOrientation();
   private final FrameOrientation footstepOrientation = new FrameOrientation();
   private void updateFrames(Footstep nextFootstep)
   {
      RobotSide stanceSide = nextFootstep.getRobotSide().getOppositeSide();

      icpPlanner.getFinalDesiredCenterOfMassPosition(tempFinalCoM);
      finalCoM.setXY(tempFinalCoM);
      predictedCoMPosition.setXY(tempFinalCoM);

      stanceFootOrientation.setToZero(fullRobotModel.getFoot(stanceSide).getBodyFixedFrame());
      nextFootstep.getOrientationIncludingFrame(footstepOrientation);

      ReferenceFrame pelvisFrame = fullRobotModel.getPelvis().getBodyFixedFrame();
      stanceFootOrientation.changeFrame(pelvisFrame);
      footstepOrientation.changeFrame(pelvisFrame);
      predictedPelvisOrientation.setToZero(pelvisFrame);
      predictedPelvisOrientation.interpolate(stanceFootOrientation, footstepOrientation, 0.5);

      predictedCoMFrame.update();
      predictedPelvisFrame.update();
      for (RobotSide robotSide : RobotSide.values)
         predictedHipFrames.get(robotSide).update();
   }

   private void updateLegLengthLimits()
   {
      this.maximumLegLength.set(thighLength + shinLength);

      double minimumLegLength = Math.pow(thighLength, 2.0) + Math.pow(shinLength, 2.0) +
            2 * thighLength * shinLength * Math.cos(maximumDesiredKneeBend.getDoubleValue());
      minimumLegLength = Math.sqrt(minimumLegLength);
      this.minimumLegLength.set(minimumLegLength);
   }

   private final FramePoint tempFinalCoM3D = new FramePoint();
   private void computeHeightLineFromStance(RobotSide supportSide)
   {
      OneDoFJoint supportAnklePitchJoint = fullRobotModel.getLegJoint(supportSide, LegJointName.ANKLE_PITCH);
      tempAnklePoint.setToZero(supportAnklePitchJoint.getFrameAfterJoint());
      tempAnklePoint.changeFrame(worldFrame);
      tempAnklePoint.getFrameTuple2d(tempPoint2d);

      // get the hip location in XY
      tempFinalCoM3D.setToZero(predictedHipFrames.get(supportSide));
      tempFinalCoM3D.changeFrame(worldFrame);
      tempFinalCoM.setByProjectionOntoXYPlaneIncludingFrame(tempFinalCoM3D);

      ankleLocations.get(supportSide).set(tempAnklePoint);

      tempFinalCoM.changeFrame(worldFrame);
      hipMaximumLocations.get(supportSide).setXY(tempFinalCoM);
      hipMinimumLocations.get(supportSide).setXY(tempFinalCoM);

      double planarDistance = tempFinalCoM.distance(tempPoint2d);

      double minimumHeight, maximumHeight;
      if (planarDistance >= minimumLegLength.getDoubleValue())
      {
         minimumHeight = 0.0;
      }
      else
      {
         minimumHeight = Math.sqrt(Math.pow(minimumLegLength.getDoubleValue(), 2.0) - Math.pow(planarDistance, 2.0));
         minimumHeight += tempAnklePoint.getZ();
      }
      if (planarDistance >= maximumLegLength.getDoubleValue())
      {
         maximumHeight = 0.0;
      }
      else
      {
         maximumHeight = Math.sqrt(Math.pow(maximumLegLength.getDoubleValue(), 2.0) - Math.pow(planarDistance, 2.0));
         maximumHeight += tempAnklePoint.getZ();
      }


      finalCoM.setZ(0.5 * (minimumHeight + maximumHeight));
      hipMaximumLocations.get(supportSide).setZ(maximumHeight);
      hipMinimumLocations.get(supportSide).setZ(minimumHeight);

      stanceLegMinimumHeight.set(minimumHeight);
      stanceLegMaximumHeight.set(maximumHeight);
      stanceHeightLine.set(minimumHeight, maximumHeight);

      stanceMaximumLengthCheck.set(hipMaximumLocations.get(supportSide).distance(tempAnklePoint));
      stanceMinimumLengthCheck.set(hipMinimumLocations.get(supportSide).distance(tempAnklePoint));
   }

   private void computeHeightLineFromStep(Footstep nextFootstep)
   {
      RobotSide swingSide = nextFootstep.getRobotSide();

      nextFootstep.getPositionIncludingFrame(tempAnklePoint);
      tempAnklePoint.changeFrame(worldFrame);
      tempAnklePoint.getFrameTuple2d(tempPoint2d);

      tempFinalCoM3D.setToZero(predictedHipFrames.get(swingSide));
      tempFinalCoM3D.changeFrame(worldFrame);
      tempFinalCoM.setByProjectionOntoXYPlaneIncludingFrame(tempFinalCoM3D);

      double planarDistance = tempFinalCoM.distance(tempPoint2d);

      tempFinalCoM.changeFrame(worldFrame);
      tempAnklePoint.changeFrame(worldFrame);
      ankleLocations.get(swingSide).set(tempAnklePoint);

      hipMaximumLocations.get(swingSide).setXY(tempFinalCoM);
      hipMinimumLocations.get(swingSide).setXY(tempFinalCoM);

      double minimumHeight, maximumHeight;
      if (planarDistance >= minimumLegLength.getDoubleValue())
      {
         minimumHeight = 0.0;
      }
      else
      {
         minimumHeight = Math.sqrt(Math.pow(minimumLegLength.getDoubleValue(), 2.0) - Math.pow(planarDistance, 2.0));
         minimumHeight += tempAnklePoint.getZ();
      }
      if (planarDistance >= maximumLegLength.getDoubleValue())
      {
         maximumHeight = 0.0;
      }
      else
      {
         maximumHeight = Math.sqrt(Math.pow(maximumLegLength.getDoubleValue(), 2.0) - Math.pow(planarDistance, 2.0));
         maximumHeight += tempAnklePoint.getZ();
      }

      hipMaximumLocations.get(swingSide).setZ(maximumHeight);
      hipMinimumLocations.get(swingSide).setZ(minimumHeight);

      swingLegMinimumHeight.set(minimumHeight);
      swingLegMaximumHeight.set(maximumHeight);
      stepHeightLine.set(minimumHeight, maximumHeight);

      swingMaximumLengthCheck.set(hipMaximumLocations.get(swingSide).distance(tempAnklePoint));
      swingMinimumLengthCheck.set(hipMinimumLocations.get(swingSide).distance(tempAnklePoint));
   }
}
