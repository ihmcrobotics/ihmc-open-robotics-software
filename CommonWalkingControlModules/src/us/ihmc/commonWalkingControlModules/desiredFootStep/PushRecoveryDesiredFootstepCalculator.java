package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.humanoidRobotics.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;


public class PushRecoveryDesiredFootstepCalculator implements DesiredFootstepCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("PushRecoveryDesiredStepLocationCalcualtor");

   private final CommonHumanoidReferenceFrames referenceFrames;
   private final CouplingRegistry couplingRegistry;

   private final SideDependentList<YoFramePoint> stepPositions = new SideDependentList<YoFramePoint>();
   private final SideDependentList<YoFrameOrientation> stepOrientations = new SideDependentList<YoFrameOrientation>();
   private final DoubleYoVariable stepDistance = new DoubleYoVariable("stepDistance", registry);

   private final SideDependentList<? extends ContactablePlaneBody> contactableBodies;

   public PushRecoveryDesiredFootstepCalculator(SideDependentList<? extends ContactablePlaneBody> contactableBodies,
           CommonHumanoidReferenceFrames referenceFrames, CouplingRegistry couplingRegistry, YoVariableRegistry parentRegistry)
   {
      this.contactableBodies = contactableBodies;
      this.referenceFrames = referenceFrames;
      this.couplingRegistry = couplingRegistry;

      for (RobotSide swingSide : RobotSide.values)
      {
         RobotSide supportSide = swingSide.getOppositeSide();
         ReferenceFrame ankleZUpFrame = referenceFrames.getAnkleZUpFrame(supportSide);
         stepPositions.put(swingSide, new YoFramePoint(swingSide.getCamelCaseNameForStartOfExpression() + "StepPosition", "", ankleZUpFrame, registry));
         stepOrientations.put(swingSide,
                              new YoFrameOrientation(swingSide.getCamelCaseNameForStartOfExpression() + "StepOrientation", "", ankleZUpFrame, registry));
      }

      parentRegistry.addChild(registry);
   }


   public void initializeDesiredFootstep(RobotSide supportLegSide)
   {
      computeInitialDesiredFootstep(supportLegSide);
   }
   
   public Footstep predictFootstepAfterDesiredFootstep(RobotSide supportLegSide, Footstep desiredFootstep)
   {
      return null;
   }

   public Footstep updateAndGetDesiredFootstep(RobotSide supportLegSide)
   {
//    double estimatedSwingTimeRemaining = couplingRegistry.getEstimatedSwingTimeRemaining();
//    if (estimatedSwingTimeRemaining > 0.1)
//    {
//       computeInitialDesiredFootstep(supportLegSide);
//    }

      RobotSide swingSide = supportLegSide.getOppositeSide();
      FramePose footstepPose = new FramePose(stepPositions.get(swingSide).getFramePointCopy(), stepOrientations.get(swingSide).getFrameOrientationCopy());
      ContactablePlaneBody foot = contactableBodies.get(swingSide);

      boolean trustHeight = false;

      PoseReferenceFrame footstepPoseFrame = new PoseReferenceFrame("footstepPoseFrame", footstepPose);
      Footstep desiredFootstep = new Footstep(foot.getRigidBody(), swingSide, foot.getSoleFrame(), footstepPoseFrame, trustHeight);

      return desiredFootstep;
   }


   private void computeInitialDesiredFootstep(RobotSide supportLegSide)
   {
      if (couplingRegistry.getCaptureRegion() == null)
      {
         // ICP probably went outside the foot on the wrong side; no capture region. Can't handle this currently.
//       System.out.println("supportLegForWalkingCtrlr: " + couplingRegistry.getSupportLeg());
//       throw new RuntimeException("capture region is null! supportLegForWalkingCtrlr: " + couplingRegistry.getSupportLeg());
         return;
      }

      ReferenceFrame supportFootFrame = referenceFrames.getAnkleZUpFrame(supportLegSide);

      FramePoint captureRegionCentroid = couplingRegistry.getCaptureRegion().getCentroidCopy().toFramePoint();
      captureRegionCentroid.changeFrame(supportFootFrame);
      FramePoint sweetSpot = couplingRegistry.getOldBipedSupportPolygons().getSweetSpotCopy(supportLegSide).toFramePoint();
      sweetSpot.changeFrame(supportFootFrame);

      FrameVector offsetFromSweetSpot = new FrameVector(captureRegionCentroid);
      offsetFromSweetSpot.sub(sweetSpot);
      offsetFromSweetSpot.normalize();
      offsetFromSweetSpot.scale(stepDistance.getDoubleValue());

      FramePoint footstepPosition = new FramePoint(sweetSpot);
      footstepPosition.add(offsetFromSweetSpot);
      footstepPosition.setZ(0.0);

      RobotSide swingSide = supportLegSide.getOppositeSide();
      stepPositions.get(swingSide).set(footstepPosition);
      stepOrientations.get(supportLegSide.getOppositeSide()).setYawPitchRoll(0.0, 0.0, 0.0);
   }

   public void setupParametersForM2V2()
   {
      stepDistance.set(0.5);
   }

   public void setupParametersForR2()
   {
   }
   
   public boolean isDone()
   {
      return false;
   }

   @Override
   public void initialize()
   {
   }
}
