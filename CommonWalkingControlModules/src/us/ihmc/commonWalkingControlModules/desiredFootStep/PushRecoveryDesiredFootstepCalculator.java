package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class PushRecoveryDesiredFootstepCalculator implements DesiredFootstepCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("PushRecoveryDesiredStepLocationCalcualtor");

   private final CommonWalkingReferenceFrames referenceFrames;
   private final CouplingRegistry couplingRegistry;

   private final SideDependentList<YoFramePoint> stepPositions = new SideDependentList<YoFramePoint>();
   private final SideDependentList<YoFrameOrientation> stepOrientations = new SideDependentList<YoFrameOrientation>();
   private final DoubleYoVariable stepDistance = new DoubleYoVariable("stepDistance", registry);

   public PushRecoveryDesiredFootstepCalculator(CommonWalkingReferenceFrames referenceFrames, CouplingRegistry couplingRegistry,
           YoVariableRegistry parentRegistry)
   {

      this.referenceFrames = referenceFrames;
      this.couplingRegistry = couplingRegistry;
      
      for (RobotSide swingSide : RobotSide.values())
      {
         RobotSide supportSide = swingSide.getOppositeSide();
         ReferenceFrame ankleZUpFrame = referenceFrames.getAnkleZUpFrame(supportSide);
         stepPositions.put(swingSide, new YoFramePoint(swingSide.getCamelCaseNameForStartOfExpression() + "StepPosition", "", ankleZUpFrame, registry));
         stepOrientations.put(swingSide, new YoFrameOrientation(swingSide.getCamelCaseNameForStartOfExpression() + "StepOrientation", "", ankleZUpFrame, registry));
      }
      
      parentRegistry.addChild(registry);
   }


   public void initializeDesiredFootstep(RobotSide supportLegSide)
   {
      computeInitialDesiredFootstep(supportLegSide);
   }

   public Footstep updateAndGetDesiredFootstep(RobotSide supportLegSide)
   {
//      double estimatedSwingTimeRemaining = couplingRegistry.getEstimatedSwingTimeRemaining();
//      if (estimatedSwingTimeRemaining > 0.1)
//      {
//         computeInitialDesiredFootstep(supportLegSide);
//      }

      RobotSide swingSide = supportLegSide.getOppositeSide();
      FramePose footstepPose = new FramePose(stepPositions.get(swingSide).getFramePointCopy(), stepOrientations.get(swingSide).getFrameOrientationCopy());
      Footstep desiredFootstep = new Footstep(swingSide, footstepPose);
      return desiredFootstep;
   }


   private void computeInitialDesiredFootstep(RobotSide supportLegSide)
   {
      if (couplingRegistry.getCaptureRegion() == null)
      {
         // ICP probably went outside the foot on the wrong side; no capture region. Can't handle this currently.
         System.out.println("supportLegForWalkingCtrlr: " + couplingRegistry.getSupportLeg());
         throw new RuntimeException("capture region is null! supportLegForWalkingCtrlr: " + couplingRegistry.getSupportLeg());
      }
      
      ReferenceFrame supportFootFrame = referenceFrames.getAnkleZUpFrame(supportLegSide);
      
      FramePoint captureRegionCentroid = couplingRegistry.getCaptureRegion().getCentroidCopy().toFramePoint();
      captureRegionCentroid.changeFrame(supportFootFrame);
      FramePoint sweetSpot = couplingRegistry.getBipedSupportPolygons().getSweetSpotCopy(supportLegSide).toFramePoint();
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
}
