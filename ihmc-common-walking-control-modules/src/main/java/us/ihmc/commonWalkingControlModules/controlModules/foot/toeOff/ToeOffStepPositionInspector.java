package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff;

import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class ToeOffStepPositionInspector
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final SideDependentList<MovingReferenceFrame> soleZUpFrames;

   private final YoBoolean isSteppingDown = new YoBoolean("isSteppingDown", registry);
   private final YoBoolean isSteppingUp = new YoBoolean("isSteppingUp", registry);

   private final YoBoolean isStepLongEnough = new YoBoolean("isStepLongEnough", registry);
   private final YoBoolean isStepFarEnoughForward = new YoBoolean("isStepFarEnoughForward", registry);

   private final DoubleProvider minStepLengthForToeOff;
   private final DoubleProvider minStepForwardForToeOff;
   private final DoubleProvider heightChangeForNonFlatStep;

   private final double inPlaceWidth;
   private final double footLength;

   private final FramePoint3D trailingFootPosition = new FramePoint3D();
   private final FramePoint3D leadingFootPosition = new FramePoint3D();

   private final PoseReferenceFrame leadingFootFrame = new PoseReferenceFrame("leadingFootFrame", worldFrame);
   private final ZUpFrame leadingFootZUpFrame = new ZUpFrame(leadingFootFrame, "leadingFootZUpFrame");

   public ToeOffStepPositionInspector(SideDependentList<MovingReferenceFrame> soleZUpFrames,
                                      WalkingControllerParameters walkingControllerParameters,
                                      ToeOffParameters toeOffParameters,
                                      double inPlaceWidth,
                                      double footLength,
                                      YoRegistry parentRegistry)
   {
      this.soleZUpFrames = soleZUpFrames;
      this.inPlaceWidth = inPlaceWidth;
      this.footLength = footLength;

      minStepLengthForToeOff = new DoubleParameter("minStepLengthForToeOff", registry, toeOffParameters.getMinStepLengthForToeOff());
      minStepForwardForToeOff = new DoubleParameter("minStepForwardForToeOff", registry, footLength);
      heightChangeForNonFlatStep = new DoubleParameter("heightChangeForNonFlatStep", registry, walkingControllerParameters.getHeightChangeForNonFlatStep());

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      isSteppingUp.set(false);
      isSteppingDown.set(false);
      isStepLongEnough.set(false);
      isStepFarEnoughForward.set(false);
   }

   public boolean isFrontFootWellPositionedForToeOff(RobotSide trailingLeg, FramePose3DReadOnly frontFootPose)
   {
      ReferenceFrame trailingFootFrame = soleZUpFrames.get(trailingLeg);
      trailingFootPosition.setToZero(trailingFootFrame);
      leadingFootPosition.setIncludingFrame(frontFootPose.getPosition());
      leadingFootPosition.changeFrame(trailingFootFrame);

      leadingFootFrame.setPoseAndUpdate(frontFootPose);
      leadingFootZUpFrame.update();

      // Only add width when computing step length if it's wider than the nominal width
      if (Math.abs(leadingFootPosition.getY()) > inPlaceWidth)
         leadingFootPosition.setY(leadingFootPosition.getY() + trailingLeg.negateIfRightSide(inPlaceWidth));
      else
         leadingFootPosition.setY(0.0);

      isSteppingUp.set(leadingFootPosition.getZ() > heightChangeForNonFlatStep.getValue());
      isSteppingDown.set(leadingFootPosition.getZ() < -heightChangeForNonFlatStep.getValue());

      double scale = 1.0;
      if (isSteppingDown.getBooleanValue())
         scale = 0.5;

      double minStepLength = Math.max(scale * minStepLengthForToeOff.getValue(), footLength);
      double minStepForward = Math.max(scale * minStepForwardForToeOff.getValue(), footLength);
      isStepLongEnough.set(leadingFootPosition.distance(trailingFootPosition) > minStepLength);

      trailingFootPosition.changeFrame(leadingFootZUpFrame);

      boolean stepIsFarEnoughForwardFromStance = leadingFootPosition.getX() > minStepForward;
      boolean stanceIsFarEnoughBackwardFromStep = trailingFootPosition.getX() < minStepForward;

      isStepFarEnoughForward.set(stepIsFarEnoughForwardFromStance && stanceIsFarEnoughBackwardFromStep);

      if (isSteppingUp.getBooleanValue())
         return true;

      return isStepLongEnough.getValue() && isStepFarEnoughForward.getValue();
   }

   public boolean isSteppingUp()
   {
      return isSteppingUp.getBooleanValue();
   }
}
