package us.ihmc.commonWalkingControlModules.controlModules.desiredPelvisOrientation;

import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredPelvisOrientationControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;


public class SimpleDesiredPelvisOrientationControlModule implements DesiredPelvisOrientationControlModule
{
   private final CommonHumanoidReferenceFrames referenceFrames;

   private final ReferenceFrame desiredHeadingFrame;

   private final YoVariableRegistry registry = new YoVariableRegistry("SimpleDesiredPelvisOrientationControlModule");
   private final DoubleYoVariable twistScale = new DoubleYoVariable("twistScale", registry);
   private final AlphaFilteredYoVariable alphaFilteredTwistScale;
   private final DoubleYoVariable userDesiredPelvisPitch = new DoubleYoVariable("userDesiredPelvisPitch", registry);
   private final DoubleYoVariable pelvisRollPelvisYawScale = new DoubleYoVariable("pelvisYawToRollScale", registry);

   private final YoFrameOrientation desiredPelvisOrientation;
   private final BooleanYoVariable usingExternallySpecifiedOrientation = new BooleanYoVariable("usingExternallySpecifiedOrientation", registry);
   
   private final BooleanYoVariable useTwistScale = new BooleanYoVariable("useTwistScale", registry);

   public SimpleDesiredPelvisOrientationControlModule(CommonHumanoidReferenceFrames referenceFrames, DesiredHeadingControlModule desiredHeadingControlModule, double controlDT,
           YoVariableRegistry parentRegistry)
   {
      this.referenceFrames = referenceFrames;

      this.desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();
      this.desiredPelvisOrientation = new YoFrameOrientation("desiredPelvis", "", desiredHeadingFrame, registry);
      this.alphaFilteredTwistScale = new AlphaFilteredYoVariable("alphaFitleredTwistScale", registry, AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(1.0, controlDT));
      useTwistScale.set(true);
      
      parentRegistry.addChild(registry);
   }

   public FrameOrientation getDesiredPelvisOrientationSingleSupportCopy(RobotSide robotSide)
   {
      if (!usingExternallySpecifiedOrientation.getBooleanValue())
         setDesiredPelvisOrientationScaledKneeToKnee(robotSide);
      return desiredPelvisOrientation.getFrameOrientationCopy();
   }

   public FrameOrientation getDesiredPelvisOrientationDoubleSupportCopy()
   {
      if (!usingExternallySpecifiedOrientation.getBooleanValue())
         setDesiredPelvisOrientationScaledKneeToKnee(null);
      return desiredPelvisOrientation.getFrameOrientationCopy();
   }

   private void setDesiredPelvisOrientationScaledKneeToKnee(RobotSide supportLeg)
   {
      ReferenceFrame leftKneeFrame = referenceFrames.getLegJointFrame(RobotSide.LEFT, LegJointName.KNEE);
      ReferenceFrame rightKneeFrame = referenceFrames.getLegJointFrame(RobotSide.RIGHT, LegJointName.KNEE);

      FramePoint leftKneeOrigin = new FramePoint(leftKneeFrame);
      FramePoint rightKneeOrigin = new FramePoint(rightKneeFrame);

      leftKneeOrigin.changeFrame(desiredHeadingFrame);
      rightKneeOrigin.changeFrame(desiredHeadingFrame);

      // Robot pelvis yaw is oriented at 90 deg about z axis from atan2 function's 0 orientation
      double desiredPelvisYaw = getDesiredPelvisYaw(leftKneeOrigin, rightKneeOrigin);

      double desiredPelvisRoll = getDesiredPelvisRoll(desiredPelvisYaw);
//      double desiredPelvisRoll = 0.0;

      desiredPelvisOrientation.setYawPitchRoll(desiredPelvisYaw, userDesiredPelvisPitch.getDoubleValue(), desiredPelvisRoll);
   }

   private double getDesiredPelvisRoll(double desiredPelvisYaw)
   {
      double desiredPelvisRoll = pelvisRollPelvisYawScale.getDoubleValue() * desiredPelvisYaw;
      return desiredPelvisRoll;
   }

   private double getDesiredPelvisYaw(FramePoint leftPoint, FramePoint rightPoint)
   {
      double currentTwistScale = useTwistScale.getBooleanValue() ? twistScale.getDoubleValue() : 0.0;
      alphaFilteredTwistScale.update(currentTwistScale);
      
      double desiredPelvisYaw = Math.atan2(-(leftPoint.getX() - rightPoint.getX()), (leftPoint.getY() - rightPoint.getY()));
      desiredPelvisYaw = desiredPelvisYaw * alphaFilteredTwistScale.getDoubleValue();
      return desiredPelvisYaw;
   }
   
   public void setParametersForR2()
   {
      twistScale.set(0.3);
      userDesiredPelvisPitch.set(0.1); // 061010: changed to 0.1 to account for heavy backpack
      pelvisRollPelvisYawScale.set(0.25);
   }
   
   public void setParametersForM2V2()
   {
      twistScale.set(0.2);
      userDesiredPelvisPitch.set(0.0);      
      pelvisRollPelvisYawScale.set(0.2);
   }

   public void setDesiredPelvisOrientation(FrameOrientation orientation)
   {
      if (orientation != null)
      {
         this.desiredPelvisOrientation.set(orientation);
         this.usingExternallySpecifiedOrientation.set(true);
      }
      else
         this.usingExternallySpecifiedOrientation.set(false);
   }

   public FrameOrientation getEstimatedOrientationAtEndOfStepCopy(RobotSide stanceSide, FramePoint desiredFootLocation)
   {
      ReferenceFrame referenceFrame = desiredFootLocation.getReferenceFrame();
      FramePoint desiredFootPosition = desiredFootLocation;
      FramePoint stanceFoot = new FramePoint(referenceFrames.getAnkleZUpFrame(stanceSide));
      
      stanceFoot.changeFrame(referenceFrame);
      
      
      double desiredPelvisYaw;
      
      if(stanceSide == RobotSide.RIGHT)
      {
         desiredPelvisYaw = getDesiredPelvisYaw(desiredFootPosition, stanceFoot);
      }
      else
      {
         desiredPelvisYaw = getDesiredPelvisYaw(stanceFoot, desiredFootPosition);
      }
      
      double desiredPelvisRoll = getDesiredPelvisRoll(desiredPelvisYaw);
      
      return new FrameOrientation(referenceFrame, desiredPelvisYaw, userDesiredPelvisPitch.getDoubleValue(), desiredPelvisRoll);
   }

   public void useTwistScale(boolean useTwistScale)
   {
      this.useTwistScale.set(useTwistScale);
   }
}
