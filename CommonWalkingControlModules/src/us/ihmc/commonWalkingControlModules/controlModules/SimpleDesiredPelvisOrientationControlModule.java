package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.controlModules.DesiredPelvisOrientationControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;

public class SimpleDesiredPelvisOrientationControlModule implements DesiredPelvisOrientationControlModule
{
   private final CommonWalkingReferenceFrames referenceFrames;

   private final ReferenceFrame desiredHeadingFrame;

   private final DoubleYoVariable twistScale;
   private final DoubleYoVariable userDesiredPelvisPitch;

   private final YoFrameOrientation desiredPelvisOrientation;

   public SimpleDesiredPelvisOrientationControlModule(CommonWalkingReferenceFrames referenceFrames, DesiredHeadingControlModule desiredHeadingControlModule,
           YoVariableRegistry registry)
   {
      this.referenceFrames = referenceFrames;

      this.desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();

      this.desiredPelvisOrientation = new YoFrameOrientation("desiredPelvis", "", desiredHeadingFrame, registry);

      userDesiredPelvisPitch = new DoubleYoVariable("userDesiredPelvisPitch", registry);
      twistScale = new DoubleYoVariable("twistScale", registry);

      twistScale.set(0.3);

      // 061010: changed to 0.1 to account for heavy backpack
      userDesiredPelvisPitch.set(0.1);
   }

   public Orientation getDesiredPelvisOrientationSingleSupport(RobotSide robotSide)
   {
//    return getDesiredPelvisOrientationYawAllTheWayAnkleToAnkle();
//      return getDesiredPelvisOrientationScaledAnkleToAnkle(robotSide);

      return getDesiredPelvisOrientationScaledKneeToKnee(robotSide);
   }

   public Orientation getDesiredPelvisOrientationDoubleSupport()
   {
//    return getDesiredPelvisOrientationScaledAnkleToAnkle(null);

      return getDesiredPelvisOrientationScaledKneeToKnee(null);
   }

  
   private Orientation getDesiredPelvisOrientationScaledKneeToKnee(RobotSide supportLeg)
   {
      ReferenceFrame leftKneeFrame = referenceFrames.getKneeFrame(RobotSide.LEFT);
      ReferenceFrame rightKneeFrame = referenceFrames.getKneeFrame(RobotSide.RIGHT);

      FramePoint leftKneeOrigin = new FramePoint(leftKneeFrame);
      FramePoint rightKneeOrigin = new FramePoint(rightKneeFrame);

      leftKneeOrigin = leftKneeOrigin.changeFrameCopy(desiredHeadingFrame);
      rightKneeOrigin = rightKneeOrigin.changeFrameCopy(desiredHeadingFrame);

      // Robot pelvis yaw is oriented at 90 deg about z axis from atan2 function's 0 orientation
      double desiredPelvisYaw = Math.atan2(-(leftKneeOrigin.getX() - rightKneeOrigin.getX()), (leftKneeOrigin.getY() - rightKneeOrigin.getY()));
      desiredPelvisYaw = desiredPelvisYaw * twistScale.getDoubleValue();

      double desiredPelvisRoll = 0.25 * desiredPelvisYaw;
//      double desiredPelvisRoll = 0.0;


      desiredPelvisOrientation.setYawPitchRoll(desiredPelvisYaw, userDesiredPelvisPitch.getDoubleValue(), desiredPelvisRoll);

      return desiredPelvisOrientation.getFrameOrientationCopy();
   }
}
