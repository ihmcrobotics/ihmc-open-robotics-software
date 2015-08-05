package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.humanoidRobotics.frames.CommonHumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.partNames.LegJointName;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.controllers.PDController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.trajectories.YoMinimumJerkTrajectory;


public class FootOrientationControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("FootOrientationControlModule");

   private final PDController pitchController = new PDController("footPitch", registry);
   private final PDController rollController = new PDController("footRoll", registry);

   private final DoubleYoVariable desiredFootPitch = new DoubleYoVariable("desiredFootPitch", "", registry);
   private final DoubleYoVariable desiredFootRoll = new DoubleYoVariable("desiredFootRoll", "", registry);

   private final DoubleYoVariable currentFootPitch = new DoubleYoVariable("currentFootPitch", "", registry);
   private final DoubleYoVariable currentFootRoll = new DoubleYoVariable("currentFootRoll", "", registry);


   private final YoMinimumJerkTrajectory minimumJerkTrajectory;
   private final ProcessedSensorsInterface processedSensors;
   private final CommonHumanoidReferenceFrames referenceFrames;

   private final ReferenceFrame desiredHeadingFrame;

   private final YoFrameOrientation initialOrientation;
   private final YoFrameOrientation finalOrientation;

   private final FrameOrientation desiredFootOrientation;


   public FootOrientationControlModule(ProcessedSensorsInterface processedSensors, CommonHumanoidReferenceFrames referenceFrames,
           DesiredHeadingControlModule desiredHeadingControlModule, YoVariableRegistry parentRegistry)
   {
      minimumJerkTrajectory = new YoMinimumJerkTrajectory("footOrientation", registry);
      this.processedSensors = processedSensors;
      this.referenceFrames = referenceFrames;
      this.desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();
      desiredFootOrientation = new FrameOrientation(desiredHeadingFrame);
      initialOrientation = new YoFrameOrientation("footInitialOrientation", "", desiredHeadingFrame, registry);
      finalOrientation = new YoFrameOrientation("footFinalOrientation", "", desiredHeadingFrame, registry);

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }
   
   public void setParametersForR2()
   {
      pitchController.setProportionalGain(0.0); // 10.0);    // 500.0); //200.0); //0.0);
      rollController.setProportionalGain(10.0);    // 0.0);

      pitchController.setDerivativeGain(0.0); // 1.0);    // 0.0);
      rollController.setDerivativeGain(1.0);    // 0.0);
   }
   
   public void setParametersForM2V2()
   {
      pitchController.setProportionalGain(0.0);
      rollController.setProportionalGain(0.0);

      pitchController.setDerivativeGain(0.0);
      rollController.setDerivativeGain(0.0);
   }

   public void initializeFootOrientationMove(double moveDuration, FrameOrientation finalOrientation, RobotSide supportFoot)
   {
      minimumJerkTrajectory.setParams(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, moveDuration);
      this.finalOrientation.set(finalOrientation);
      this.initialOrientation.set(getFootOrientationInFrame(desiredHeadingFrame, supportFoot));
   }

   public void addAdditionalTorqueForFootOrientationControl(LegTorques legTorquesToAddAddionalTorque, double timeInMove)
   {
      minimumJerkTrajectory.computeTrajectory(timeInMove);
      double alpha = minimumJerkTrajectory.getPosition();

      RobotSide supportLeg = legTorquesToAddAddionalTorque.getRobotSide();

      FrameOrientation currentFootOrientation = getFootOrientationInFrame(desiredHeadingFrame, supportLeg);

      desiredFootOrientation.interpolate(initialOrientation.getFrameOrientationCopy(), finalOrientation.getFrameOrientationCopy(), alpha);

      double[] currentFootYawPitchRoll = currentFootOrientation.getYawPitchRoll();
      currentFootPitch.set(currentFootYawPitchRoll[1]);
      currentFootRoll.set(currentFootYawPitchRoll[2]);

      double[] desiredFootYawPitchRoll = desiredFootOrientation.getYawPitchRoll();
      desiredFootPitch.set(desiredFootYawPitchRoll[1]);
      desiredFootRoll.set(desiredFootYawPitchRoll[2]);

      double currentPitchVelocity = processedSensors.getLegJointVelocity(supportLeg, LegJointName.ANKLE_PITCH);
      double currentRollVelocity = processedSensors.getLegJointVelocity(supportLeg, LegJointName.ANKLE_ROLL);

      double desiredPitchVelocity = 0.0;
      double desiredRollVelocity = 0.0;

      double pitchTorque = pitchController.compute(currentFootPitch.getDoubleValue(), desiredFootPitch.getDoubleValue(), currentPitchVelocity,
                              desiredPitchVelocity);
      double rollTorque = rollController.compute(currentFootRoll.getDoubleValue(), desiredFootRoll.getDoubleValue(), currentRollVelocity, desiredRollVelocity);

      legTorquesToAddAddionalTorque.addTorque(LegJointName.ANKLE_PITCH, pitchTorque);
      legTorquesToAddAddionalTorque.addTorque(LegJointName.ANKLE_ROLL, rollTorque);
   }
   
   private FrameOrientation getFootOrientationInFrame(ReferenceFrame referenceFrame, RobotSide supportFoot)
   {
      ReferenceFrame supportFootFrame = referenceFrames.getFootFrame(supportFoot);
      FrameOrientation orientation = new FrameOrientation(referenceFrame, supportFootFrame.getTransformToDesiredFrame(referenceFrame));
      return orientation;
   }
}
