package us.ihmc.commonWalkingControlModules.trajectories;

import static us.ihmc.commonWalkingControlModules.trajectories.InitialClearancePositionTrajectoryGenerator.defaultLeaveTimeInPercentOfTrajectoryTime;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;

public class InitialClearancePoseTrajectoryGenerator implements PoseTrajectoryGenerator
{
   private final InitialClearancePositionTrajectoryGenerator positionTrajectoryGenerator;
   private final SimpleOrientationTrajectoryGenerator orientationTrajectoryGenerator;

   private final FramePoint3D tempPosition = new FramePoint3D();
   private final FrameQuaternion tempOrientation = new FrameQuaternion();

   public InitialClearancePoseTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, false, referenceFrame, parentRegistry, false, null);
   }

   public InitialClearancePoseTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry, boolean visualize,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(namePrefix, false, referenceFrame, parentRegistry, visualize, yoGraphicsListRegistry);
   }

   public InitialClearancePoseTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame,
         YoVariableRegistry parentRegistry)
   {
      this(namePrefix, allowMultipleFrames, referenceFrame, parentRegistry, false, null);
   }

   public InitialClearancePoseTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame,
         YoVariableRegistry parentRegistry, boolean visualize, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);
      positionTrajectoryGenerator = new InitialClearancePositionTrajectoryGenerator(namePrefix, allowMultipleFrames, referenceFrame, registry, visualize,
            yoGraphicsListRegistry);
      orientationTrajectoryGenerator = new SimpleOrientationTrajectoryGenerator(namePrefix, allowMultipleFrames, referenceFrame, registry);
   }

   public void registerAndSwitchFrame(ReferenceFrame desiredFrame)
   {
      positionTrajectoryGenerator.registerAndSwitchFrame(desiredFrame);
      orientationTrajectoryGenerator.registerAndSwitchFrame(desiredFrame);
   }

   public void registerNewTrajectoryFrame(ReferenceFrame newReferenceFrame)
   {
      positionTrajectoryGenerator.registerNewTrajectoryFrame(newReferenceFrame);
      orientationTrajectoryGenerator.registerNewTrajectoryFrame(newReferenceFrame);
   }

   public void changeFrame(ReferenceFrame referenceFrame)
   {
      positionTrajectoryGenerator.changeFrame(referenceFrame);
      orientationTrajectoryGenerator.changeFrame(referenceFrame);
   }

   public void switchTrajectoryFrame(ReferenceFrame referenceFrame)
   {
      positionTrajectoryGenerator.switchTrajectoryFrame(referenceFrame);
      orientationTrajectoryGenerator.switchTrajectoryFrame(referenceFrame);
   }

   public void setInitialPose(FramePose initialPose)
   {
      initialPose.getPositionIncludingFrame(tempPosition);
      initialPose.getOrientationIncludingFrame(tempOrientation);
      positionTrajectoryGenerator.setInitialPosition(tempPosition);
      orientationTrajectoryGenerator.setInitialOrientation(tempOrientation);
   }

   public void setFinalPose(FramePose finalPose)
   {
      finalPose.getPositionIncludingFrame(tempPosition);
      finalPose.getOrientationIncludingFrame(tempOrientation);
      positionTrajectoryGenerator.setFinalPosition(tempPosition);
      orientationTrajectoryGenerator.setFinalOrientation(tempOrientation);
   }

   public void setInitialClearance(FrameVector3D initialDirection, double leaveDistance)
   {
      positionTrajectoryGenerator.setInitialClearance(initialDirection, leaveDistance);
   }

   public void setTrajectoryTime(double newTrajectoryTime)
   {
      double leaveTime = defaultLeaveTimeInPercentOfTrajectoryTime * newTrajectoryTime;
      positionTrajectoryGenerator.setTrajectoryTime(newTrajectoryTime, leaveTime);
      orientationTrajectoryGenerator.setTrajectoryTime(newTrajectoryTime - leaveTime);
   }

   public void setTrajectoryTime(double newTrajectoryTime, double approachTime)
   {
      positionTrajectoryGenerator.setTrajectoryTime(newTrajectoryTime, approachTime);
      orientationTrajectoryGenerator.setTrajectoryTime(newTrajectoryTime - approachTime);
   }

   public void initialize()
   {
      positionTrajectoryGenerator.initialize();
      orientationTrajectoryGenerator.initialize();
   }

   public void compute(double time)
   {
      positionTrajectoryGenerator.compute(time);
      orientationTrajectoryGenerator.compute(time);
   }

   public void showVisualization()
   {
      positionTrajectoryGenerator.showVisualization();
   }

   public void hideVisualization()
   {
      positionTrajectoryGenerator.hideVisualization();
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      positionTrajectoryGenerator.getPosition(positionToPack);
   }

   public void getVelocity(FrameVector3D velocityToPack)
   {
      positionTrajectoryGenerator.getVelocity(velocityToPack);
   }

   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      positionTrajectoryGenerator.getAcceleration(accelerationToPack);
   }

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationTrajectoryGenerator.getOrientation(orientationToPack);
   }

   public void getAngularVelocity(FrameVector3D angularVelocityToPack)
   {
      orientationTrajectoryGenerator.getAngularVelocity(angularVelocityToPack);
   }

   public void getAngularAcceleration(FrameVector3D angularAccelerationToPack)
   {
      orientationTrajectoryGenerator.getAngularAcceleration(angularAccelerationToPack);
   }

   public void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   public void getAngularData(FrameQuaternion orientationToPack, FrameVector3D angularVelocityToPack, FrameVector3D angularAccelerationToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
      getAngularAcceleration(angularAccelerationToPack);
   }

   public void getPose(FramePose framePoseToPack)
   {
      positionTrajectoryGenerator.getPosition(tempPosition);
      framePoseToPack.changeFrame(tempPosition.getReferenceFrame());
      framePoseToPack.setPosition(tempPosition);

      orientationTrajectoryGenerator.getOrientation(tempOrientation);
      framePoseToPack.setOrientation(tempOrientation);
   }

   public boolean isDone()
   {
      return positionTrajectoryGenerator.isDone() && orientationTrajectoryGenerator.isDone();
   }

   public String toString()
   {
      String ret = "";
      ret += positionTrajectoryGenerator.toString();
      ret += orientationTrajectoryGenerator.toString();
      return ret;
   }
}
