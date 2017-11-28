package us.ihmc.commonWalkingControlModules.trajectories;

import static us.ihmc.commonWalkingControlModules.trajectories.LeadInOutPositionTrajectoryGenerator.defaultClearanceTimeInPercentOfTrajectoryTime;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;

public class LeadInOutPoseTrajectoryGenerator implements PoseTrajectoryGenerator
{
   private final YoVariableRegistry registry;

   private final LeadInOutPositionTrajectoryGenerator positionTrajectoryGenerator;
   private final SimpleOrientationTrajectoryGenerator orientationTrajectoryGenerator;

   private final YoDouble leaveTime;

   private final FramePoint3D tempPosition = new FramePoint3D();
   private final FrameQuaternion tempOrientation = new FrameQuaternion();

   public LeadInOutPoseTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, false, referenceFrame, parentRegistry, false, null);
   }

   public LeadInOutPoseTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry, boolean visualize,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(namePrefix, false, referenceFrame, parentRegistry, visualize, yoGraphicsListRegistry);
   }

   public LeadInOutPoseTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, allowMultipleFrames, referenceFrame, parentRegistry, false, null);
   }

   public LeadInOutPoseTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry,
         boolean visualize, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);
      positionTrajectoryGenerator = new LeadInOutPositionTrajectoryGenerator(namePrefix, allowMultipleFrames, referenceFrame, registry, visualize,
            yoGraphicsListRegistry);
      orientationTrajectoryGenerator = new SimpleOrientationTrajectoryGenerator(namePrefix, allowMultipleFrames, referenceFrame, registry);

      leaveTime = positionTrajectoryGenerator.getYoLeaveTime();
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

   public void setInitialLeadOut(FramePose initialPose, FrameVector3D initialDirection, double leaveDistance)
   {
      initialPose.getPositionIncludingFrame(tempPosition);
      initialPose.getOrientationIncludingFrame(tempOrientation);
      positionTrajectoryGenerator.setInitialLeadOut(tempPosition, initialDirection, leaveDistance);
      orientationTrajectoryGenerator.setInitialOrientation(tempOrientation);
   }

   public void setFinalLeadIn(FramePose finalPose, FrameVector3D finalDirection, double approachDistance)
   {
      finalPose.getPositionIncludingFrame(tempPosition);
      finalPose.getOrientationIncludingFrame(tempOrientation);
      positionTrajectoryGenerator.setFinalLeadIn(tempPosition, finalDirection, approachDistance);
      orientationTrajectoryGenerator.setFinalOrientation(tempOrientation);
   }

   public void setTrajectoryTime(double newTrajectoryTime)
   {
      double leaveTime = defaultClearanceTimeInPercentOfTrajectoryTime * newTrajectoryTime;
      double approachTime = defaultClearanceTimeInPercentOfTrajectoryTime * newTrajectoryTime;
      positionTrajectoryGenerator.setTrajectoryTime(newTrajectoryTime, leaveTime, approachTime);
      orientationTrajectoryGenerator.setTrajectoryTime(newTrajectoryTime - leaveTime - approachTime);
   }

   public void setTrajectoryTime(double newTrajectoryTime, double leaveTime, double approachTime)
   {
      positionTrajectoryGenerator.setTrajectoryTime(newTrajectoryTime, leaveTime, approachTime);
      orientationTrajectoryGenerator.setTrajectoryTime(newTrajectoryTime - leaveTime - approachTime);
   }

   public void initialize()
   {
      positionTrajectoryGenerator.initialize();
      orientationTrajectoryGenerator.initialize();
   }

   public void compute(double time)
   {
      positionTrajectoryGenerator.compute(time);
      orientationTrajectoryGenerator.compute(time - leaveTime.getDoubleValue());
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
