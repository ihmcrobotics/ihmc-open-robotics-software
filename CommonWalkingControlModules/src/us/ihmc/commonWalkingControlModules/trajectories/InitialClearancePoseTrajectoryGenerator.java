package us.ihmc.commonWalkingControlModules.trajectories;

import static us.ihmc.commonWalkingControlModules.trajectories.InitialClearancePositionTrajectoryGenerator.defaultLeaveTimeInPercentOfTrajectoryTime;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class InitialClearancePoseTrajectoryGenerator implements PoseTrajectoryGenerator
{
   private final InitialClearancePositionTrajectoryGenerator positionTrajectoryGenerator;
   private final SimpleOrientationTrajectoryGenerator orientationTrajectoryGenerator;

   private final FramePoint tempPosition = new FramePoint();
   private final FrameOrientation tempOrientation = new FrameOrientation();

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

   public void setInitialClearance(FrameVector initialDirection, double leaveDistance)
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

   public void getPosition(FramePoint positionToPack)
   {
      positionTrajectoryGenerator.getPosition(positionToPack);
   }

   public void getVelocity(FrameVector velocityToPack)
   {
      positionTrajectoryGenerator.getVelocity(velocityToPack);
   }

   public void getAcceleration(FrameVector accelerationToPack)
   {
      positionTrajectoryGenerator.getAcceleration(accelerationToPack);
   }

   public void getOrientation(FrameOrientation orientationToPack)
   {
      orientationTrajectoryGenerator.getOrientation(orientationToPack);
   }

   public void getAngularVelocity(FrameVector angularVelocityToPack)
   {
      orientationTrajectoryGenerator.getAngularVelocity(angularVelocityToPack);
   }

   public void getAngularAcceleration(FrameVector angularAccelerationToPack)
   {
      orientationTrajectoryGenerator.getAngularAcceleration(angularAccelerationToPack);
   }

   public void getLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   public void getAngularData(FrameOrientation orientationToPack, FrameVector angularVelocityToPack, FrameVector angularAccelerationToPack)
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
