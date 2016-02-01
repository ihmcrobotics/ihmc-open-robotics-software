package us.ihmc.commonWalkingControlModules.trajectories;

import static us.ihmc.commonWalkingControlModules.trajectories.LeadInOutPositionTrajectoryGenerator.defaultClearanceTimeInPercentOfTrajectoryTime;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;


public class LeadInOutPoseTrajectoryGenerator implements PoseTrajectoryGenerator
{
   private final YoVariableRegistry registry;

   private final LeadInOutPositionTrajectoryGenerator positionTrajectoryGenerator;
   private final SimpleOrientationTrajectoryGenerator orientationTrajectoryGenerator;

   private final DoubleYoVariable leaveTime;

   private final FramePoint tempPosition = new FramePoint();
   private final FrameOrientation tempOrientation = new FrameOrientation();

   public LeadInOutPoseTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, false, referenceFrame, parentRegistry, false, null);
   }

   public LeadInOutPoseTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry,
         boolean visualize, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(namePrefix, false, referenceFrame, parentRegistry, visualize, yoGraphicsListRegistry);
   }

   public LeadInOutPoseTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame,
         YoVariableRegistry parentRegistry)
   {
      this(namePrefix, allowMultipleFrames, referenceFrame, parentRegistry, false, null);
   }

   public LeadInOutPoseTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame,
         YoVariableRegistry parentRegistry, boolean visualize, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);
      positionTrajectoryGenerator = new LeadInOutPositionTrajectoryGenerator(namePrefix, allowMultipleFrames, referenceFrame, registry, visualize, yoGraphicsListRegistry);
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

   public void setInitialLeadOut(FramePose initialPose, FrameVector initialDirection, double leaveDistance)
   {
      initialPose.getPositionIncludingFrame(tempPosition);
      initialPose.getOrientationIncludingFrame(tempOrientation);
      positionTrajectoryGenerator.setInitialLeadOut(tempPosition, initialDirection, leaveDistance);
      orientationTrajectoryGenerator.setInitialOrientation(tempOrientation);
   }

   public void setFinalLeadIn(FramePose finalPose, FrameVector finalDirection, double approachDistance)
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

   public void get(FramePoint positionToPack)
   {
      positionTrajectoryGenerator.get(positionToPack);
   }

   public void packVelocity(FrameVector velocityToPack)
   {
      positionTrajectoryGenerator.packVelocity(velocityToPack);
   }

   public void packAcceleration(FrameVector accelerationToPack)
   {
      positionTrajectoryGenerator.packAcceleration(accelerationToPack);
   }

   public void get(FrameOrientation orientationToPack)
   {
      orientationTrajectoryGenerator.get(orientationToPack);
   }

   public void packAngularVelocity(FrameVector angularVelocityToPack)
   {
      orientationTrajectoryGenerator.packAngularVelocity(angularVelocityToPack);
   }

   public void packAngularAcceleration(FrameVector angularAccelerationToPack)
   {
      orientationTrajectoryGenerator.packAngularAcceleration(angularAccelerationToPack);
   }

   public void packLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      get(positionToPack);
      packVelocity(velocityToPack);
      packAcceleration(accelerationToPack);
   }

   public void packAngularData(FrameOrientation orientationToPack, FrameVector angularVelocityToPack, FrameVector angularAccelerationToPack)
   {
      get(orientationToPack);
      packAngularVelocity(angularVelocityToPack);
      packAngularAcceleration(angularAccelerationToPack);
   }

   public void get(FramePose framePoseToPack)
   {
      positionTrajectoryGenerator.get(tempPosition);
      framePoseToPack.changeFrame(tempPosition.getReferenceFrame());
      framePoseToPack.setPosition(tempPosition);

      orientationTrajectoryGenerator.get(tempOrientation);
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
