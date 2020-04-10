package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.behaviors.goalLocation.GoalDetectorBehaviorService;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.ros2.Ros2Node;

public class ConstantGoalDetectorBehaviorService extends GoalDetectorBehaviorService
{
   private final HumanoidReferenceFrames referenceFrames;
   private final Point3D constantGoalLocationInMidFeetZUpFrame = new Point3D();

   public ConstantGoalDetectorBehaviorService(String robotName, HumanoidReferenceFrames referenceFrames, Point3D constantGoalLocationInMidFeetZUpFrame, Ros2Node ros2Node)
   {
      super(robotName, ConstantGoalDetectorBehaviorService.class.getSimpleName(), ros2Node);
      this.referenceFrames = referenceFrames;
      this.constantGoalLocationInMidFeetZUpFrame.set(constantGoalLocationInMidFeetZUpFrame);
   }

   @Override
   public boolean getGoalHasBeenLocated()
   {
      return true;
   }

   @Override
   public void getReportedGoalPoseWorldFrame(FramePose3D framePoseToPack)
   {
      ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      FramePoint3D goalPosition = new FramePoint3D(midFeetZUpFrame, constantGoalLocationInMidFeetZUpFrame);
      goalPosition.changeFrame(ReferenceFrame.getWorldFrame());

      FrameQuaternion goalOrientation = new FrameQuaternion(midFeetZUpFrame);
      goalOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      framePoseToPack.getPosition().set(goalPosition);
      framePoseToPack.getOrientation().set(goalOrientation);
   }

   @Override
   public void doThreadAction()
   {
   }

   @Override
   public void initialize()
   {
   }

}
