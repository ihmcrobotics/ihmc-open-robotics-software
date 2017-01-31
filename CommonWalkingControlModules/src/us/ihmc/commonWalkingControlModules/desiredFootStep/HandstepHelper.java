package us.ihmc.commonWalkingControlModules.desiredFootStep;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;

public class HandstepHelper
{
   private final FullHumanoidRobotModel fullRobotModel;

   public HandstepHelper(FullHumanoidRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;
   }

   public Handstep getDesiredHandstep(RobotSide robotSide, Tuple3d position, Vector3d surfaceNormal, double rotationAngleAboutNormal,
         double swingTrajectoryTime)
   {
      RigidBodyTransform transformOne = computeHandstepTransform(true, position, surfaceNormal, rotationAngleAboutNormal);
      FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame(), transformOne);
      FrameVector surfaceNormalFrameVector = new FrameVector(ReferenceFrame.getWorldFrame(), surfaceNormal);

      RigidBody hand = fullRobotModel.getHand(robotSide);
      Handstep handstep = new Handstep(robotSide, hand, framePose, surfaceNormalFrameVector, swingTrajectoryTime);

      return handstep;
   }

   public static RigidBodyTransform computeHandstepTransform(boolean rotateZIntoX, Tuple3d position, Vector3d surfaceNormal, double rotationAngleAboutNormal)
   {
      surfaceNormal.normalize();
      AxisAngle4d rotationAxisAngle = new AxisAngle4d();
      GeometryTools.getAxisAngleFromZUpToVector(surfaceNormal, rotationAxisAngle);

      AxisAngle4d rotationAboutNormal = new AxisAngle4d(surfaceNormal, rotationAngleAboutNormal);

      RigidBodyTransform transformOne = new RigidBodyTransform();
      transformOne.setRotationAndZeroTranslation(rotationAboutNormal);

      RigidBodyTransform transformTwo = new RigidBodyTransform();
      transformTwo.setRotationAndZeroTranslation(rotationAxisAngle);
      transformOne.multiply(transformTwo);

      if (rotateZIntoX)
      {
         RigidBodyTransform transformThree = new RigidBodyTransform();
         transformThree.setRotationPitchAndZeroTranslation(Math.PI / 2.0);
         transformOne.multiply(transformThree);
      }

      transformOne.setTranslation(new Vector3d(position));

      return transformOne;
   }
}
