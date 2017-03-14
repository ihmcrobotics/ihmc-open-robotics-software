package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
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

   public Handstep getDesiredHandstep(RobotSide robotSide, Tuple3DBasics position, Vector3D surfaceNormal, double rotationAngleAboutNormal,
         double swingTrajectoryTime)
   {
      RigidBodyTransform transformOne = computeHandstepTransform(true, position, surfaceNormal, rotationAngleAboutNormal);
      FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame(), transformOne);
      FrameVector surfaceNormalFrameVector = new FrameVector(ReferenceFrame.getWorldFrame(), surfaceNormal);

      RigidBody hand = fullRobotModel.getHand(robotSide);
      Handstep handstep = new Handstep(robotSide, hand, framePose, surfaceNormalFrameVector, swingTrajectoryTime);

      return handstep;
   }

   public static RigidBodyTransform computeHandstepTransform(boolean rotateZIntoX, Tuple3DBasics position, Vector3D surfaceNormal, double rotationAngleAboutNormal)
   {
      surfaceNormal.normalize();
      AxisAngle rotationAxisAngle = new AxisAngle();
      EuclidGeometryTools.axisAngleFromZUpToVector3D(surfaceNormal, rotationAxisAngle);

      AxisAngle rotationAboutNormal = new AxisAngle(surfaceNormal, rotationAngleAboutNormal);

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

      transformOne.setTranslation(new Vector3D(position));

      return transformOne;
   }
}
