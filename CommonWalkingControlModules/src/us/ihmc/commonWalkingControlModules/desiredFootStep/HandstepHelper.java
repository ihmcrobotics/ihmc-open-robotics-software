package us.ihmc.commonWalkingControlModules.desiredFootStep;

import javax.media.j3d.Transform3D;
import javax.vecmath.AxisAngle4d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.GeometryTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

public class HandstepHelper
{
   private final FullRobotModel fullRobotModel;
   
   public HandstepHelper(FullRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;
   }
   
   public Handstep getDesiredHandstep(RobotSide robotSide, Tuple3d position, Vector3d surfaceNormal, double rotationAngleAboutNormal, double swingTrajectoryTime)
   {
      Transform3D transformOne = computeHandstepTransform(true, position, surfaceNormal, rotationAngleAboutNormal);
      FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame(), transformOne);
      FrameVector surfaceNormalFrameVector = new FrameVector(ReferenceFrame.getWorldFrame(), surfaceNormal);
      
      RigidBody hand = fullRobotModel.getHand(robotSide);
      Handstep handstep = new Handstep(robotSide, hand, framePose, surfaceNormalFrameVector, swingTrajectoryTime);

      return handstep;
   }

   public static Transform3D computeHandstepTransform(boolean rotateZIntoX, Tuple3d position, Vector3d surfaceNormal, double rotationAngleAboutNormal)
   {
      surfaceNormal.normalize();
      AxisAngle4d rotationAxisAngle = new AxisAngle4d();
      GeometryTools.getRotationBasedOnNormal(rotationAxisAngle, surfaceNormal);

      AxisAngle4d rotationAboutNormal = new AxisAngle4d(surfaceNormal, rotationAngleAboutNormal);

      Transform3D transformOne = new Transform3D();
      transformOne.set(rotationAboutNormal);

      Transform3D transformTwo = new Transform3D();
      transformTwo.set(rotationAxisAngle);
      transformOne.mul(transformTwo);

      if (rotateZIntoX)
      {
         Transform3D transformThree = new Transform3D();
         transformThree.rotY(Math.PI/2.0);
         transformOne.mul(transformThree);
      }
      
      transformOne.setTranslation(new Vector3d(position));

      return transformOne;
   }
}
