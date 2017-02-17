package us.ihmc.avatar.testTools;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.RectangularContactableBody;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ScriptedFootstepGenerator
{
   private final SideDependentList<ContactablePlaneBody> bipedFeet;

   public ScriptedFootstepGenerator(HumanoidReferenceFrames referenceFrames, FullHumanoidRobotModel fullRobotModel, WalkingControllerParameters walkingControllerParameters)
   {
      this.bipedFeet = setupBipedFeet(referenceFrames, fullRobotModel, walkingControllerParameters);
   }

   public ScriptedFootstepGenerator(SideDependentList<ContactablePlaneBody> bipedFeet)
   {
      this.bipedFeet = bipedFeet;
   }

   public FootstepDataListMessage generateFootstepsFromLocationsAndOrientations(RobotSide[] robotSides, double[][][] footstepLocationsAndOrientations)
   {
      return generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations, 0.0, 0.0);
   }
   
   public FootstepDataListMessage generateFootstepsFromLocationsAndOrientations(RobotSide[] robotSides, double[][][] footstepLocationsAndOrientations, double swingTime, double transferTime)
   {
      FootstepDataListMessage footstepDataList = new FootstepDataListMessage(swingTime, transferTime);

      for (int i = 0; i < robotSides.length; i++)
      {
         RobotSide robotSide = robotSides[i];
         double[][] footstepLocationAndOrientation = footstepLocationsAndOrientations[i];
         Footstep footstep = generateFootstepFromLocationAndOrientation(robotSide, footstepLocationAndOrientation);
         Point3D location = new Point3D();
         Quaternion orientation = new Quaternion();
         footstep.getPose(location, orientation);
         FootstepDataMessage footstepData = new FootstepDataMessage(robotSide, location, orientation);
         footstepDataList.add(footstepData);
      }

      return footstepDataList;
   }

   private Footstep generateFootstepFromLocationAndOrientation(RobotSide robotSide, double[][] footstepLocationAndOrientation)
   {
      double[] location = footstepLocationAndOrientation[0];
      double[] orientation = footstepLocationAndOrientation[1];

      return generateFootstepFromLocationAndOrientation(robotSide, location, orientation);
   }

   private Footstep generateFootstepFromLocationAndOrientation(RobotSide robotSide, double[] location, double[] orientation)
   {
      ContactablePlaneBody foot = bipedFeet.get(robotSide);
      boolean trustHeight = true;
      Quaternion quaternion = new Quaternion(orientation);

      FramePoint footstepPosition = new FramePoint(ReferenceFrame.getWorldFrame(), location);
      FrameOrientation footstepOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), quaternion);
      FramePose footstepPose = new FramePose(footstepPosition, footstepOrientation);
      footstepPose.changeFrame(ReferenceFrame.getWorldFrame());

      PoseReferenceFrame footstepPoseFrame = new PoseReferenceFrame("footstepPoseFrame", footstepPose);
      Footstep footstep = new Footstep(foot.getRigidBody(), robotSide, foot.getSoleFrame(), footstepPoseFrame, trustHeight);

      return footstep;
   }

   public Footstep generateFootstep(RobotSide robotSide, Point3D pointInWorldToStepTo, double footstepYaw, Vector3D surfaceNormal)
   {
      boolean trustHeight = true;
      ContactablePlaneBody foot = bipedFeet.get(robotSide);

      return generateFootstep(foot, robotSide, pointInWorldToStepTo, footstepYaw, surfaceNormal, trustHeight);
   }

   private final RotationMatrix tempMatrix3d = new RotationMatrix();

   public Footstep generateFootstep(ContactablePlaneBody foot, RobotSide robotSide, Point3D pointInWorldToStepTo, double footstepHeading, Vector3D surfaceNormal,
                                    boolean trustHeight)
   {
      getRotationGivenNormalAndHeading(tempMatrix3d, surfaceNormal, footstepHeading);

      FramePoint footstepPosition = new FramePoint(ReferenceFrame.getWorldFrame(), pointInWorldToStepTo);
      FrameOrientation footstepOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), tempMatrix3d);
      FramePose footstepPose = new FramePose(footstepPosition, footstepOrientation);
      footstepPose.changeFrame(ReferenceFrame.getWorldFrame());

      PoseReferenceFrame footstepPoseFrame = new PoseReferenceFrame("footstepPoseFrame", footstepPose);
      Footstep footstep = new Footstep(foot.getRigidBody(), robotSide, foot.getSoleFrame(), footstepPoseFrame, trustHeight);

      return footstep;
   }

   public SideDependentList<ContactablePlaneBody> setupBipedFeet(HumanoidReferenceFrames referenceFrames, FullHumanoidRobotModel fullRobotModel, WalkingControllerParameters walkingControllerParameters)
   {
      double footForward, footBack, footWidth;

      footForward = walkingControllerParameters.getFootForwardOffset();
      footBack = walkingControllerParameters.getFootBackwardOffset();
      footWidth = walkingControllerParameters.getFootWidth();

      SideDependentList<ContactablePlaneBody> bipedFeet = new SideDependentList<ContactablePlaneBody>();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody footBody = fullRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         double left = footWidth / 2.0;
         double right = -footWidth / 2.0;

         ContactablePlaneBody foot = new RectangularContactableBody(footBody, soleFrame, footForward, -footBack, left, right);
         bipedFeet.put(robotSide, foot);
      }

      return bipedFeet;
   }

   private final Vector3D tempXVector = new Vector3D();
   private final Vector3D tempYVector = new Vector3D();
   private final Vector3D tempZVector = new Vector3D();

   private void getRotationGivenNormalAndHeading(RotationMatrix matrixToPack, Vector3D normal, double heading)
   {
      if (Math.abs(normal.getZ()) < 1e-7)
         throw new RuntimeException("z component of normal must not be zero!");

      // Solve for normal dotted with xVector = 0 while keeping the x and y components of xVector in the heading direction.
      double xVectorX = Math.cos(heading);
      double xVectorY = Math.sin(heading);
      double xVectorZ = -(normal.getX() * xVectorX + normal.getY() * xVectorY) / normal.getZ();

      tempXVector.set(xVectorX, xVectorY, xVectorZ);
      tempZVector.set(normal);
      tempYVector.cross(tempZVector, tempXVector);

      tempXVector.normalize();
      tempYVector.normalize();
      tempZVector.normalize();

      matrixToPack.setColumns(tempXVector, tempYVector, tempZVector);
   }
}
