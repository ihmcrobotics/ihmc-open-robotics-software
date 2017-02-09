package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class FootstepTools
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public static Footstep generateFootstepFromFootstepData(FootstepDataMessage footstepData, ContactablePlaneBody contactableBody)
   {
      return generateFootstepFromFootstepData(footstepData, contactableBody, -1);
   }

   public static Footstep generateFootstepFromFootstepData(FootstepDataMessage footstepData, ReferenceFrame soleFrame, RigidBody rigidBody)
   {
      return generateFootstepFromFootstepData(footstepData, soleFrame, rigidBody, -1);
   }

   public static Footstep generateFootstepFromFootstepData(FootstepDataMessage footstepData, ContactablePlaneBody contactableBody, int index)
   {
      ReferenceFrame soleFrame = contactableBody.getSoleFrame();
      RigidBody rigidBody = contactableBody.getRigidBody();
      return generateFootstepFromFootstepData(footstepData, soleFrame, rigidBody, index);
   }

   public static Footstep generateFootstepFromFootstepData(FootstepDataMessage footstepData, ReferenceFrame soleFrame, RigidBody rigidBody, int index)
   {
      FramePose footstepPose = new FramePose(worldFrame, footstepData.getLocation(), footstepData.getOrientation());
      PoseReferenceFrame footstepPoseFrame = new PoseReferenceFrame("footstepPoseFrame", footstepPose);

      List<Point2d> contactPoints = footstepData.getPredictedContactPoints();
      if (contactPoints != null && contactPoints.size() == 0)
         contactPoints = null;


      RobotSide robotSide = footstepData.getRobotSide();
      TrajectoryType trajectoryType = footstepData.getTrajectoryType();

      Footstep footstep;

      if (index == -1)
         footstep = new Footstep(rigidBody, robotSide, soleFrame, footstepPoseFrame, true, contactPoints);
      else
      {
         String id = "footstep_" + index;
         footstep = new Footstep(id, rigidBody, robotSide, soleFrame, footstepPoseFrame, true, contactPoints);
      }

      footstep.setTrajectoryType(trajectoryType);
      footstep.setSwingHeight(footstepData.swingHeight);
      switch (footstepData.getOrigin())
      {
      case AT_ANKLE_FRAME:
         break;
      case AT_SOLE_FRAME:
         footstep.setSolePose(footstepPose);
         break;
      default:
         throw new RuntimeException("Should not get there.");
      }
      return footstep;
   }
}
