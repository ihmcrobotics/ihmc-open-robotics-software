package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.FramePoint3D;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;

public class FootstepTestHelper
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;

   public FootstepTestHelper(SideDependentList<? extends ContactablePlaneBody> contactableFeet)
   {
      this.contactableFeet = contactableFeet;
   }

   public List<Footstep> createFootsteps(double stepWidth, double stepLength, int numberOfSteps)
   {
      final ArrayList<Footstep> footsteps = new ArrayList<>();

      RobotSide currentSide = RobotSide.LEFT;

      FramePoint3D lastFootstepPosition = new FramePoint3D(contactableFeet.get(currentSide).getSoleFrame());
      lastFootstepPosition.changeFrame(worldFrame);

      for (int i = 0; i < numberOfSteps; i++)
      {
         currentSide = currentSide.getOppositeSide();
         double x = lastFootstepPosition.getX();
         if (i < numberOfSteps - 1)
            x += stepLength;
         double y = lastFootstepPosition.getY() + currentSide.negateIfRightSide(stepWidth);
         Footstep footstep = createFootstep(currentSide, x, y);
         footstep.getPosition(lastFootstepPosition);
         lastFootstepPosition.changeFrame(worldFrame);
         footsteps.add(footstep);
      }

      return footsteps;
   }

   private Footstep createFootstep(RobotSide robotSide, double x, double y)
   {
      return createFootstep(robotSide, new Point3D(x, y, 0.0), new Quaternion(0.0, 0.0, 0.0, 1.0));
   }

   public Footstep createFootstep(RobotSide robotSide, Point3D position, Quaternion orientation)
   {
      FramePose footstepPose = new FramePose();
      footstepPose.setPose(position, orientation);

      return createFootstep(robotSide, footstepPose);
   }

   public Footstep createFootstep(RobotSide robotSide, FramePose footstepPose)
   {
      RigidBody foot = contactableFeet.get(robotSide).getRigidBody();
      Footstep ret = new Footstep(foot, robotSide, footstepPose);
      ret.setPredictedContactPointsFromFramePoint2ds(contactableFeet.get(robotSide).getContactPoints2d());

      return ret;
   }

   public List<Footstep> convertToFootsteps(FootstepDataListMessage footstepDataListMessage)
   {
      return footstepDataListMessage.footstepDataList.stream().map(this::convertToFootstep).collect(Collectors.toList());
   }

   public Footstep convertToFootstep(FootstepDataMessage footstepDataMessage)
   {
      RobotSide robotSide = footstepDataMessage.getRobotSide();
      RigidBody foot = contactableFeet.get(robotSide).getRigidBody();
      FramePose solePose = new FramePose(worldFrame, footstepDataMessage.getLocation(), footstepDataMessage.getOrientation());
      Footstep footstep = new Footstep(foot, robotSide, solePose);
      if (footstepDataMessage.getPredictedContactPoints() != null && !footstepDataMessage.getPredictedContactPoints().isEmpty())
         footstep.setPredictedContactPointsFromPoint2ds(footstepDataMessage.getPredictedContactPoints());
      else
         footstep.setPredictedContactPointsFromFramePoint2ds(contactableFeet.get(robotSide).getContactPoints2d());

      return footstep;
   }
}
