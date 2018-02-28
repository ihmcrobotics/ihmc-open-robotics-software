package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.footstep.Footstep;
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
      FramePose3D footstepPose = new FramePose3D();
      footstepPose.set(position, orientation);

      return createFootstep(robotSide, footstepPose);
   }

   public Footstep createFootstep(RobotSide robotSide, FramePose3D footstepPose)
   {
      RigidBody foot = contactableFeet.get(robotSide).getRigidBody();
      Footstep ret = new Footstep(robotSide, footstepPose);
      ret.setPredictedContactPoints(contactableFeet.get(robotSide).getContactPoints2d());

      return ret;
   }

   public List<Footstep> convertToFootsteps(FootstepDataListMessage footstepDataListMessage)
   {
      return footstepDataListMessage.footstepDataList.stream().map(this::convertToFootstep).collect(Collectors.toList());
   }

   public Footstep convertToFootstep(FootstepDataMessage footstepDataMessage)
   {
      RobotSide robotSide = RobotSide.fromByte(footstepDataMessage.getRobotSide());
      RigidBody foot = contactableFeet.get(robotSide).getRigidBody();
      FramePose3D solePose = new FramePose3D(worldFrame, footstepDataMessage.getLocation(), footstepDataMessage.getOrientation());
      Footstep footstep = new Footstep(robotSide, solePose);
      if (footstepDataMessage.getPredictedContactPoints() != null && !footstepDataMessage.getPredictedContactPoints().isEmpty())
         footstep.setPredictedContactPoints(footstepDataMessage.getPredictedContactPoints());
      else
         footstep.setPredictedContactPoints(contactableFeet.get(robotSide).getContactPoints2d());

      return footstep;
   }
}
