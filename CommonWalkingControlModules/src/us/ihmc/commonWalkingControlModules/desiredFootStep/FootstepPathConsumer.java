package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.SimpleTwoWaypointTrajectoryParameters;
import us.ihmc.utilities.math.trajectories.TrajectoryWaypointGenerationMethod;
import us.ihmc.utilities.math.trajectories.TwoWaypointTrajectoryUtils;
import us.ihmc.utilities.math.trajectories.providers.TrajectoryParameters;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.humanoidRobot.footstep.Footstep;

/**
 * User: Matt
 * Date: 1/18/13
 */
public class FootstepPathConsumer implements PacketConsumer<FootstepDataList>
{
   private boolean DEBUG = false;
   private FootstepPathCoordinator footstepPathCoordinator;
   private final SideDependentList<? extends ContactablePlaneBody> bipedFeet;
   private final HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters;

   public FootstepPathConsumer(SideDependentList<? extends ContactablePlaneBody> bipedFeet, FootstepPathCoordinator footstepPathCoordinator,
                               HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters)
   {
      this.footstepPathCoordinator = footstepPathCoordinator;
      this.bipedFeet = bipedFeet;
      this.mapFromFootstepsToTrajectoryParameters = mapFromFootstepsToTrajectoryParameters;
   }

   public void receivedPacket(FootstepDataList footstepList)
   {
      ArrayList<Footstep> footsteps = new ArrayList<Footstep>();
      
      TrajectoryParameters trajectoryParameters = null;
      if(footstepList.getTrajectoryWaypointGenerationMethod() != null)
      {
         if(footstepList.getTrajectoryWaypointGenerationMethod() == TrajectoryWaypointGenerationMethod.BY_BOX)
         {
            trajectoryParameters = new SimpleTwoWaypointTrajectoryParameters(TwoWaypointTrajectoryUtils.getFlatBoxFromSquare(footstepList.getTrajectoryBoxData()));
         }
         else
         {
            trajectoryParameters = new SimpleTwoWaypointTrajectoryParameters(footstepList.getTrajectoryWaypointGenerationMethod());
         }
      }
      else
         System.out.println("trajectory parameters are null");
      
      for (int i = 0; i < footstepList.size(); i++)
      {
         FootstepData footstepData = footstepList.get(i);
         String id = "footstep_" + i;
         
         ContactablePlaneBody contactableBody = bipedFeet.get(footstepData.getRobotSide());

         FramePose footstepPose = new FramePose(ReferenceFrame.getWorldFrame(), footstepData.getLocation(), footstepData.getOrientation());
         PoseReferenceFrame footstepPoseFrame = new PoseReferenceFrame("footstepPoseFrame", footstepPose);
         List<Point2d> contactPoints = footstepData.getPredictedContactPoints();
         if (contactPoints !=null && contactPoints.size() == 0)
            throw new RuntimeException("Cannot have an empty list of contact points in FootstepData. Should be null to use the default controller contact points.");
         Footstep footstep = new Footstep(id, contactableBody.getRigidBody(), footstepData.getRobotSide(), contactableBody.getSoleFrame(), footstepPoseFrame, true, contactPoints);
         
         footsteps.add(footstep);
         mapFromFootstepsToTrajectoryParameters.put(footstep, trajectoryParameters);

         if (DEBUG)
         {
            System.out.println("FootstepPathConsumer received " + footstep);
         }
      }
      
      footstepPathCoordinator.setSwingTime(footstepList.swingTime);
      footstepPathCoordinator.setTransferTime(footstepList.transferTime);
      footstepPathCoordinator.updatePath(footsteps);
   }

}