package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.FootstepData;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.FootstepDataList;
import us.ihmc.commonWalkingControlModules.trajectories.SimpleTwoWaypointTrajectoryParameters;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;

import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryParameters;

/**
 * User: Matt
 * Date: 1/18/13
 */
public class FootstepPathConsumer implements ObjectConsumer<FootstepDataList>
{
   private boolean DEBUG = false;
   private FootstepPathCoordinator footstepPathCoordinator;
   private final Collection<? extends ContactablePlaneBody> rigidBodyList;
   private final HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters;

   public FootstepPathConsumer(Collection<? extends ContactablePlaneBody> rigidBodyList, FootstepPathCoordinator footstepPathCoordinator,
                               HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters)
   {
      this.footstepPathCoordinator = footstepPathCoordinator;
      this.rigidBodyList = rigidBodyList;
      this.mapFromFootstepsToTrajectoryParameters = mapFromFootstepsToTrajectoryParameters;
   }

   public void consumeObject(FootstepDataList footstepList)
   {
      ArrayList<Footstep> footsteps = new ArrayList<Footstep>();
      for (FootstepData footstepData : footstepList)
      {
         ContactablePlaneBody contactableBody = findContactableBodyByName(footstepData.getRigidBodyName());

         FramePose footstepPose = new FramePose(ReferenceFrame.getWorldFrame(), footstepData.getLocation(), footstepData.getOrientation());
         PoseReferenceFrame footstepPoseFrame = new PoseReferenceFrame("footstepPoseFrame", footstepPose);
         ReferenceFrame soleReferenceFrame = FootstepUtils.createSoleFrame(footstepPoseFrame, contactableBody);

         
         FootstepUtils.getContactPointsInFrame(contactableBody, soleReferenceFrame);
         List<FramePoint> expectedContactPoints = FootstepUtils.getContactPointsInFrame(contactableBody, soleReferenceFrame);
         

         Footstep footstep = new Footstep(footstepData.getId(), contactableBody, footstepPoseFrame, soleReferenceFrame, expectedContactPoints, true);
         TrajectoryParameters trajectoryParameters = null;

         if (footstepData.getTrajectoryWaypointGenerationMethod() != null)
         {
            switch (footstepData.getTrajectoryWaypointGenerationMethod())
            {
               case BY_BOX :
                  trajectoryParameters = new SimpleTwoWaypointTrajectoryParameters(footstepData.getTrajectoryBox());
                  break;
                default:
                  trajectoryParameters = new SimpleTwoWaypointTrajectoryParameters(footstepData.getTrajectoryWaypointGenerationMethod());
                  break;
            }
         }
         
         footsteps.add(footstep);
         mapFromFootstepsToTrajectoryParameters.put(footstep, trajectoryParameters);

         if (DEBUG)
         {
            System.out.println("FootstepPathConsumer received " + footstep);
         }
      }

      footstepPathCoordinator.updatePath(footsteps);
   }

   private ContactablePlaneBody findContactableBodyByName(String rigidBodyName)
   {
      for (ContactablePlaneBody contactableBody : rigidBodyList)
      {
         if (contactableBody.getRigidBody().getName().equals(rigidBodyName))
         {
            return contactableBody;
         }
      }

      throw new RuntimeException("Rigid body not found: " + rigidBodyName);
   }

}
