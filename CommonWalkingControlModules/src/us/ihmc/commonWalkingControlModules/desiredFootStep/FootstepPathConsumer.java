package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactableBody;
import us.ihmc.utilities.io.streamingData.AbstractStreamingDataConsumer;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import javax.vecmath.Point3d;
import java.util.ArrayList;
import java.util.Collection;

/**
 * User: Matt
 * Date: 1/18/13
 */
public class FootstepPathConsumer extends AbstractStreamingDataConsumer<ArrayList>
{
   private FootstepPathCoordinator footstepPathCoordinator;
   private final Collection<? extends ContactableBody> rigidBodyList;

   public FootstepPathConsumer(long dataIdentifier, Collection<? extends ContactableBody> rigidBodyList, FootstepPathCoordinator footstepPathCoordinator)
   {
      super(dataIdentifier, ArrayList.class);
      this.footstepPathCoordinator = footstepPathCoordinator;
      this.rigidBodyList = rigidBodyList;
   }

   protected void processPacket(ArrayList footstepList)
   {
      ArrayList<Footstep> footsteps = new ArrayList<Footstep>();
      for (Object footstepObject : footstepList)
      {
         FootstepData footstepData = (FootstepData) footstepObject;
         ContactableBody contactableBody = findContactableBodyByName(footstepData.getRigidBodyName());
         ArrayList<FramePoint> expectedContactPoints = new ArrayList<FramePoint>();
         for (int i = 0; i < footstepData.getExpectedContactPoints().size(); i++)
         {
            Point3d point3d = footstepData.getExpectedContactPoints().get(i);
            FramePoint framePoint = new FramePoint(contactableBody.getBodyFrame(), point3d);
            expectedContactPoints.add(framePoint);
         }
         Footstep footstep = new Footstep(contactableBody.getRigidBody(), new FramePose(ReferenceFrame.getWorldFrame(), footstepData.getLocation(), footstepData.getOrientation()), expectedContactPoints);
         footsteps.add(footstep);
      }
      footstepPathCoordinator.updatePath(footsteps);
   }

   private ContactableBody findContactableBodyByName(String rigidBodyName)
   {
      for (ContactableBody contactableBody : rigidBodyList)
      {
         if (contactableBody.getRigidBody().getName().equals(rigidBodyName))
         {
            return contactableBody;
         }
      }

      throw new RuntimeException("Rigid body not found: " + rigidBodyName);
   }
}
