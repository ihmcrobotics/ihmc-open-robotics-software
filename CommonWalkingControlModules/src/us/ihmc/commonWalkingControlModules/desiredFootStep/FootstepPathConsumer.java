package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
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
   private boolean DEBUG = false;
   private FootstepPathCoordinator footstepPathCoordinator;
   private final Collection<? extends ContactablePlaneBody> rigidBodyList;

   public FootstepPathConsumer(long dataIdentifier, Collection<? extends ContactablePlaneBody> rigidBodyList, FootstepPathCoordinator footstepPathCoordinator)
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
         ContactablePlaneBody contactableBody = findContactableBodyByName(footstepData.getRigidBodyName());
         ArrayList<FramePoint> expectedContactPoints = new ArrayList<FramePoint>();
         for (int i = 0; i < footstepData.getExpectedContactPoints().size(); i++)
         {
            Point3d point3d = footstepData.getExpectedContactPoints().get(i);
            FramePoint framePoint = new FramePoint(contactableBody.getBodyFrame(), point3d);
            expectedContactPoints.add(framePoint);
         }
         Footstep footstep = new Footstep(footstepData.getId(), contactableBody, new FramePose(ReferenceFrame.getWorldFrame(), footstepData.getLocation(), footstepData.getOrientation()), expectedContactPoints, footstepData.getTrustHeight());
         footsteps.add(footstep);
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
