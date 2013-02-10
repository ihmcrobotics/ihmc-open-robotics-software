package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.Collection;
import java.util.concurrent.ConcurrentLinkedQueue;

import javax.vecmath.Point3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.utilities.io.streamingData.StreamingDataConsumer;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class FootstepConsumer implements FootstepProvider, StreamingDataConsumer
{
   private final ConcurrentLinkedQueue<Footstep> footstepQueue = new ConcurrentLinkedQueue<Footstep>();
   private final long dataIdentifier;
   private final Collection<? extends ContactablePlaneBody> rigidBodyList;
   int j=0;

   public FootstepConsumer(long dataIdentifier, Collection<? extends ContactablePlaneBody> rigidBodyList)
   {
      this.dataIdentifier = dataIdentifier;
      this.rigidBodyList = rigidBodyList;
   }

   public boolean canHandle(Object object)
   {
      return object instanceof FootstepData;
   }

   public void consume(long dataIdentifier, Object object)
   {
      if (dataIdentifier != this.dataIdentifier)
      {
         throw new RuntimeException("Wrong data identifier: " + dataIdentifier + ". Expected: " + this.dataIdentifier);
      }
      System.out.println("FootstepConsumer: consume: "+(++j)+" footsteps received, Ah Ah Ah!");
      FootstepData footstepData = (FootstepData) object;
      ContactablePlaneBody contactableBody = findContactableBodyByName(footstepData.getRigidBodyName());
      ArrayList<FramePoint> expectedContactPoints = new ArrayList<FramePoint>();
      for (int i = 0; i < footstepData.getExpectedContactPoints().size(); i++)
      {
         Point3d point3d = footstepData.getExpectedContactPoints().get(i);
         FramePoint framePoint = new FramePoint(contactableBody.getBodyFrame(), point3d);
         expectedContactPoints.add(framePoint);
      }
      
      boolean trustHeight = footstepData.getTrustHeight();
      Footstep footstep = new Footstep(contactableBody.getRigidBody(), new FramePose(ReferenceFrame.getWorldFrame(), footstepData.getLocation(), footstepData.getOrientation()), expectedContactPoints, trustHeight);
      System.out.println("footstep = " + footstep);
      footstepQueue.add(footstep);
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

   public Footstep poll()
   {
      return footstepQueue.poll();
   }

   public boolean isEmpty()
   {
      return footstepQueue.isEmpty();
   }

   public void notifyComplete()
   {
      System.out.println("FootstepConsumer: notifyComplete not implemented in FootstepConsumer");
   }

   public long getDataIdentifier()
   {
      return dataIdentifier;
   }
}
