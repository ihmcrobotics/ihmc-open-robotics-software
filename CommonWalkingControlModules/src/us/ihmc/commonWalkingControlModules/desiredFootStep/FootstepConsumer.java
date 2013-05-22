package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.FootstepData;
import us.ihmc.utilities.io.streamingData.StreamingDataConsumer;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class FootstepConsumer implements FootstepProvider, StreamingDataConsumer
{
   private final ConcurrentLinkedQueue<Footstep> footstepQueue = new ConcurrentLinkedQueue<Footstep>();
   private final long dataIdentifier;
   private final Collection<? extends ContactablePlaneBody> rigidBodyList;
   private int currentIndex = 0;

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
      
//      System.out.println("FootstepConsumer: consume: " + (++j) + " footsteps received, Ah Ah Ah!");
      FootstepData footstepData = (FootstepData) object;
      ContactablePlaneBody contactableBody = findContactableBodyByName(footstepData.getRigidBodyName());

      boolean trustHeight = true;
      String id = "footstep_" + currentIndex;
      FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame(), footstepData.getLocation(), footstepData.getOrientation());
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("poseReferenceFrame", framePose);
      ReferenceFrame soleReferenceFrame = FootstepUtils.createSoleFrame(poseReferenceFrame, contactableBody);

      List<FramePoint> expectedContactPoints = FootstepUtils.getContactPointsInFrame(contactableBody, soleReferenceFrame);


      Footstep footstep = new Footstep(id, contactableBody, poseReferenceFrame, soleReferenceFrame, expectedContactPoints, trustHeight);

//    System.out.println("footstep = " + footstep);
      footstepQueue.add(footstep);
      
      currentIndex++;
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

   public Footstep peek()
   {
      return footstepQueue.peek();
   }
   
   public Footstep peekPeek()
   {
      Iterator<Footstep> iterator = footstepQueue.iterator();
      
      if (iterator.hasNext()) 
      {
         iterator.next();
      }
      else
      {
         return null;
      }
      if (iterator.hasNext()) 
      {
         return iterator.next();
      }
      else
      {
         return null;
      }
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

   public int getNumberOfFootstepsToProvide()
   {
      return footstepQueue.size();
   }
}
