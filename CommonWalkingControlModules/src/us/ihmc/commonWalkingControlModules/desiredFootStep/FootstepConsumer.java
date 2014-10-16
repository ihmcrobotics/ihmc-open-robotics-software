package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.Iterator;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.io.streamingData.StreamingDataConsumer;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.humanoidRobot.footstep.Footstep;

public class FootstepConsumer implements FootstepProvider, StreamingDataConsumer, ObjectConsumer<FootstepData>
{
   private final ConcurrentLinkedQueue<Footstep> footstepQueue = new ConcurrentLinkedQueue<Footstep>();
   private final long dataIdentifier;
   private final SideDependentList<? extends ContactablePlaneBody> rigidBodyList;
   private int currentIndex = 0;

   public FootstepConsumer(long dataIdentifier, SideDependentList<? extends ContactablePlaneBody> bipedFeet)
   {
      this.dataIdentifier = dataIdentifier;
      this.rigidBodyList = bipedFeet;
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

      FootstepData footstepData = (FootstepData) object;
      ContactablePlaneBody contactableBody = rigidBodyList.get(footstepData.getRobotSide());

      boolean trustHeight = true;
      String id = "footstep_" + currentIndex;
      FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame(), footstepData.getLocation(), footstepData.getOrientation());
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("poseReferenceFrame", framePose);
      Footstep footstep = new Footstep(id, contactableBody.getRigidBody(), footstepData.getRobotSide(), contactableBody.getSoleFrame(), poseReferenceFrame, trustHeight);

      footstepQueue.add(footstep);

      currentIndex++;
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

   public boolean isBlindWalking()
   {
      return false;
   }

   public void consumeObject(FootstepData object)
   {
      // TODO Auto-generated method stub
   }
}