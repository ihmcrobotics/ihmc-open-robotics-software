package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.Iterator;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.communication.streamingData.StreamingDataConsumer;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PoseReferenceFrame;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;

public class FootstepConsumer implements FootstepProvider, StreamingDataConsumer, PacketConsumer<FootstepData>
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

   @Override
   public boolean canHandle(Object object)
   {
      return object instanceof FootstepData;
   }

   @Override
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
      footstep.trajectoryType = footstepData.getTrajectoryType();
      footstep.swingHeight = footstepData.swingHeight;

      footstepQueue.add(footstep);

      currentIndex++;
   }

   @Override
   public Footstep poll()
   {
      return footstepQueue.poll();
   }

   @Override
   public Footstep peek()
   {
      return footstepQueue.peek();
   }

   @Override
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

   @Override
   public boolean isEmpty()
   {
      return footstepQueue.isEmpty();
   }

   @Override
   public void notifyComplete(FramePose actualFootPoseInWorld)
   {
      System.out.println("FootstepConsumer: notifyComplete not implemented in FootstepConsumer");
   }

   @Override
   public void notifyWalkingComplete()
   {
      System.out.println("FootstepConsumer: notifyWalkingComplete not implemented in FootstepConsumer");
   }

   @Override
   public long getDataIdentifier()
   {
      return dataIdentifier;
   }

   @Override
   public int getNumberOfFootstepsToProvide()
   {
      return footstepQueue.size();
   }

   @Override
   public boolean isBlindWalking()
   {
      return false;
   }

   @Override
   public void receivedPacket(FootstepData object)
   {
      // TODO Auto-generated method stub
   }

   @Override
   public boolean isPaused()
   {
      return false;
   }

   @Override
   public void cancelPlan()
   {
      footstepQueue.clear();
   }
}