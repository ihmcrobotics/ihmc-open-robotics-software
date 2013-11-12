package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commonWalkingControlModules.packets.FootPosePacket;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;

/**
 * Created with IntelliJ IDEA.
 * User: pneuhaus
 * Date: 5/22/13
 * Time: 11:45 AM
 * To change this template use File | Settings | File Templates.
 */

public class DesiredFootPoseProvider implements ObjectConsumer<FootPosePacket>
{
   private final SideDependentList<AtomicReference<FootPosePacket>> footPosePacket = new SideDependentList<AtomicReference<FootPosePacket>>();
   
   private final SideDependentList<FramePose> desiredFootPoses = new SideDependentList<FramePose>();

   public DesiredFootPoseProvider()
   {
      FramePose pose = new FramePose(ReferenceFrame.getWorldFrame());
      for (RobotSide robotSide : RobotSide.values)
      {
         footPosePacket.put(robotSide, new AtomicReference<FootPosePacket>());
         desiredFootPoses.put(robotSide, pose);
      }
   }

   public boolean checkForNewPose(RobotSide robotSide)
   {
      return footPosePacket.get(robotSide).get() != null;
   }

   public FramePose getDesiredFootPose(RobotSide robotSide)
   {
      FootPosePacket object = footPosePacket.get(robotSide).getAndSet(null);
      if(object != null)
      {         
         
         FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), object.getPosition(), object.getOrientation());
         desiredFootPoses.put(robotSide, pose);
      }
      
      return desiredFootPoses.get(robotSide);
   }

   public void consumeObject(FootPosePacket object)
   {
      footPosePacket.get(object.getRobotSide()).set(object);
   }
}
