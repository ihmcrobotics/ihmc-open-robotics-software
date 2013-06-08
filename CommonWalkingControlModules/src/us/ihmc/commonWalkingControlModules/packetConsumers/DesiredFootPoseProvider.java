package us.ihmc.commonWalkingControlModules.packetConsumers;

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
   private final SideDependentList<FramePose> desiredFootPoses = new SideDependentList<FramePose>();
   private SideDependentList<Boolean> hasNewPose = new SideDependentList<Boolean>();

   public DesiredFootPoseProvider()
   {
      FramePose pose = new FramePose(ReferenceFrame.getWorldFrame());
      for (RobotSide robotSide : RobotSide.values)
      {
         hasNewPose.put(robotSide, false);
         // Just initializing to whatever...
         desiredFootPoses.put(robotSide, pose);
      }
   }

   public synchronized boolean checkForNewPose(RobotSide robotSide)
   {
      return hasNewPose.get(robotSide);
   }

   public synchronized FramePose getDesiredFootPose(RobotSide robotSide)
   {
      hasNewPose.put(robotSide, false);

      return desiredFootPoses.get(robotSide);
   }

   public synchronized void consumeObject(FootPosePacket object)
   {
      RobotSide robotSide = object.getRobotSide();
      hasNewPose.put(robotSide, true);

      FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), object.getPosition(), object.getOrientation());
      desiredFootPoses.put(robotSide, pose);
   }
}
