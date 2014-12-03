package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.packets.manipulation.StopArmMotionPacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.net.ObjectConsumer;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class DesiredHandPoseProvider implements ObjectConsumer<HandPosePacket>, HandPoseProvider
{
   private final SideDependentList<AtomicReference<HandPosePacket>> packets = new SideDependentList<AtomicReference<HandPosePacket>>();
   private final SideDependentList<AtomicReference<StopArmMotionPacket>> pausePackets = new SideDependentList<AtomicReference<StopArmMotionPacket>>();

   private final SideDependentList<FramePose> homePositions = new SideDependentList<FramePose>();
   private final SideDependentList<FramePose> desiredHandPoses = new SideDependentList<FramePose>();
   private final SideDependentList<Map<OneDoFJoint, Double>> finalDesiredJointAngleMaps = new SideDependentList<Map<OneDoFJoint, Double>>();
   private double trajectoryTime = 1.0;

   private final ReferenceFrame chestFrame;

   private final SideDependentList<ReferenceFrame> packetReferenceFrames;
   private final FullRobotModel fullRobotModel;
   private final ObjectConsumer<StopArmMotionPacket> handPauseCommandConsumer;

   public DesiredHandPoseProvider(FullRobotModel fullRobotModel, SideDependentList<RigidBodyTransform> desiredHandPosesWithRespectToChestFrame)
   {
      this.fullRobotModel = fullRobotModel;
      chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
      packetReferenceFrames = new SideDependentList<ReferenceFrame>(chestFrame, chestFrame);

      for (RobotSide robotSide : RobotSide.values)
      {
         packets.put(robotSide, new AtomicReference<HandPosePacket>());
         pausePackets.put(robotSide, new AtomicReference<StopArmMotionPacket>());

         homePositions.put(robotSide, new FramePose(chestFrame, desiredHandPosesWithRespectToChestFrame.get(robotSide)));

         desiredHandPoses.put(robotSide, homePositions.get(robotSide));

         finalDesiredJointAngleMaps.put(robotSide, new LinkedHashMap<OneDoFJoint, Double>());
      }

      handPauseCommandConsumer = new ObjectConsumer<StopArmMotionPacket>()
      {
         public void consumeObject(StopArmMotionPacket object)
         {
            pausePackets.get(object.getRobotSide()).set(object);
         }
      };
   }

   private void updateFromNewestPacket(RobotSide robotSide)
   {
      HandPosePacket object = packets.get(robotSide).getAndSet(null);

      if (object != null)
      {
         trajectoryTime = object.getTrajectoryTime();

         if (object.isToHomePosition())
         {
            desiredHandPoses.put(robotSide, homePositions.get(robotSide));
         }
         else
         {
            if (object.getDataType() == HandPosePacket.DataType.HAND_POSE)
            {
               switch (object.getReferenceFrame())
               {
               case WORLD:
                  packetReferenceFrames.put(robotSide, ReferenceFrame.getWorldFrame());

                  break;

               case CHEST:
                  packetReferenceFrames.put(robotSide, chestFrame);

                  break;

               default:
                  throw new RuntimeException("Unkown frame");
               }

               FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), object.getPosition(), object.getOrientation());
               pose.changeFrame(packetReferenceFrames.get(robotSide));
               desiredHandPoses.put(robotSide, pose);
            }

            Map<OneDoFJoint, Double> finalDesiredJointAngleMap = finalDesiredJointAngleMaps.get(robotSide);

            int i = -1;
            for (ArmJointName armJoint : fullRobotModel.getRobotSpecificJointNames().getArmJointNames())
            {
               if (object.getJointAngles() != null)
                  finalDesiredJointAngleMap.put(fullRobotModel.getArmJoint(robotSide, armJoint), object.getJointAngles()[++i]);
            }
         }
      }
   }

   public boolean checkForNewPose(RobotSide robotSide)
   {
      return packets.get(robotSide).get() != null;
   }
   
   public boolean checkForNewPauseCommand(RobotSide robotSide)
   {
      return pausePackets.get(robotSide).get() != null;
   }
   
   public void getPauseCommand(RobotSide robotSide)
   {
      pausePackets.get(robotSide).getAndSet(null);
   }

   public HandPosePacket.DataType checkPacketDataType(RobotSide robotSide)
   {
      return packets.get(robotSide).get().getDataType();
   }

   public boolean checkForHomePosition(RobotSide robotSide)
   {
      if (!checkForNewPose(robotSide))
         return false;

      if (!packets.get(robotSide).get().isToHomePosition())
         return false;

      HandPosePacket object = packets.get(robotSide).getAndSet(null);
      trajectoryTime = object.getTrajectoryTime();
      desiredHandPoses.put(robotSide, homePositions.get(robotSide));

      return true;
   }

   public FramePose getDesiredHandPose(RobotSide robotSide)
   {
      updateFromNewestPacket(robotSide);

      return desiredHandPoses.get(robotSide);
   }

   public Map<OneDoFJoint, Double> getFinalDesiredJointAngleMaps(RobotSide robotSide)
   {
      updateFromNewestPacket(robotSide);

      return finalDesiredJointAngleMaps.get(robotSide);
   }

   public ReferenceFrame getDesiredReferenceFrame(RobotSide robotSide)
   {
      return packetReferenceFrames.get(robotSide);
   }

   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   public void consumeObject(HandPosePacket object)
   {
      packets.get(object.getRobotSide()).set(object);
   }

   public EnumMap<ArmJointName, Double> getFinalDesiredJointAngleEnumMap(RobotSide robotSide)
   {
      EnumMap<ArmJointName, Double> ret = new EnumMap<ArmJointName, Double>(ArmJointName.class);

      for (ArmJointName armJoint : fullRobotModel.getRobotSpecificJointNames().getArmJointNames())
      {
         ret.put(armJoint, finalDesiredJointAngleMaps.get(robotSide).get(fullRobotModel.getArmJoint(robotSide, armJoint)));
      }

      return ret;
   }

   public ObjectConsumer<StopArmMotionPacket> getHandPauseCommandConsumer()
   {
      return handPauseCommandConsumer;
   }
}
