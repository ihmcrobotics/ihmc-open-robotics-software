package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.manipulation.HandPoseListPacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.StopArmMotionPacket;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryPacket;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class DesiredHandPoseProvider implements PacketConsumer<HandPosePacket>, HandPoseProvider
{
   private final SideDependentList<AtomicReference<HandPosePacket>> packets = new SideDependentList<AtomicReference<HandPosePacket>>();
   private final SideDependentList<AtomicReference<HandPoseListPacket>> handPoseListPackets = new SideDependentList<AtomicReference<HandPoseListPacket>>();
   private final AtomicReference<WholeBodyTrajectoryPacket> wholeBodyTrajectoryHandPoseListPackets = new AtomicReference<WholeBodyTrajectoryPacket>();
   private final SideDependentList<AtomicReference<StopArmMotionPacket>> pausePackets = new SideDependentList<AtomicReference<StopArmMotionPacket>>();

   private final SideDependentList<FramePose> homePositions = new SideDependentList<FramePose>();
   private final SideDependentList<FramePose> desiredHandPoses = new SideDependentList<FramePose>();
   private final SideDependentList<Map<OneDoFJoint, Double>> finalDesiredJointAngleMaps = new SideDependentList<Map<OneDoFJoint, Double>>();
   private final SideDependentList<Map<OneDoFJoint, double[]>> desiredJointAngleForWaypointTrajectoryMaps = new SideDependentList<>();
   private double trajectoryTime = 1.0;

   private final ReferenceFrame chestFrame;

   private final SideDependentList<ReferenceFrame> packetReferenceFrames;
   private final FullRobotModel fullRobotModel;
   private final PacketConsumer<StopArmMotionPacket> handPauseCommandConsumer;

   private final PacketConsumer<HandPoseListPacket> handPoseListConsumer;

   private final PacketConsumer<WholeBodyTrajectoryPacket> wholeBodyTrajectoryHandPoseListConsumer;

   public PacketConsumer<WholeBodyTrajectoryPacket> getWholeBodyTrajectoryPacketConsumer()
   {
      return wholeBodyTrajectoryHandPoseListConsumer;
   }

   public DesiredHandPoseProvider(FullRobotModel fullRobotModel, SideDependentList<RigidBodyTransform> desiredHandPosesWithRespectToChestFrame)
   {
      this.fullRobotModel = fullRobotModel;
      chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
      packetReferenceFrames = new SideDependentList<ReferenceFrame>(chestFrame, chestFrame);

      for (RobotSide robotSide : RobotSide.values)
      {
         packets.put(robotSide, new AtomicReference<HandPosePacket>());
         pausePackets.put(robotSide, new AtomicReference<StopArmMotionPacket>());
         handPoseListPackets.put(robotSide, new AtomicReference<HandPoseListPacket>());

         homePositions.put(robotSide, new FramePose(chestFrame, desiredHandPosesWithRespectToChestFrame.get(robotSide)));

         desiredHandPoses.put(robotSide, homePositions.get(robotSide));

         finalDesiredJointAngleMaps.put(robotSide, new LinkedHashMap<OneDoFJoint, Double>());
         desiredJointAngleForWaypointTrajectoryMaps.put(robotSide, new LinkedHashMap<OneDoFJoint, double[]>());
      }

      handPauseCommandConsumer = new PacketConsumer<StopArmMotionPacket>()
      {
         @Override
         public void receivedPacket(StopArmMotionPacket object)
         {
            pausePackets.get(object.getRobotSide()).set(object);
         }
      };

      handPoseListConsumer = new PacketConsumer<HandPoseListPacket>()
      {
         @Override
         public void receivedPacket(HandPoseListPacket object)
         {
            handPoseListPackets.get(object.getRobotSide()).set(object);
         }
      };

      wholeBodyTrajectoryHandPoseListConsumer = new PacketConsumer<WholeBodyTrajectoryPacket>()
      {
         @Override
         public void receivedPacket(WholeBodyTrajectoryPacket packet)
         {
            System.out.println("DesiredHandPoseProvider: PACKET received");
            wholeBodyTrajectoryHandPoseListPackets.set(packet);
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

   @Override
   public boolean checkForNewPose(RobotSide robotSide)
   {
      return packets.get(robotSide).get() != null;
   }

   @Override
   public boolean checkForNewPauseCommand(RobotSide robotSide)
   {
      return pausePackets.get(robotSide).get() != null;
   }

   @Override
   public boolean checkForNewPoseList(RobotSide robotSide)
   {
      return handPoseListPackets.get(robotSide).get() != null;
   }

   @Override
   public void getPauseCommand(RobotSide robotSide)
   {
      pausePackets.get(robotSide).getAndSet(null);
   }

   @Override
   public HandPosePacket.DataType checkPacketDataType(RobotSide robotSide)
   {
      return packets.get(robotSide).get().getDataType();
   }

   @Override
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

   @Override
   public FramePose getDesiredHandPose(RobotSide robotSide)
   {
      updateFromNewestPacket(robotSide);

      return desiredHandPoses.get(robotSide);
   }

   @Override
   public Map<OneDoFJoint, Double> getFinalDesiredJointAngleMaps(RobotSide robotSide)
   {
      updateFromNewestPacket(robotSide);

      return finalDesiredJointAngleMaps.get(robotSide);
   }

   @Override
   public Map<OneDoFJoint, double[]> getDesiredJointAngleForWaypointTrajectory(RobotSide robotSide)
   {
      HandPoseListPacket object = handPoseListPackets.get(robotSide).getAndSet(null);
      if (object == null)
         return null;

      trajectoryTime = object.getTrajectoryTime();

      int i = -1;
      Map<OneDoFJoint, double[]> desiredJointAngles = desiredJointAngleForWaypointTrajectoryMaps.get(robotSide);
      for (ArmJointName armJoint : fullRobotModel.getRobotSpecificJointNames().getArmJointNames())
      {
         if (object.getJointAngles() != null)
         {
            double[] jointAngles = object.getJointAngles()[++i];
            desiredJointAngles.put(fullRobotModel.getArmJoint(robotSide, armJoint), jointAngles);
         }
      }

      return desiredJointAngles;
   }

   @Override
   public ReferenceFrame getDesiredReferenceFrame(RobotSide robotSide)
   {
      return packetReferenceFrames.get(robotSide);
   }

   @Override
   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   @Override
   public void receivedPacket(HandPosePacket object)
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

   public PacketConsumer<StopArmMotionPacket> getHandPauseCommandConsumer()
   {
      return handPauseCommandConsumer;
   }

   public PacketConsumer<HandPoseListPacket> getHandPoseListConsumer()
   {
      return handPoseListConsumer;
   }


   @Override
   public boolean checkForNewWholeBodyPoseList(RobotSide robotSide)
   {
      if (wholeBodyTrajectoryHandPoseListPackets.get() != null)
      {
         return wholeBodyTrajectoryHandPoseListPackets.get().hasArmTrajectory(robotSide);
      }
      else
      {
         return false;
      }
   }

   @Override
   public double[] getDesiredWholeBodyTrajectoryTimeArray()
   {
      return wholeBodyTrajectoryHandPoseListPackets.get().timeSincePrevious;
   }

   @Override
   public double[][] getDesiredWholeBodyTrajectoryPositionArray(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
      {
         return wholeBodyTrajectoryHandPoseListPackets.get().leftArmJointAngle;
      }
      else
      {
         return wholeBodyTrajectoryHandPoseListPackets.get().rightArmJointAngle;
      }
   }

   @Override
   public double[][] getDesiredWholeBodyTrajectoryVelocityArray(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
      {
         return wholeBodyTrajectoryHandPoseListPackets.get().leftArmJointVelocity;
      }
      else
      {
         return wholeBodyTrajectoryHandPoseListPackets.get().rightArmJointVelocity;
      }
   }

   @Override
   public void setWholeBodyTrajectoryPacketAtomicReferenceToNull(RobotSide robotSide)
   {
      if (robotSide == RobotSide.RIGHT)
      {
         wholeBodyTrajectoryHandPoseListPackets.set(null);
      }
   }

}
