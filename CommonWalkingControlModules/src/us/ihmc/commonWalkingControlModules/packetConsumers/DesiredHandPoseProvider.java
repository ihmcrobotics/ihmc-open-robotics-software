package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.manipulation.ArmJointTrajectoryPacket;
import us.ihmc.communication.packets.manipulation.HandPoseListPacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandRotateAboutAxisPacket;
import us.ihmc.communication.packets.manipulation.StopArmMotionPacket;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryPacket;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.ScrewTools;

public class DesiredHandPoseProvider implements PacketConsumer<HandPosePacket>, HandPoseProvider
{
   private final SideDependentList<AtomicReference<HandPosePacket>> handPosePackets = new SideDependentList<AtomicReference<HandPosePacket>>();
   private final SideDependentList<AtomicReference<HandPoseListPacket>> handPoseListPackets = new SideDependentList<AtomicReference<HandPoseListPacket>>();
   private final SideDependentList<AtomicReference<HandRotateAboutAxisPacket>> handRotateAboutAxisPackets = new SideDependentList<AtomicReference<HandRotateAboutAxisPacket>>();
   private final AtomicReference<WholeBodyTrajectoryPacket> wholeBodyTrajectoryHandPoseListPackets = new AtomicReference<WholeBodyTrajectoryPacket>();
   private final SideDependentList<AtomicReference<StopArmMotionPacket>> pausePackets = new SideDependentList<AtomicReference<StopArmMotionPacket>>();
   private final SideDependentList<AtomicReference<ArmJointTrajectoryPacket>> armJointTrajectoryPackets = new SideDependentList<AtomicReference<ArmJointTrajectoryPacket>>();
   
   private final SideDependentList<FramePose> homePositions = new SideDependentList<FramePose>();
   private final SideDependentList<FramePose> desiredHandPoses = new SideDependentList<FramePose>();
   private final SideDependentList<FramePose[]> desiredHandPosesList = new SideDependentList<FramePose[]>();
   private final SideDependentList<Map<OneDoFJoint, Double>> finalDesiredJointAngleMaps = new SideDependentList<Map<OneDoFJoint, Double>>();
   private final SideDependentList<Map<OneDoFJoint, double[]>> desiredJointAngleForWaypointTrajectoryMaps = new SideDependentList<>();
   private final SideDependentList<Map<OneDoFJoint, double[]>> desiredHandPoseListJointAngles = new SideDependentList<>();
   private final SideDependentList<Point3d> rotationAxisOriginsInWorld = new SideDependentList<Point3d>();
   private final SideDependentList<Vector3d> rotationAxesInWorld = new SideDependentList<Vector3d>();
   private final SideDependentList<Double> rotationAnglesRightHandRules = new SideDependentList<>();
   private double trajectoryTime = 1.0;

   private final ReferenceFrame chestFrame;

   private final int numberOfArmJoints;

   private final SideDependentList<ReferenceFrame> packetReferenceFrames;
   private final FullRobotModel fullRobotModel;

   private final PacketConsumer<StopArmMotionPacket> handPauseCommandConsumer;
   private final PacketConsumer<HandPoseListPacket> handPoseListConsumer;
   private final PacketConsumer<HandRotateAboutAxisPacket> handRotateAboutAxisConsumer;
   private final PacketConsumer<WholeBodyTrajectoryPacket> wholeBodyTrajectoryHandPoseListConsumer;
   private final PacketConsumer<ArmJointTrajectoryPacket> armJointTrajectoryPacketConsumer;
   private final GlobalDataProducer globalDataProducer;

   public DesiredHandPoseProvider(FullRobotModel fullRobotModel, SideDependentList<RigidBodyTransform> desiredHandPosesWithRespectToChestFrame,
         GlobalDataProducer globalDataProducer)
   {
      this.fullRobotModel = fullRobotModel;
      this.globalDataProducer = globalDataProducer;
      chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
      packetReferenceFrames = new SideDependentList<ReferenceFrame>(chestFrame, chestFrame);
      numberOfArmJoints = ScrewTools.computeDegreesOfFreedom(ScrewTools.createJointPath(fullRobotModel.getChest(), fullRobotModel.getHand(RobotSide.LEFT)));

      for (RobotSide robotSide : RobotSide.values)
      {
         handPosePackets.put(robotSide, new AtomicReference<HandPosePacket>());
         pausePackets.put(robotSide, new AtomicReference<StopArmMotionPacket>());
         handPoseListPackets.put(robotSide, new AtomicReference<HandPoseListPacket>());
         handRotateAboutAxisPackets.put(robotSide, new AtomicReference<HandRotateAboutAxisPacket>());
         armJointTrajectoryPackets.put(robotSide, new AtomicReference<ArmJointTrajectoryPacket>());

         homePositions.put(robotSide, new FramePose(chestFrame, desiredHandPosesWithRespectToChestFrame.get(robotSide)));
         desiredHandPoses.put(robotSide, homePositions.get(robotSide));
         finalDesiredJointAngleMaps.put(robotSide, new LinkedHashMap<OneDoFJoint, Double>());
         desiredJointAngleForWaypointTrajectoryMaps.put(robotSide, new LinkedHashMap<OneDoFJoint, double[]>());
         desiredHandPoseListJointAngles.put(robotSide, new LinkedHashMap<OneDoFJoint, double[]>());
         rotationAxisOriginsInWorld.put(robotSide, new Point3d());
         rotationAxesInWorld.put(robotSide, new Vector3d());
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

      handRotateAboutAxisConsumer = new PacketConsumer<HandRotateAboutAxisPacket>()
      {
         @Override
         public void receivedPacket(HandRotateAboutAxisPacket object)
         {
            handRotateAboutAxisPackets.get(object.getRobotSide()).set(object);
         }
      };

      wholeBodyTrajectoryHandPoseListConsumer = new PacketConsumer<WholeBodyTrajectoryPacket>()
      {
         @Override
         public void receivedPacket(WholeBodyTrajectoryPacket packet)
         {
            wholeBodyTrajectoryHandPoseListPackets.set(packet);
         }
      };
      
      armJointTrajectoryPacketConsumer = new PacketConsumer<ArmJointTrajectoryPacket>()
      {
         @Override
         public void receivedPacket(ArmJointTrajectoryPacket object)
         {
            armJointTrajectoryPackets.get(object.robotSide).set(object);
         }
      };
   }

   private void updateFromNewestHandPosePacket(RobotSide robotSide)
   {
      HandPosePacket object = handPosePackets.get(robotSide).getAndSet(null);

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
            ArmJointName[] armJointNames = fullRobotModel.getRobotSpecificJointNames().getArmJointNames();
            for (ArmJointName armJoint : armJointNames)
            {
               if (object.getJointAngles() != null && object.getJointAngles().length == armJointNames.length)
                  finalDesiredJointAngleMap.put(fullRobotModel.getArmJoint(robotSide, armJoint), object.getJointAngles()[++i]);
            }
         }
      }
   }

   private void updateFromNewestHandPoseListPacket(RobotSide robotSide)
   {
      HandPoseListPacket object = handPoseListPackets.get(robotSide).getAndSet(null);

      if (object != null)
      {
         trajectoryTime = object.getTrajectoryTime();

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
               throw new RuntimeException("Unknown frame");
            }

            Point3d[] desiredPositions = object.getPositions();
            Quat4d[] desiredOrientations = object.getOrientations();
            int numberOfPoses = desiredPositions.length;

            desiredHandPosesList.clear();
            desiredHandPosesList.put(robotSide, new FramePose[numberOfPoses]);
            FramePose[] desiredPoses = desiredHandPosesList.get(robotSide);

            for (int i = 0; i < numberOfPoses; i++)
            {
               desiredPoses[i] = new FramePose(ReferenceFrame.getWorldFrame(), desiredPositions[i], desiredOrientations[i]);
               desiredPoses[i].changeFrame(packetReferenceFrames.get(robotSide));
            }
         }
         else
         {
            int i = -1;
            for (ArmJointName armJoint : fullRobotModel.getRobotSpecificJointNames().getArmJointNames())
            {
               if (object.getJointAngles() != null)
               {
                  double[] jointAngles = object.getJointAngles()[++i];
                  desiredHandPoseListJointAngles.get(robotSide).put(fullRobotModel.getArmJoint(robotSide, armJoint), jointAngles);
               }
            }
         }
      }
   }

   private void updateFromNewestHandRotateAboutAxisPacket(RobotSide robotSide)
   {
      HandRotateAboutAxisPacket object = handRotateAboutAxisPackets.get(robotSide).getAndSet(null);

      if (object != null)
      {
         trajectoryTime = object.getTrajectoryTime();
         rotationAxisOriginsInWorld.get(robotSide).set(object.getRotationAxisOriginInWorld());
         rotationAxesInWorld.get(robotSide).set(object.getRotationAxisInWorld());
         rotationAnglesRightHandRules.put(robotSide, object.getRotationAngleRightHandRule());
      }
   }

   @Override
   public boolean checkForNewPose(RobotSide robotSide)
   {
      return handPosePackets.get(robotSide).get() != null;
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
   public boolean checkForNewRotateAboutAxisPacket(RobotSide robotSide)
   {
      return handRotateAboutAxisPackets.get(robotSide).get() != null;
   }
   
   @Override
   public boolean checkForNewArmJointTrajectory(RobotSide robotSide)
   {
      return armJointTrajectoryPackets.get(robotSide).get() != null;
   }
   
   @Override
   public ArmJointTrajectoryPacket getArmJointTrajectoryPacket(RobotSide robotSide)
   {
      return armJointTrajectoryPackets.get(robotSide).getAndSet(null);
   }
   
   @Override
   public void getPauseCommand(RobotSide robotSide)
   {
      pausePackets.get(robotSide).getAndSet(null);
   }

   @Override
   public HandPosePacket.DataType checkHandPosePacketDataType(RobotSide robotSide)
   {
      return handPosePackets.get(robotSide).get().getDataType();
   }

   @Override
   public HandPosePacket.DataType checkHandPoseListPacketDataType(RobotSide robotSide)
   {
      return handPoseListPackets.get(robotSide).get().getDataType();
   }

   @Override
   public boolean checkForHomePosition(RobotSide robotSide)
   {
      if (!checkForNewPose(robotSide))
         return false;

      if (!handPosePackets.get(robotSide).get().isToHomePosition())
         return false;

      HandPosePacket object = handPosePackets.get(robotSide).getAndSet(null);
      trajectoryTime = object.getTrajectoryTime();
      desiredHandPoses.put(robotSide, homePositions.get(robotSide));

      return true;
   }

   @Override
   public FramePose getDesiredHandPose(RobotSide robotSide)
   {
      updateFromNewestHandPosePacket(robotSide);
      return desiredHandPoses.get(robotSide);
   }

   @Override
   public FramePose[] getDesiredHandPoses(RobotSide robotSide)
   {
      updateFromNewestHandPoseListPacket(robotSide);
      return desiredHandPosesList.get(robotSide);
   }

   @Override
   public Map<OneDoFJoint, Double> getFinalDesiredJointAngleMaps(RobotSide robotSide)
   {
      updateFromNewestHandPosePacket(robotSide);
      return finalDesiredJointAngleMaps.get(robotSide);
   }

   @Override
   public Map<OneDoFJoint, double[]> getDesiredJointAngleForWaypointTrajectory(RobotSide robotSide)
   {
      updateFromNewestHandPoseListPacket(robotSide);
      return desiredHandPoseListJointAngles.get(robotSide);
   }

   @Override
   public Point3d getRotationAxisOriginInWorld(RobotSide robotSide)
   {
      updateFromNewestHandRotateAboutAxisPacket(robotSide);
      return rotationAxisOriginsInWorld.get(robotSide);
   }

   @Override
   public Vector3d getRotationAxisInWorld(RobotSide robotSide)
   {
      updateFromNewestHandRotateAboutAxisPacket(robotSide);
      return rotationAxesInWorld.get(robotSide);
   }

   @Override
   public double getRotationAngleRightHandRule(RobotSide robotSide)
   {
      updateFromNewestHandRotateAboutAxisPacket(robotSide);
      return rotationAnglesRightHandRules.get(robotSide);
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
      if (globalDataProducer != null)
      {
         String errorMessage = PacketValidityChecker.validateHandPosePacket(object, numberOfArmJoints);
         if (errorMessage != null)
         {
            globalDataProducer.notifyInvalidPacketReceived(HandPosePacket.class, errorMessage);
            return;
         }
      }
      handPosePackets.get(object.getRobotSide()).set(object);
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

   public PacketConsumer<HandRotateAboutAxisPacket> getHandRotateAboutAxisConsumer()
   {
      return handRotateAboutAxisConsumer;
   }
   
   public PacketConsumer<ArmJointTrajectoryPacket> getArmJointTrajectoryConsumer()
   {
      return armJointTrajectoryPacketConsumer;
   }

   @Override
   public boolean checkForNewWholeBodyPoseList(RobotSide robotSide)
   {
      if (wholeBodyTrajectoryHandPoseListPackets.get() != null)
      {
         return true;
      }
      else
      {
         return false;
      }
   }

   public PacketConsumer<WholeBodyTrajectoryPacket> getWholeBodyTrajectoryPacketConsumer()
   {
      return wholeBodyTrajectoryHandPoseListConsumer;
   }

   @Override
   public double[] getDesiredWholeBodyTrajectoryTimeArray()
   {
      return wholeBodyTrajectoryHandPoseListPackets.get().timeAtWaypoint;
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
