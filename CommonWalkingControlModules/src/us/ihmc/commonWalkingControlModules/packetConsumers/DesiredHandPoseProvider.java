package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmJointTrajectoryPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPoseListPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandRotateAboutAxisPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryPacket;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class DesiredHandPoseProvider implements PacketConsumer<HandPosePacket>, HandPoseProvider
{
   private final SideDependentList<AtomicReference<HandPosePacket>> handPosePackets = new SideDependentList<AtomicReference<HandPosePacket>>();
   private final SideDependentList<AtomicReference<HandPoseListPacket>> handPoseListPackets = new SideDependentList<AtomicReference<HandPoseListPacket>>();
   private final SideDependentList<AtomicReference<HandRotateAboutAxisPacket>> handRotateAboutAxisPackets = new SideDependentList<AtomicReference<HandRotateAboutAxisPacket>>();
   private final AtomicReference<WholeBodyTrajectoryPacket> wholeBodyTrajectoryHandPoseListPackets = new AtomicReference<WholeBodyTrajectoryPacket>();
   private final SideDependentList<AtomicReference<StopAllTrajectoryMessage>> pausePackets = new SideDependentList<AtomicReference<StopAllTrajectoryMessage>>();
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
   private final SideDependentList<Boolean> controlHandAngleAboutAxis = new SideDependentList<>();
   private final SideDependentList<Double> graspOffsetsFromControlFrame = new SideDependentList<Double>();
   private final SideDependentList<Vector3d> forceConstraintsInWorld = new SideDependentList<Vector3d>();
   private final SideDependentList<Double> tangentialForce = new SideDependentList<Double>();
   private final SideDependentList<HandRotateAboutAxisPacket.DataType> handRotateAboutAxisDataType = new SideDependentList<HandRotateAboutAxisPacket.DataType>();

   private final SideDependentList<AtomicReference<boolean[]>> controlledAxes = new SideDependentList<>();
   private final SideDependentList<Double> percentOfTrajectoryWithOrientationControlled = new SideDependentList<Double>(1.0, 1.0);

   private double trajectoryTime = 1.0;

   private final ReferenceFrame chestFrame;

   private final int numberOfArmJoints;

   private final SideDependentList<ReferenceFrame> packetReferenceFrames;
   private final FullHumanoidRobotModel fullRobotModel;

   private final PacketConsumer<StopAllTrajectoryMessage> handPauseCommandConsumer;
   private final PacketConsumer<HandPoseListPacket> handPoseListConsumer;
   private final PacketConsumer<HandRotateAboutAxisPacket> handRotateAboutAxisConsumer;
   private final PacketConsumer<WholeBodyTrajectoryPacket> wholeBodyTrajectoryHandPoseListConsumer;
   private final PacketConsumer<ArmJointTrajectoryPacket> armJointTrajectoryPacketConsumer;
   private final HumanoidGlobalDataProducer globalDataProducer;

   public DesiredHandPoseProvider(CommonHumanoidReferenceFrames referenceFrames, FullHumanoidRobotModel fullRobotModel,
         SideDependentList<RigidBodyTransform> desiredHandPosesWithRespectToChestFrame, HumanoidGlobalDataProducer globalDataProducer)
   {
      this.fullRobotModel = fullRobotModel;
      this.globalDataProducer = globalDataProducer;
      chestFrame = referenceFrames.getMidFeetUnderPelvisFrame();
      packetReferenceFrames = new SideDependentList<ReferenceFrame>(chestFrame, chestFrame);
      numberOfArmJoints = ScrewTools.computeDegreesOfFreedom(ScrewTools.createJointPath(fullRobotModel.getChest(), fullRobotModel.getHand(RobotSide.LEFT)));

      for (RobotSide robotSide : RobotSide.values)
      {
         handPosePackets.put(robotSide, new AtomicReference<HandPosePacket>());
         pausePackets.put(robotSide, new AtomicReference<StopAllTrajectoryMessage>());
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
         forceConstraintsInWorld.put(robotSide, new Vector3d());

         controlledAxes.put(robotSide, new AtomicReference<boolean[]>(null));
      }

      handPauseCommandConsumer = new PacketConsumer<StopAllTrajectoryMessage>()
      {
         @Override
         public void receivedPacket(StopAllTrajectoryMessage object)
         {
            pausePackets.get(RobotSide.LEFT).set(object);
            pausePackets.get(RobotSide.RIGHT).set(object);
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
         controlledAxes.get(robotSide).set(object.getControlledOrientationAxes());
         percentOfTrajectoryWithOrientationControlled.put(robotSide, object.getPercentOfTrajectoryWithHandOrientationBeingControlled());

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
         controlHandAngleAboutAxis.put(robotSide, object.controlHandAngleAboutAxis());
         graspOffsetsFromControlFrame.put(robotSide, object.getGraspOffsetFromControlFrame());
         handRotateAboutAxisDataType.put(robotSide, object.getDataType());

         if (object.dataType == HandRotateAboutAxisPacket.DataType.ROTATE_ABOUT_AXIS_FORCE_CONTROLLED)
         {
            tangentialForce.put(robotSide, object.desiredTangentialForce);
            forceConstraintsInWorld.put(robotSide, object.forceConstraint);
         }
      }
   }

   @Override
   public void clear()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         handPosePackets.get(robotSide).set(null);
         handPoseListPackets.get(robotSide).set(null);
         handRotateAboutAxisPackets.get(robotSide).set(null);
         armJointTrajectoryPackets.get(robotSide).set(null);
      }
   }

   @Override
   public boolean checkForNewPose(RobotSide robotSide)
   {
      return handPosePackets.get(robotSide).get() != null;
   }

   @Override
   public boolean checkAndResetStopCommand(RobotSide robotSide)
   {
      return pausePackets.get(robotSide).getAndSet(null) != null;
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
      return armJointTrajectoryPackets.get(robotSide).get() != null || wholeBodyTrajectoryHandPoseListPackets.get() != null;
   }

   @Override
   public ArmJointTrajectoryPacket getArmJointTrajectoryPacket(RobotSide robotSide)
   {
      ArmJointTrajectoryPacket packet = null;

      if (armJointTrajectoryPackets.get(robotSide).get() != null)
      {
         packet = armJointTrajectoryPackets.get(robotSide).getAndSet(null);
      }

      if (wholeBodyTrajectoryHandPoseListPackets.get() != null)
      {
         if (robotSide.equals(RobotSide.LEFT))
         {
            WholeBodyTrajectoryPacket wholeBodyPacket = wholeBodyTrajectoryHandPoseListPackets.get();
            packet = wholeBodyPacket.leftArmTrajectory;
         }
         else if (robotSide.equals(RobotSide.RIGHT))
         {
            WholeBodyTrajectoryPacket wholeBodyPacket = wholeBodyTrajectoryHandPoseListPackets.getAndSet(null);
            packet = wholeBodyPacket.rightArmTrajectory;
         }
      }

      if (globalDataProducer != null && packet != null)
      {
         String errorMessage = PacketValidityChecker.validateArmJointTrajectoryPacket(packet);
         if (errorMessage != null)
         {
            globalDataProducer.notifyInvalidPacketReceived(HandPosePacket.class, errorMessage);
            System.out.println(errorMessage);
            return null;
         }
      }

      return packet;
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
   public double getGraspOffsetFromControlFrame(RobotSide robotSide)
   {
      updateFromNewestHandRotateAboutAxisPacket(robotSide);
      return graspOffsetsFromControlFrame.get(robotSide);
   }

   @Override
   public boolean controlHandAngleAboutAxis(RobotSide robotSide)
   {
      updateFromNewestHandRotateAboutAxisPacket(robotSide);
      return controlHandAngleAboutAxis.get(robotSide);
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

   public PacketConsumer<StopAllTrajectoryMessage> getHandPauseCommandConsumer()
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

   public PacketConsumer<WholeBodyTrajectoryPacket> getWholeBodyTrajectoryPacketConsumer()
   {
      return wholeBodyTrajectoryHandPoseListConsumer;
   }

   @Override
   public boolean[] getControlledOrientationAxes(RobotSide robotSide)
   {
      return controlledAxes.get(robotSide).get();
   }

   @Override
   public double getPercentOfTrajectoryWithOrientationBeingControlled(RobotSide robotSide)
   {
      return percentOfTrajectoryWithOrientationControlled.get(robotSide);
   }

   @Override
   public Vector3d getForceConstraint(RobotSide robotSide)
   {
      updateFromNewestHandRotateAboutAxisPacket(robotSide);
      return forceConstraintsInWorld.get(robotSide);
   }

   @Override
   public double getTangentialForce(RobotSide robotSide)
   {
      updateFromNewestHandRotateAboutAxisPacket(robotSide);
      return tangentialForce.get(robotSide);
   }

   @Override
   public HandRotateAboutAxisPacket.DataType checkHandRotateAboutAxisDataType(RobotSide robotSide)
   {
      updateFromNewestHandRotateAboutAxisPacket(robotSide);
      return handRotateAboutAxisDataType.get(robotSide);
   }
}
