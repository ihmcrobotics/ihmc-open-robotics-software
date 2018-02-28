package us.ihmc.humanoidRobotics.communication.util;

import us.ihmc.communication.packets.BoundingBoxesPacket;
import us.ihmc.communication.packets.IMUPacket;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.ObjectDetectorResultPacket;
import us.ihmc.communication.packets.PlanarRegionMessage;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.Polygon2DMessage;
import us.ihmc.communication.packets.SpatialVectorMessage;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.EuclideanTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.EuclideanTrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.JointspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.SO3TrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.SO3TrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryPoint1DMessage;
import us.ihmc.humanoidRobotics.communication.packets.heightQuadTree.HeightQuadTreeLeafMessage;
import us.ihmc.humanoidRobotics.communication.packets.heightQuadTree.HeightQuadTreeMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.RigidBodyExplorationConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WaypointBasedTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.momentum.CenterOfMassTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.momentum.MomentumTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.AdjustFootstepMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPathPlanPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.NeckTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.SnapFootstepPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.SpineTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager.ChestHybridJointspaceTaskspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager.HandHybridJointspaceTaskspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager.HeadHybridJointspaceTaskspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.idl.TempPreallocatedList;
import us.ihmc.idl.TempPreallocatedList.ListAllocator;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;

/**
 * This class provides the tools to trim down the size of messages before sending them on the
 * network. This won't be useful once using DDS, but it is more likely needed for Kryo to prevent it
 * from sending unused data stored in PreallocatedList.
 */
public final class MessageTrimmingTools
{
   public static interface MessageTrimmer<T>
   {
      T trim(T messageToTrim);
   }

   public static MessageTrimmer<AdjustFootstepMessage> createAdjustFootstepMessageTrimmer()
   {
      return new MessageTrimmer<AdjustFootstepMessage>()
      {
         @Override
         public AdjustFootstepMessage trim(AdjustFootstepMessage messageToTrim)
         {
            AdjustFootstepMessage trimmed = new AdjustFootstepMessage();
            trimmed.predictedContactPoints = new TempPreallocatedList<>(Point2D.class, Point2D::new, messageToTrim.predictedContactPoints.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<ArmTrajectoryMessage> createArmTrajectoryMessageTrimmer()
   {
      MessageTrimmer<JointspaceTrajectoryMessage> messageTrimmer = createJointspaceTrajectoryMessageTrimmer();

      return new MessageTrimmer<ArmTrajectoryMessage>()
      {
         @Override
         public ArmTrajectoryMessage trim(ArmTrajectoryMessage messageToTrim)
         {
            ArmTrajectoryMessage trimmed = new ArmTrajectoryMessage();
            trimmed.jointspaceTrajectory = messageTrimmer.trim(messageToTrim.jointspaceTrajectory);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<BoundingBoxesPacket> createBoundingBoxesPacketTrimmer()
   {
      return new MessageTrimmer<BoundingBoxesPacket>()
      {
         @Override
         public BoundingBoxesPacket trim(BoundingBoxesPacket messageToTrim)
         {
            BoundingBoxesPacket trimmed = new BoundingBoxesPacket();
            trimmed.labels = new TempPreallocatedList<>(StringBuilder.class, StringBuilder::new, messageToTrim.labels.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<CapturabilityBasedStatus> createCapturabilityBasedStatusTrimmer()
   {
      return new MessageTrimmer<CapturabilityBasedStatus>()
      {
         @Override
         public CapturabilityBasedStatus trim(CapturabilityBasedStatus messageToTrim)
         {
            CapturabilityBasedStatus trimmed = new CapturabilityBasedStatus();
            trimmed.leftFootSupportPolygon = new TempPreallocatedList<>(Point2D.class, Point2D::new, messageToTrim.leftFootSupportPolygon.size());
            trimmed.rightFootSupportPolygon = new TempPreallocatedList<>(Point2D.class, Point2D::new, messageToTrim.rightFootSupportPolygon.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<CenterOfMassTrajectoryMessage> createCenterOfMassTrajectoryMessageTrimmer()
   {
      MessageTrimmer<EuclideanTrajectoryMessage> messageTrimmer = createEuclideanTrajectoryMessageTrimmer();

      return new MessageTrimmer<CenterOfMassTrajectoryMessage>()
      {
         @Override
         public CenterOfMassTrajectoryMessage trim(CenterOfMassTrajectoryMessage messageToTrim)
         {
            CenterOfMassTrajectoryMessage trimmed = new CenterOfMassTrajectoryMessage();
            trimmed.euclideanTrajectory = messageTrimmer.trim(messageToTrim.euclideanTrajectory);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<ChestHybridJointspaceTaskspaceTrajectoryMessage> createChestHybridJointspaceTaskspaceTrajectoryMessageTrimmer()
   {
      MessageTrimmer<JointspaceTrajectoryMessage> messageTrimmerJointspace = createJointspaceTrajectoryMessageTrimmer();
      MessageTrimmer<SO3TrajectoryMessage> messageTrimmerTaskspace = createSO3TrajectoryMessageTrimmer();

      return new MessageTrimmer<ChestHybridJointspaceTaskspaceTrajectoryMessage>()
      {
         @Override
         public ChestHybridJointspaceTaskspaceTrajectoryMessage trim(ChestHybridJointspaceTaskspaceTrajectoryMessage messageToTrim)
         {
            ChestHybridJointspaceTaskspaceTrajectoryMessage trimmed = new ChestHybridJointspaceTaskspaceTrajectoryMessage();
            trimmed.jointspaceTrajectoryMessage = messageTrimmerJointspace.trim(messageToTrim.jointspaceTrajectoryMessage);
            trimmed.taskspaceTrajectoryMessage = messageTrimmerTaskspace.trim(messageToTrim.taskspaceTrajectoryMessage);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<ChestTrajectoryMessage> createChestTrajectoryMessageTrimmer()
   {
      MessageTrimmer<SO3TrajectoryMessage> messageTrimmer = createSO3TrajectoryMessageTrimmer();

      return new MessageTrimmer<ChestTrajectoryMessage>()
      {
         @Override
         public ChestTrajectoryMessage trim(ChestTrajectoryMessage messageToTrim)
         {
            ChestTrajectoryMessage trimmed = new ChestTrajectoryMessage();
            trimmed.so3Trajectory = messageTrimmer.trim(messageToTrim.so3Trajectory);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<EuclideanTrajectoryMessage> createEuclideanTrajectoryMessageTrimmer()
   {
      return new MessageTrimmer<EuclideanTrajectoryMessage>()
      {
         @Override
         public EuclideanTrajectoryMessage trim(EuclideanTrajectoryMessage messageToTrim)
         {
            EuclideanTrajectoryMessage trimmed = new EuclideanTrajectoryMessage();
            trimmed.taskspaceTrajectoryPoints = new TempPreallocatedList<>(EuclideanTrajectoryPointMessage.class, EuclideanTrajectoryPointMessage::new,
                                                                           messageToTrim.taskspaceTrajectoryPoints.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<FootstepDataListMessage> createFootstepDataListMessageTrimmer()
   {
      return new MessageTrimmer<FootstepDataListMessage>()
      {
         @Override
         public FootstepDataListMessage trim(FootstepDataListMessage messageToTrim)
         {
            FootstepDataListMessage trimmed = new FootstepDataListMessage();
            trimmed.footstepDataList = trimList(messageToTrim.footstepDataList, createFootstepDataMessageTrimmer(), FootstepDataMessage.class);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<FootstepDataMessage> createFootstepDataMessageTrimmer()
   {
      return new MessageTrimmer<FootstepDataMessage>()
      {
         @Override
         public FootstepDataMessage trim(FootstepDataMessage messageToTrim)
         {
            FootstepDataMessage trimmed = new FootstepDataMessage();
            trimmed.predictedContactPoints = new TempPreallocatedList<>(Point2D.class, Point2D::new, messageToTrim.predictedContactPoints.size());
            trimmed.positionWaypoints = new TempPreallocatedList<>(Point3D.class, Point3D::new, messageToTrim.positionWaypoints.size());
            trimmed.swingTrajectory = new TempPreallocatedList<>(SE3TrajectoryPointMessage.class, SE3TrajectoryPointMessage::new,
                                                                 messageToTrim.swingTrajectory.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<FootstepPathPlanPacket> createFootstepPathPlanPacketTrimmer()
   {
      return new MessageTrimmer<FootstepPathPlanPacket>()
      {
         @Override
         public FootstepPathPlanPacket trim(FootstepPathPlanPacket messageToTrim)
         {
            FootstepPathPlanPacket trimmed = new FootstepPathPlanPacket();
            trimmed.originalGoals = trimList(messageToTrim.originalGoals, createFootstepDataMessageTrimmer(), FootstepDataMessage.class);
            trimmed.pathPlan = trimList(messageToTrim.pathPlan, createFootstepDataMessageTrimmer(), FootstepDataMessage.class);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<FootstepPlanningRequestPacket> createFootstepPlanningRequestPacketTrimmer()
   {
      return new MessageTrimmer<FootstepPlanningRequestPacket>()
      {
         MessageTrimmer<PlanarRegionsListMessage> messageTrimmer = createPlanarRegionsListMessageTrimmer();

         @Override
         public FootstepPlanningRequestPacket trim(FootstepPlanningRequestPacket messageToTrim)
         {
            FootstepPlanningRequestPacket trimmed = new FootstepPlanningRequestPacket();
            trimmed.planarRegionsListMessage = messageTrimmer.trim(messageToTrim.planarRegionsListMessage);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<FootstepPlanningToolboxOutputStatus> createFootstepPlanningToolboxOutputStatusTrimmer()
   {
      return new MessageTrimmer<FootstepPlanningToolboxOutputStatus>()
      {
         MessageTrimmer<PlanarRegionsListMessage> planarRegionsTrimmer = createPlanarRegionsListMessageTrimmer();
         MessageTrimmer<FootstepDataListMessage> footstepListTrimmer = createFootstepDataListMessageTrimmer();

         @Override
         public FootstepPlanningToolboxOutputStatus trim(FootstepPlanningToolboxOutputStatus messageToTrim)
         {
            FootstepPlanningToolboxOutputStatus trimmed = new FootstepPlanningToolboxOutputStatus();
            trimmed.planarRegionsListMessage = planarRegionsTrimmer.trim(messageToTrim.planarRegionsListMessage);
            trimmed.footstepDataList = footstepListTrimmer.trim(messageToTrim.footstepDataList);
            trimmed.bodyPath = new TempPreallocatedList<>(Point2D.class, Point2D::new, messageToTrim.bodyPath.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<FootstepPlanRequestPacket> createFootstepPlanRequestPacketTrimmer()
   {
      return new MessageTrimmer<FootstepPlanRequestPacket>()
      {
         MessageTrimmer<FootstepDataMessage> messageTrimmer = createFootstepDataMessageTrimmer();

         @Override
         public FootstepPlanRequestPacket trim(FootstepPlanRequestPacket messageToTrim)
         {
            FootstepPlanRequestPacket trimmed = new FootstepPlanRequestPacket();
            trimmed.goals = trimList(messageToTrim.goals, messageTrimmer, FootstepDataMessage.class);
            trimmed.startFootstep = messageTrimmer.trim(messageToTrim.startFootstep);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<FootTrajectoryMessage> createFootTrajectoryMessageTrimmer()
   {
      MessageTrimmer<SE3TrajectoryMessage> se3TrajectoryMessageTrimmer = createSE3TrajectoryMessageTrimmer();

      return new MessageTrimmer<FootTrajectoryMessage>()
      {
         @Override
         public FootTrajectoryMessage trim(FootTrajectoryMessage messageToTrim)
         {
            FootTrajectoryMessage trimmed = new FootTrajectoryMessage();
            trimmed.se3Trajectory = se3TrajectoryMessageTrimmer.trim(messageToTrim.se3Trajectory);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<HandHybridJointspaceTaskspaceTrajectoryMessage> createHandHybridJointspaceTaskspaceTrajectoryMessageTrimmer()
   {
      MessageTrimmer<JointspaceTrajectoryMessage> messageTrimmerJointspace = createJointspaceTrajectoryMessageTrimmer();
      MessageTrimmer<SE3TrajectoryMessage> messageTrimmerTaskspace = createSE3TrajectoryMessageTrimmer();

      return new MessageTrimmer<HandHybridJointspaceTaskspaceTrajectoryMessage>()
      {
         @Override
         public HandHybridJointspaceTaskspaceTrajectoryMessage trim(HandHybridJointspaceTaskspaceTrajectoryMessage messageToTrim)
         {
            HandHybridJointspaceTaskspaceTrajectoryMessage trimmed = new HandHybridJointspaceTaskspaceTrajectoryMessage();
            trimmed.jointspaceTrajectoryMessage = messageTrimmerJointspace.trim(messageToTrim.jointspaceTrajectoryMessage);
            trimmed.taskspaceTrajectoryMessage = messageTrimmerTaskspace.trim(messageToTrim.taskspaceTrajectoryMessage);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<HandLoadBearingMessage> createHandLoadBearingMessageTrimmer()
   {
      MessageTrimmer<JointspaceTrajectoryMessage> messageTrimmer = createJointspaceTrajectoryMessageTrimmer();

      return new MessageTrimmer<HandLoadBearingMessage>()
      {
         @Override
         public HandLoadBearingMessage trim(HandLoadBearingMessage messageToTrim)
         {
            HandLoadBearingMessage trimmed = new HandLoadBearingMessage();
            trimmed.jointspaceTrajectory = messageTrimmer.trim(messageToTrim.jointspaceTrajectory);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<HandTrajectoryMessage> createHandTrajectoryMessageTrimmer()
   {
      MessageTrimmer<SE3TrajectoryMessage> messageTrimmer = createSE3TrajectoryMessageTrimmer();

      return new MessageTrimmer<HandTrajectoryMessage>()
      {
         @Override
         public HandTrajectoryMessage trim(HandTrajectoryMessage messageToTrim)
         {
            HandTrajectoryMessage trimmed = new HandTrajectoryMessage();
            trimmed.se3Trajectory = messageTrimmer.trim(messageToTrim.se3Trajectory);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<HeadHybridJointspaceTaskspaceTrajectoryMessage> createHeadHybridJointspaceTaskspaceTrajectoryMessageTrimmer()
   {
      MessageTrimmer<JointspaceTrajectoryMessage> messageTrimmerJointspace = createJointspaceTrajectoryMessageTrimmer();
      MessageTrimmer<SO3TrajectoryMessage> messageTrimmerTaskspace = createSO3TrajectoryMessageTrimmer();

      return new MessageTrimmer<HeadHybridJointspaceTaskspaceTrajectoryMessage>()
      {
         @Override
         public HeadHybridJointspaceTaskspaceTrajectoryMessage trim(HeadHybridJointspaceTaskspaceTrajectoryMessage messageToTrim)
         {
            HeadHybridJointspaceTaskspaceTrajectoryMessage trimmed = new HeadHybridJointspaceTaskspaceTrajectoryMessage();
            trimmed.jointspaceTrajectoryMessage = messageTrimmerJointspace.trim(messageToTrim.jointspaceTrajectoryMessage);
            trimmed.taskspaceTrajectoryMessage = messageTrimmerTaskspace.trim(messageToTrim.taskspaceTrajectoryMessage);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<HeadTrajectoryMessage> createHeadTrajectoryMessageTrimmer()
   {
      MessageTrimmer<SO3TrajectoryMessage> messageTrimmer = createSO3TrajectoryMessageTrimmer();

      return new MessageTrimmer<HeadTrajectoryMessage>()
      {
         @Override
         public HeadTrajectoryMessage trim(HeadTrajectoryMessage messageToTrim)
         {
            HeadTrajectoryMessage trimmed = new HeadTrajectoryMessage();
            trimmed.so3Trajectory = messageTrimmer.trim(messageToTrim.so3Trajectory);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<HeightQuadTreeMessage> createHeightQuadTreeMessageTrimmer()
   {
      return new MessageTrimmer<HeightQuadTreeMessage>()
      {
         @Override
         public HeightQuadTreeMessage trim(HeightQuadTreeMessage messageToTrim)
         {
            HeightQuadTreeMessage trimmed = new HeightQuadTreeMessage();
            trimmed.leaves = new TempPreallocatedList<>(HeightQuadTreeLeafMessage.class, HeightQuadTreeLeafMessage::new, messageToTrim.leaves.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<JointspaceTrajectoryMessage> createJointspaceTrajectoryMessageTrimmer()
   {
      return new MessageTrimmer<JointspaceTrajectoryMessage>()
      {
         @Override
         public JointspaceTrajectoryMessage trim(JointspaceTrajectoryMessage messageToTrim)
         {
            JointspaceTrajectoryMessage trimmed = new JointspaceTrajectoryMessage();
            trimmed.jointTrajectoryMessages = trimList(messageToTrim.jointTrajectoryMessages, createOneDoFJointTrajectoryMessageTrimmer(),
                                                       OneDoFJointTrajectoryMessage.class);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<MomentumTrajectoryMessage> createMomentumTrajectoryMessageTrimmer()
   {
      MessageTrimmer<EuclideanTrajectoryMessage> messageTrimmer = createEuclideanTrajectoryMessageTrimmer();

      return new MessageTrimmer<MomentumTrajectoryMessage>()
      {
         @Override
         public MomentumTrajectoryMessage trim(MomentumTrajectoryMessage messageToTrim)
         {
            MomentumTrajectoryMessage trimmed = new MomentumTrajectoryMessage();
            trimmed.angularMomentumTrajectory = messageTrimmer.trim(messageToTrim.angularMomentumTrajectory);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<NeckTrajectoryMessage> createNeckTrajectoryMessageTrimmer()
   {
      MessageTrimmer<JointspaceTrajectoryMessage> messageTrimmer = createJointspaceTrajectoryMessageTrimmer();

      return new MessageTrimmer<NeckTrajectoryMessage>()
      {
         @Override
         public NeckTrajectoryMessage trim(NeckTrajectoryMessage messageToTrim)
         {
            NeckTrajectoryMessage trimmed = new NeckTrajectoryMessage();
            trimmed.jointspaceTrajectory = messageTrimmer.trim(messageToTrim.jointspaceTrajectory);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<ObjectDetectorResultPacket> createObjectDetectorResultPacketTrimmer()
   {
      MessageTrimmer<BoundingBoxesPacket> messageTrimmer = createBoundingBoxesPacketTrimmer();

      return new MessageTrimmer<ObjectDetectorResultPacket>()
      {
         @Override
         public ObjectDetectorResultPacket trim(ObjectDetectorResultPacket messageToTrim)
         {
            ObjectDetectorResultPacket trimmed = new ObjectDetectorResultPacket();
            trimmed.boundingBoxes = messageTrimmer.trim(messageToTrim.boundingBoxes);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<OneDoFJointTrajectoryMessage> createOneDoFJointTrajectoryMessageTrimmer()
   {
      return new MessageTrimmer<OneDoFJointTrajectoryMessage>()
      {
         @Override
         public OneDoFJointTrajectoryMessage trim(OneDoFJointTrajectoryMessage messageToTrim)
         {
            OneDoFJointTrajectoryMessage trimmed = new OneDoFJointTrajectoryMessage();
            trimmed.trajectoryPoints = new TempPreallocatedList<>(TrajectoryPoint1DMessage.class, TrajectoryPoint1DMessage::new,
                                                                  messageToTrim.trajectoryPoints.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<PelvisHeightTrajectoryMessage> createPelvisHeightTrajectoryMessageTrimmer()
   {
      MessageTrimmer<EuclideanTrajectoryMessage> messageTrimmer = createEuclideanTrajectoryMessageTrimmer();

      return new MessageTrimmer<PelvisHeightTrajectoryMessage>()
      {
         @Override
         public PelvisHeightTrajectoryMessage trim(PelvisHeightTrajectoryMessage messageToTrim)
         {
            PelvisHeightTrajectoryMessage trimmed = new PelvisHeightTrajectoryMessage();
            trimmed.euclideanTrajectory = messageTrimmer.trim(messageToTrim.euclideanTrajectory);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<PelvisOrientationTrajectoryMessage> createPelvisOrientationTrajectoryMessageTrimmer()
   {
      MessageTrimmer<SO3TrajectoryMessage> messageTrimmer = createSO3TrajectoryMessageTrimmer();

      return new MessageTrimmer<PelvisOrientationTrajectoryMessage>()
      {
         @Override
         public PelvisOrientationTrajectoryMessage trim(PelvisOrientationTrajectoryMessage messageToTrim)
         {
            PelvisOrientationTrajectoryMessage trimmed = new PelvisOrientationTrajectoryMessage();
            trimmed.so3Trajectory = messageTrimmer.trim(messageToTrim.so3Trajectory);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<PelvisTrajectoryMessage> createPelvisTrajectoryMessageTrimmer()
   {
      MessageTrimmer<SE3TrajectoryMessage> messageTrimmer = createSE3TrajectoryMessageTrimmer();

      return new MessageTrimmer<PelvisTrajectoryMessage>()
      {
         @Override
         public PelvisTrajectoryMessage trim(PelvisTrajectoryMessage messageToTrim)
         {
            PelvisTrajectoryMessage trimmed = new PelvisTrajectoryMessage();
            trimmed.se3Trajectory = messageTrimmer.trim(messageToTrim.se3Trajectory);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<PlanarRegionMessage> createPlanarRegionMessageTrimmer()
   {
      MessageTrimmer<Polygon2DMessage> messageTrimmer = createPolygon2DMessageTrimmer();

      return new MessageTrimmer<PlanarRegionMessage>()
      {
         @Override
         public PlanarRegionMessage trim(PlanarRegionMessage messageToTrim)
         {
            PlanarRegionMessage trimmed = new PlanarRegionMessage();
            trimmed.concaveHull = messageTrimmer.trim(messageToTrim.concaveHull);
            trimmed.convexPolygons = trimList(messageToTrim.convexPolygons, createPolygon2DMessageTrimmer(), Polygon2DMessage.class);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<PlanarRegionsListMessage> createPlanarRegionsListMessageTrimmer()
   {
      return new MessageTrimmer<PlanarRegionsListMessage>()
      {
         @Override
         public PlanarRegionsListMessage trim(PlanarRegionsListMessage messageToTrim)
         {
            PlanarRegionsListMessage trimmed = new PlanarRegionsListMessage();
            trimmed.planarRegions = trimList(messageToTrim.planarRegions, createPlanarRegionMessageTrimmer(), PlanarRegionMessage.class);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<Polygon2DMessage> createPolygon2DMessageTrimmer()
   {
      return new MessageTrimmer<Polygon2DMessage>()
      {
         @Override
         public Polygon2DMessage trim(Polygon2DMessage messageToTrim)
         {
            Polygon2DMessage trimmed = new Polygon2DMessage();
            trimmed.vertices = new TempPreallocatedList<>(Point2D32.class, Point2D32::new, messageToTrim.vertices.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<RobotConfigurationData> createRobotConfigurationDataTrimmer()
   {
      return new MessageTrimmer<RobotConfigurationData>()
      {
         @Override
         public RobotConfigurationData trim(RobotConfigurationData messageToTrim)
         {
            RobotConfigurationData trimmed = new RobotConfigurationData();
            trimmed.imuSensorData = new TempPreallocatedList<>(IMUPacket.class, IMUPacket::new, messageToTrim.imuSensorData.size());
            trimmed.momentAndForceDataAllForceSensors = new TempPreallocatedList<>(SpatialVectorMessage.class, SpatialVectorMessage::new,
                                                                                   messageToTrim.momentAndForceDataAllForceSensors.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<SE3TrajectoryMessage> createSE3TrajectoryMessageTrimmer()
   {
      return new MessageTrimmer<SE3TrajectoryMessage>()
      {
         @Override
         public SE3TrajectoryMessage trim(SE3TrajectoryMessage messageToTrim)
         {
            SE3TrajectoryMessage trimmed = new SE3TrajectoryMessage();
            trimmed.taskspaceTrajectoryPoints = new TempPreallocatedList<>(SE3TrajectoryPointMessage.class, SE3TrajectoryPointMessage::new,
                                                                           messageToTrim.taskspaceTrajectoryPoints.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<SnapFootstepPacket> createSnapFootstepPacketTrimmer()
   {

      return new MessageTrimmer<SnapFootstepPacket>()
      {
         MessageTrimmer<FootstepDataMessage> messageTrimmer = createFootstepDataMessageTrimmer();

         @Override
         public SnapFootstepPacket trim(SnapFootstepPacket messageToTrim)
         {
            SnapFootstepPacket trimmed = new SnapFootstepPacket();
            trimmed.footstepData = trimList(messageToTrim.footstepData, messageTrimmer, FootstepDataMessage.class);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<SO3TrajectoryMessage> createSO3TrajectoryMessageTrimmer()
   {
      return new MessageTrimmer<SO3TrajectoryMessage>()
      {
         @Override
         public SO3TrajectoryMessage trim(SO3TrajectoryMessage messageToTrim)
         {
            SO3TrajectoryMessage trimmed = new SO3TrajectoryMessage();
            trimmed.taskspaceTrajectoryPoints = new TempPreallocatedList<>(SO3TrajectoryPointMessage.class, SO3TrajectoryPointMessage::new,
                                                                           messageToTrim.taskspaceTrajectoryPoints.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<SpineTrajectoryMessage> createSpineTrajectoryMessageTrimmer()
   {
      MessageTrimmer<JointspaceTrajectoryMessage> messageTrimmer = createJointspaceTrajectoryMessageTrimmer();

      return new MessageTrimmer<SpineTrajectoryMessage>()
      {
         @Override
         public SpineTrajectoryMessage trim(SpineTrajectoryMessage messageToTrim)
         {
            SpineTrajectoryMessage trimmed = new SpineTrajectoryMessage();
            trimmed.jointspaceTrajectory = messageTrimmer.trim(messageToTrim.jointspaceTrajectory);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<WaypointBasedTrajectoryMessage> createWaypointBasedTrajectoryMessageTrimmer()
   {

      return new MessageTrimmer<WaypointBasedTrajectoryMessage>()
      {
         @Override
         public WaypointBasedTrajectoryMessage trim(WaypointBasedTrajectoryMessage messageToTrim)
         {
            WaypointBasedTrajectoryMessage trimmed = new WaypointBasedTrajectoryMessage();
            trimmed.waypoints = new TempPreallocatedList<>(Pose3D.class, Pose3D::new, messageToTrim.waypoints.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<WholeBodyTrajectoryMessage> createWholeBodyTrajectoryMessageTrimmer()
   {
      MessageTrimmer<ArmTrajectoryMessage> armMessageTrimmer = createArmTrajectoryMessageTrimmer();
      MessageTrimmer<HandTrajectoryMessage> handMessageTrimmer = createHandTrajectoryMessageTrimmer();
      MessageTrimmer<FootTrajectoryMessage> footMessageTrimmer = createFootTrajectoryMessageTrimmer();
      MessageTrimmer<ChestTrajectoryMessage> chestMessageTrimmer = createChestTrajectoryMessageTrimmer();
      MessageTrimmer<HeadTrajectoryMessage> headMessageTrimmer = createHeadTrajectoryMessageTrimmer();
      MessageTrimmer<PelvisTrajectoryMessage> pelvisMessageTrimmer = createPelvisTrajectoryMessageTrimmer();

      return new MessageTrimmer<WholeBodyTrajectoryMessage>()
      {
         @Override
         public WholeBodyTrajectoryMessage trim(WholeBodyTrajectoryMessage messageToTrim)
         {
            WholeBodyTrajectoryMessage trimmed = new WholeBodyTrajectoryMessage();
            trimmed.leftArmTrajectoryMessage = armMessageTrimmer.trim(messageToTrim.leftArmTrajectoryMessage);
            trimmed.rightArmTrajectoryMessage = armMessageTrimmer.trim(messageToTrim.rightArmTrajectoryMessage);
            trimmed.leftHandTrajectoryMessage = handMessageTrimmer.trim(messageToTrim.leftHandTrajectoryMessage);
            trimmed.rightHandTrajectoryMessage = handMessageTrimmer.trim(messageToTrim.rightHandTrajectoryMessage);
            trimmed.leftFootTrajectoryMessage = footMessageTrimmer.trim(messageToTrim.leftFootTrajectoryMessage);
            trimmed.rightFootTrajectoryMessage = footMessageTrimmer.trim(messageToTrim.rightFootTrajectoryMessage);
            trimmed.chestTrajectoryMessage = chestMessageTrimmer.trim(messageToTrim.chestTrajectoryMessage);
            trimmed.headTrajectoryMessage = headMessageTrimmer.trim(messageToTrim.headTrajectoryMessage);
            trimmed.pelvisTrajectoryMessage = pelvisMessageTrimmer.trim(messageToTrim.pelvisTrajectoryMessage);
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<WholeBodyTrajectoryToolboxMessage> createWholeBodyTrajectoryToolboxMessageTrimmer()
   {

      return new MessageTrimmer<WholeBodyTrajectoryToolboxMessage>()
      {
         @Override
         public WholeBodyTrajectoryToolboxMessage trim(WholeBodyTrajectoryToolboxMessage messageToTrim)
         {
            WholeBodyTrajectoryToolboxMessage trimmed = new WholeBodyTrajectoryToolboxMessage();
            trimmed.endEffectorTrajectories = trimList(messageToTrim.endEffectorTrajectories, createWaypointBasedTrajectoryMessageTrimmer(),
                                                       WaypointBasedTrajectoryMessage.class);
            trimmed.explorationConfigurations = new TempPreallocatedList<>(RigidBodyExplorationConfigurationMessage.class,
                                                                           RigidBodyExplorationConfigurationMessage::new,
                                                                           messageToTrim.explorationConfigurations.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static MessageTrimmer<WholeBodyTrajectoryToolboxOutputStatus> createWholeBodyTrajectoryToolboxOutputStatusTrimmer()
   {

      return new MessageTrimmer<WholeBodyTrajectoryToolboxOutputStatus>()
      {
         @Override
         public WholeBodyTrajectoryToolboxOutputStatus trim(WholeBodyTrajectoryToolboxOutputStatus messageToTrim)
         {
            WholeBodyTrajectoryToolboxOutputStatus trimmed = new WholeBodyTrajectoryToolboxOutputStatus();
            trimmed.robotConfigurations = new TempPreallocatedList<>(KinematicsToolboxOutputStatus.class, KinematicsToolboxOutputStatus::new,
                                                                     messageToTrim.robotConfigurations.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   private static <T> TempPreallocatedList<T> trimList(TempPreallocatedList<T> listToTrim, MessageTrimmer<T> elementTrimmer, Class<T> elementClass)
   {
      ListAllocator<T> allocator = new ListAllocator<T>()
      {
         int index = 0;

         @Override
         public T createInstance()
         { // Cheating here, knowing that the allocator will be used for each element in order starting at index 0.
            return elementTrimmer.trim(listToTrim.get(index++));
         }
      };

      return new TempPreallocatedList<>(elementClass, allocator, listToTrim.size());
   }

   private MessageTrimmingTools()
   {
   }
}
