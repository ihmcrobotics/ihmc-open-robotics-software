package us.ihmc.humanoidRobotics.communication.util;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.ParameterizedType;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import us.ihmc.communication.net.NetClassList.PacketTrimmer;
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
import us.ihmc.idl.PreallocatedList;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;

/**
 * This class provides the tools to trim down the size of messages before sending them on the
 * network. This won't be useful once using DDS, but it is more likely needed for Kryo to prevent it
 * from sending unused data stored in PreallocatedList.
 */
public final class MessageTrimmingTools
{
   public static final Map<Class<?>, PacketTrimmer<?>> classToPacketTrimmerMap = getClassToPacketTrimmerMap();

   private static Map<Class<?>, PacketTrimmer<?>> getClassToPacketTrimmerMap()
   {
      Map<Class<?>, PacketTrimmer<?>> map = new HashMap<>();
      Method[] declaredMethods = MessageTrimmingTools.class.getDeclaredMethods();
      for (Method method : declaredMethods)
      {
         if (method.getName().startsWith("create") && method.getName().endsWith("Trimmer") && method.getReturnType() == PacketTrimmer.class)
         {
            Class<?> type = (Class<?>) ((ParameterizedType) method.getGenericReturnType()).getActualTypeArguments()[0];
            try
            {
               map.put(type, (PacketTrimmer<?>) method.invoke(null));
            }
            catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
            {
               e.printStackTrace();
            }
         }
      }
      return Collections.unmodifiableMap(map);
   }

   public static PacketTrimmer<AdjustFootstepMessage> createAdjustFootstepMessageTrimmer()
   {
      return new PacketTrimmer<AdjustFootstepMessage>()
      {
         @Override
         public AdjustFootstepMessage trim(AdjustFootstepMessage messageToTrim)
         {
            AdjustFootstepMessage trimmed = new AdjustFootstepMessage();
            trimmed.predictedContactPoints = new PreallocatedList<>(Point2D.class, Point2D::new, messageToTrim.predictedContactPoints.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static PacketTrimmer<ArmTrajectoryMessage> createArmTrajectoryMessageTrimmer()
   {
      PacketTrimmer<JointspaceTrajectoryMessage> messageTrimmer = createJointspaceTrajectoryMessageTrimmer();

      return new PacketTrimmer<ArmTrajectoryMessage>()
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

   public static PacketTrimmer<BoundingBoxesPacket> createBoundingBoxesPacketTrimmer()
   {
      return new PacketTrimmer<BoundingBoxesPacket>()
      {
         @Override
         public BoundingBoxesPacket trim(BoundingBoxesPacket messageToTrim)
         {
            BoundingBoxesPacket trimmed = new BoundingBoxesPacket();
            trimmed.labels = new PreallocatedList<>(StringBuilder.class, StringBuilder::new, messageToTrim.labels.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static PacketTrimmer<CapturabilityBasedStatus> createCapturabilityBasedStatusTrimmer()
   {
      return new PacketTrimmer<CapturabilityBasedStatus>()
      {
         @Override
         public CapturabilityBasedStatus trim(CapturabilityBasedStatus messageToTrim)
         {
            CapturabilityBasedStatus trimmed = new CapturabilityBasedStatus();
            trimmed.leftFootSupportPolygon = new PreallocatedList<>(Point2D.class, Point2D::new, messageToTrim.leftFootSupportPolygon.size());
            trimmed.rightFootSupportPolygon = new PreallocatedList<>(Point2D.class, Point2D::new, messageToTrim.rightFootSupportPolygon.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static PacketTrimmer<CenterOfMassTrajectoryMessage> createCenterOfMassTrajectoryMessageTrimmer()
   {
      PacketTrimmer<EuclideanTrajectoryMessage> messageTrimmer = createEuclideanTrajectoryMessageTrimmer();

      return new PacketTrimmer<CenterOfMassTrajectoryMessage>()
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

   public static PacketTrimmer<ChestHybridJointspaceTaskspaceTrajectoryMessage> createChestHybridJointspaceTaskspaceTrajectoryMessageTrimmer()
   {
      PacketTrimmer<JointspaceTrajectoryMessage> messageTrimmerJointspace = createJointspaceTrajectoryMessageTrimmer();
      PacketTrimmer<SO3TrajectoryMessage> messageTrimmerTaskspace = createSO3TrajectoryMessageTrimmer();

      return new PacketTrimmer<ChestHybridJointspaceTaskspaceTrajectoryMessage>()
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

   public static PacketTrimmer<ChestTrajectoryMessage> createChestTrajectoryMessageTrimmer()
   {
      PacketTrimmer<SO3TrajectoryMessage> messageTrimmer = createSO3TrajectoryMessageTrimmer();

      return new PacketTrimmer<ChestTrajectoryMessage>()
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

   public static PacketTrimmer<EuclideanTrajectoryMessage> createEuclideanTrajectoryMessageTrimmer()
   {
      return new PacketTrimmer<EuclideanTrajectoryMessage>()
      {
         @Override
         public EuclideanTrajectoryMessage trim(EuclideanTrajectoryMessage messageToTrim)
         {
            EuclideanTrajectoryMessage trimmed = new EuclideanTrajectoryMessage();
            trimmed.taskspaceTrajectoryPoints = new PreallocatedList<>(EuclideanTrajectoryPointMessage.class, EuclideanTrajectoryPointMessage::new,
                                                                           messageToTrim.taskspaceTrajectoryPoints.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static PacketTrimmer<FootstepDataListMessage> createFootstepDataListMessageTrimmer()
   {
      return new PacketTrimmer<FootstepDataListMessage>()
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

   public static PacketTrimmer<FootstepDataMessage> createFootstepDataMessageTrimmer()
   {
      return new PacketTrimmer<FootstepDataMessage>()
      {
         @Override
         public FootstepDataMessage trim(FootstepDataMessage messageToTrim)
         {
            FootstepDataMessage trimmed = new FootstepDataMessage();
            trimmed.predictedContactPoints = new PreallocatedList<>(Point2D.class, Point2D::new, messageToTrim.predictedContactPoints.size());
            trimmed.positionWaypoints = new PreallocatedList<>(Point3D.class, Point3D::new, messageToTrim.positionWaypoints.size());
            trimmed.swingTrajectory = new PreallocatedList<>(SE3TrajectoryPointMessage.class, SE3TrajectoryPointMessage::new,
                                                                 messageToTrim.swingTrajectory.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static PacketTrimmer<FootstepPathPlanPacket> createFootstepPathPlanPacketTrimmer()
   {
      return new PacketTrimmer<FootstepPathPlanPacket>()
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

   public static PacketTrimmer<FootstepPlanningRequestPacket> createFootstepPlanningRequestPacketTrimmer()
   {
      return new PacketTrimmer<FootstepPlanningRequestPacket>()
      {
         PacketTrimmer<PlanarRegionsListMessage> messageTrimmer = createPlanarRegionsListMessageTrimmer();

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

   public static PacketTrimmer<FootstepPlanningToolboxOutputStatus> createFootstepPlanningToolboxOutputStatusTrimmer()
   {
      return new PacketTrimmer<FootstepPlanningToolboxOutputStatus>()
      {
         PacketTrimmer<PlanarRegionsListMessage> planarRegionsTrimmer = createPlanarRegionsListMessageTrimmer();
         PacketTrimmer<FootstepDataListMessage> footstepListTrimmer = createFootstepDataListMessageTrimmer();

         @Override
         public FootstepPlanningToolboxOutputStatus trim(FootstepPlanningToolboxOutputStatus messageToTrim)
         {
            FootstepPlanningToolboxOutputStatus trimmed = new FootstepPlanningToolboxOutputStatus();
            trimmed.planarRegionsListMessage = planarRegionsTrimmer.trim(messageToTrim.planarRegionsListMessage);
            trimmed.footstepDataList = footstepListTrimmer.trim(messageToTrim.footstepDataList);
            trimmed.bodyPath = new PreallocatedList<>(Point2D.class, Point2D::new, messageToTrim.bodyPath.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static PacketTrimmer<FootstepPlanRequestPacket> createFootstepPlanRequestPacketTrimmer()
   {
      return new PacketTrimmer<FootstepPlanRequestPacket>()
      {
         PacketTrimmer<FootstepDataMessage> messageTrimmer = createFootstepDataMessageTrimmer();

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

   public static PacketTrimmer<FootTrajectoryMessage> createFootTrajectoryMessageTrimmer()
   {
      PacketTrimmer<SE3TrajectoryMessage> se3TrajectoryMessageTrimmer = createSE3TrajectoryMessageTrimmer();

      return new PacketTrimmer<FootTrajectoryMessage>()
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

   public static PacketTrimmer<HandHybridJointspaceTaskspaceTrajectoryMessage> createHandHybridJointspaceTaskspaceTrajectoryMessageTrimmer()
   {
      PacketTrimmer<JointspaceTrajectoryMessage> messageTrimmerJointspace = createJointspaceTrajectoryMessageTrimmer();
      PacketTrimmer<SE3TrajectoryMessage> messageTrimmerTaskspace = createSE3TrajectoryMessageTrimmer();

      return new PacketTrimmer<HandHybridJointspaceTaskspaceTrajectoryMessage>()
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

   public static PacketTrimmer<HandLoadBearingMessage> createHandLoadBearingMessageTrimmer()
   {
      PacketTrimmer<JointspaceTrajectoryMessage> messageTrimmer = createJointspaceTrajectoryMessageTrimmer();

      return new PacketTrimmer<HandLoadBearingMessage>()
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

   public static PacketTrimmer<HandTrajectoryMessage> createHandTrajectoryMessageTrimmer()
   {
      PacketTrimmer<SE3TrajectoryMessage> messageTrimmer = createSE3TrajectoryMessageTrimmer();

      return new PacketTrimmer<HandTrajectoryMessage>()
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

   public static PacketTrimmer<HeadHybridJointspaceTaskspaceTrajectoryMessage> createHeadHybridJointspaceTaskspaceTrajectoryMessageTrimmer()
   {
      PacketTrimmer<JointspaceTrajectoryMessage> messageTrimmerJointspace = createJointspaceTrajectoryMessageTrimmer();
      PacketTrimmer<SO3TrajectoryMessage> messageTrimmerTaskspace = createSO3TrajectoryMessageTrimmer();

      return new PacketTrimmer<HeadHybridJointspaceTaskspaceTrajectoryMessage>()
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

   public static PacketTrimmer<HeadTrajectoryMessage> createHeadTrajectoryMessageTrimmer()
   {
      PacketTrimmer<SO3TrajectoryMessage> messageTrimmer = createSO3TrajectoryMessageTrimmer();

      return new PacketTrimmer<HeadTrajectoryMessage>()
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

   public static PacketTrimmer<HeightQuadTreeMessage> createHeightQuadTreeMessageTrimmer()
   {
      return new PacketTrimmer<HeightQuadTreeMessage>()
      {
         @Override
         public HeightQuadTreeMessage trim(HeightQuadTreeMessage messageToTrim)
         {
            HeightQuadTreeMessage trimmed = new HeightQuadTreeMessage();
            trimmed.leaves = new PreallocatedList<>(HeightQuadTreeLeafMessage.class, HeightQuadTreeLeafMessage::new, messageToTrim.leaves.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static PacketTrimmer<JointspaceTrajectoryMessage> createJointspaceTrajectoryMessageTrimmer()
   {
      return new PacketTrimmer<JointspaceTrajectoryMessage>()
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

   public static PacketTrimmer<MomentumTrajectoryMessage> createMomentumTrajectoryMessageTrimmer()
   {
      PacketTrimmer<EuclideanTrajectoryMessage> messageTrimmer = createEuclideanTrajectoryMessageTrimmer();

      return new PacketTrimmer<MomentumTrajectoryMessage>()
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

   public static PacketTrimmer<NeckTrajectoryMessage> createNeckTrajectoryMessageTrimmer()
   {
      PacketTrimmer<JointspaceTrajectoryMessage> messageTrimmer = createJointspaceTrajectoryMessageTrimmer();

      return new PacketTrimmer<NeckTrajectoryMessage>()
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

   public static PacketTrimmer<ObjectDetectorResultPacket> createObjectDetectorResultPacketTrimmer()
   {
      PacketTrimmer<BoundingBoxesPacket> messageTrimmer = createBoundingBoxesPacketTrimmer();

      return new PacketTrimmer<ObjectDetectorResultPacket>()
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

   public static PacketTrimmer<OneDoFJointTrajectoryMessage> createOneDoFJointTrajectoryMessageTrimmer()
   {
      return new PacketTrimmer<OneDoFJointTrajectoryMessage>()
      {
         @Override
         public OneDoFJointTrajectoryMessage trim(OneDoFJointTrajectoryMessage messageToTrim)
         {
            OneDoFJointTrajectoryMessage trimmed = new OneDoFJointTrajectoryMessage();
            trimmed.trajectoryPoints = new PreallocatedList<>(TrajectoryPoint1DMessage.class, TrajectoryPoint1DMessage::new,
                                                                  messageToTrim.trajectoryPoints.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static PacketTrimmer<PelvisHeightTrajectoryMessage> createPelvisHeightTrajectoryMessageTrimmer()
   {
      PacketTrimmer<EuclideanTrajectoryMessage> messageTrimmer = createEuclideanTrajectoryMessageTrimmer();

      return new PacketTrimmer<PelvisHeightTrajectoryMessage>()
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

   public static PacketTrimmer<PelvisOrientationTrajectoryMessage> createPelvisOrientationTrajectoryMessageTrimmer()
   {
      PacketTrimmer<SO3TrajectoryMessage> messageTrimmer = createSO3TrajectoryMessageTrimmer();

      return new PacketTrimmer<PelvisOrientationTrajectoryMessage>()
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

   public static PacketTrimmer<PelvisTrajectoryMessage> createPelvisTrajectoryMessageTrimmer()
   {
      PacketTrimmer<SE3TrajectoryMessage> messageTrimmer = createSE3TrajectoryMessageTrimmer();

      return new PacketTrimmer<PelvisTrajectoryMessage>()
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

   public static PacketTrimmer<PlanarRegionMessage> createPlanarRegionMessageTrimmer()
   {
      PacketTrimmer<Polygon2DMessage> messageTrimmer = createPolygon2DMessageTrimmer();

      return new PacketTrimmer<PlanarRegionMessage>()
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

   public static PacketTrimmer<PlanarRegionsListMessage> createPlanarRegionsListMessageTrimmer()
   {
      return new PacketTrimmer<PlanarRegionsListMessage>()
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

   public static PacketTrimmer<Polygon2DMessage> createPolygon2DMessageTrimmer()
   {
      return new PacketTrimmer<Polygon2DMessage>()
      {
         @Override
         public Polygon2DMessage trim(Polygon2DMessage messageToTrim)
         {
            Polygon2DMessage trimmed = new Polygon2DMessage();
            trimmed.vertices = new PreallocatedList<>(Point2D32.class, Point2D32::new, messageToTrim.vertices.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static PacketTrimmer<RobotConfigurationData> createRobotConfigurationDataTrimmer()
   {
      return new PacketTrimmer<RobotConfigurationData>()
      {
         @Override
         public RobotConfigurationData trim(RobotConfigurationData messageToTrim)
         {
            RobotConfigurationData trimmed = new RobotConfigurationData();
            trimmed.imuSensorData = new PreallocatedList<>(IMUPacket.class, IMUPacket::new, messageToTrim.imuSensorData.size());
            trimmed.momentAndForceDataAllForceSensors = new PreallocatedList<>(SpatialVectorMessage.class, SpatialVectorMessage::new,
                                                                                   messageToTrim.momentAndForceDataAllForceSensors.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static PacketTrimmer<SE3TrajectoryMessage> createSE3TrajectoryMessageTrimmer()
   {
      return new PacketTrimmer<SE3TrajectoryMessage>()
      {
         @Override
         public SE3TrajectoryMessage trim(SE3TrajectoryMessage messageToTrim)
         {
            SE3TrajectoryMessage trimmed = new SE3TrajectoryMessage();
            trimmed.taskspaceTrajectoryPoints = new PreallocatedList<>(SE3TrajectoryPointMessage.class, SE3TrajectoryPointMessage::new,
                                                                           messageToTrim.taskspaceTrajectoryPoints.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static PacketTrimmer<SnapFootstepPacket> createSnapFootstepPacketTrimmer()
   {

      return new PacketTrimmer<SnapFootstepPacket>()
      {
         PacketTrimmer<FootstepDataMessage> messageTrimmer = createFootstepDataMessageTrimmer();

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

   public static PacketTrimmer<SO3TrajectoryMessage> createSO3TrajectoryMessageTrimmer()
   {
      return new PacketTrimmer<SO3TrajectoryMessage>()
      {
         @Override
         public SO3TrajectoryMessage trim(SO3TrajectoryMessage messageToTrim)
         {
            SO3TrajectoryMessage trimmed = new SO3TrajectoryMessage();
            trimmed.taskspaceTrajectoryPoints = new PreallocatedList<>(SO3TrajectoryPointMessage.class, SO3TrajectoryPointMessage::new,
                                                                           messageToTrim.taskspaceTrajectoryPoints.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static PacketTrimmer<SpineTrajectoryMessage> createSpineTrajectoryMessageTrimmer()
   {
      PacketTrimmer<JointspaceTrajectoryMessage> messageTrimmer = createJointspaceTrajectoryMessageTrimmer();

      return new PacketTrimmer<SpineTrajectoryMessage>()
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

   public static PacketTrimmer<WaypointBasedTrajectoryMessage> createWaypointBasedTrajectoryMessageTrimmer()
   {

      return new PacketTrimmer<WaypointBasedTrajectoryMessage>()
      {
         @Override
         public WaypointBasedTrajectoryMessage trim(WaypointBasedTrajectoryMessage messageToTrim)
         {
            WaypointBasedTrajectoryMessage trimmed = new WaypointBasedTrajectoryMessage();
            trimmed.waypoints = new PreallocatedList<>(Pose3D.class, Pose3D::new, messageToTrim.waypoints.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static PacketTrimmer<WholeBodyTrajectoryMessage> createWholeBodyTrajectoryMessageTrimmer()
   {
      PacketTrimmer<ArmTrajectoryMessage> armMessageTrimmer = createArmTrajectoryMessageTrimmer();
      PacketTrimmer<HandTrajectoryMessage> handMessageTrimmer = createHandTrajectoryMessageTrimmer();
      PacketTrimmer<FootTrajectoryMessage> footMessageTrimmer = createFootTrajectoryMessageTrimmer();
      PacketTrimmer<ChestTrajectoryMessage> chestMessageTrimmer = createChestTrajectoryMessageTrimmer();
      PacketTrimmer<HeadTrajectoryMessage> headMessageTrimmer = createHeadTrajectoryMessageTrimmer();
      PacketTrimmer<PelvisTrajectoryMessage> pelvisMessageTrimmer = createPelvisTrajectoryMessageTrimmer();

      return new PacketTrimmer<WholeBodyTrajectoryMessage>()
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

   public static PacketTrimmer<WholeBodyTrajectoryToolboxMessage> createWholeBodyTrajectoryToolboxMessageTrimmer()
   {

      return new PacketTrimmer<WholeBodyTrajectoryToolboxMessage>()
      {
         @Override
         public WholeBodyTrajectoryToolboxMessage trim(WholeBodyTrajectoryToolboxMessage messageToTrim)
         {
            WholeBodyTrajectoryToolboxMessage trimmed = new WholeBodyTrajectoryToolboxMessage();
            trimmed.endEffectorTrajectories = trimList(messageToTrim.endEffectorTrajectories, createWaypointBasedTrajectoryMessageTrimmer(),
                                                       WaypointBasedTrajectoryMessage.class);
            trimmed.explorationConfigurations = new PreallocatedList<>(RigidBodyExplorationConfigurationMessage.class,
                                                                           RigidBodyExplorationConfigurationMessage::new,
                                                                           messageToTrim.explorationConfigurations.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   public static PacketTrimmer<WholeBodyTrajectoryToolboxOutputStatus> createWholeBodyTrajectoryToolboxOutputStatusTrimmer()
   {

      return new PacketTrimmer<WholeBodyTrajectoryToolboxOutputStatus>()
      {
         @Override
         public WholeBodyTrajectoryToolboxOutputStatus trim(WholeBodyTrajectoryToolboxOutputStatus messageToTrim)
         {
            WholeBodyTrajectoryToolboxOutputStatus trimmed = new WholeBodyTrajectoryToolboxOutputStatus();
            trimmed.robotConfigurations = new PreallocatedList<>(KinematicsToolboxOutputStatus.class, KinematicsToolboxOutputStatus::new,
                                                                     messageToTrim.robotConfigurations.size());
            trimmed.set(messageToTrim);
            return trimmed;
         }
      };
   }

   private static <T> PreallocatedList<T> trimList(PreallocatedList<T> listToTrim, PacketTrimmer<T> elementTrimmer, Class<T> elementClass)
   {
      Supplier<T> allocator = new Supplier<T>()
      {
         int index = 0;

         @Override
         public T get()
         { // Cheating here, knowing that the allocator will be used for each element in order starting at index 0.
            return elementTrimmer.trim(listToTrim.get(index++));
         }
      };

      return new PreallocatedList<>(elementClass, allocator, listToTrim.size());
   }

   private MessageTrimmingTools()
   {
   }
}
