package us.ihmc.robotModels;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.function.Function;
import java.util.stream.Collectors;

import us.ihmc.euclid.referenceFrame.FixedReferenceFrame;
import us.ihmc.euclid.referenceFrame.FrameNameRestrictionLevel;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.description.CrossFourBarJointDescription;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotDescription.CameraSensorDescription;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.ForceSensorDescription;
import us.ihmc.robotics.robotDescription.IMUSensorDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LidarSensorDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LoopClosureConstraintDescription;
import us.ihmc.robotics.robotDescription.LoopClosurePinConstraintDescription;
import us.ihmc.robotics.robotDescription.OneDoFJointDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.SliderJointDescription;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.scs2.definition.robot.CameraSensorDefinition;
import us.ihmc.scs2.definition.robot.IMUSensorDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.LidarSensorDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SensorDefinition;
import us.ihmc.scs2.definition.robot.WrenchSensorDefinition;

public class FullRobotModelWrapper implements FullRobotModel
{
   private final RigidBodyBasics elevator;
   private final FloatingJointBasics rootJoint;
   private final RigidBodyBasics rootBody;
   private final OneDoFJointBasics[] oneDoFJoints;
   private final Map<String, OneDoFJointBasics> jointNameToOneDoFJointMap;
   private final boolean enforceUniqueReferenceFrames;

   private final double totalMass;

   // Fields initialized with a JointNameMap
   private JointNameMap<?> jointNameMap;
   private RigidBodyBasics headBody;
   private Map<Enum<?>, RigidBodyBasics> endEffectors;
   private EnumMap<NeckJointName, OneDoFJointBasics> neckJoints;
   private EnumMap<SpineJointName, OneDoFJointBasics> spineJoints;

   // Fields initialized with RobotDefinition
   private IMUDefinition[] imuDefinitions;
   private ForceSensorDefinition[] forceSensorDefinitions;
   private Map<String, JointBasics> cameraJoints;
   private Map<String, ReferenceFrame> cameraFrames;
   private Map<String, JointBasics> lidarJoints;
   private Map<String, RigidBodyTransform> lidarBaseToSensorTransform;

   public FullRobotModelWrapper(FullRobotModelWrapper other)
   {
      this(other.elevator, other.enforceUniqueReferenceFrames);
      if (other.jointNameMap != null)
         setupJointNameMap(other.jointNameMap);

      if (other.imuDefinitions != null)
      {
         imuDefinitions = new IMUDefinition[other.imuDefinitions.length];

         for (int i = 0; i < other.imuDefinitions.length; i++)
         {
            IMUDefinition otherIMUDefinition = other.imuDefinitions[i];
            RigidBodyBasics body = MultiBodySystemTools.findRigidBody(elevator, otherIMUDefinition.getRigidBody().getName());
            imuDefinitions[i] = new IMUDefinition(otherIMUDefinition.getName(), body, otherIMUDefinition.getTransformFromIMUToJoint());
         }
      }

      if (other.forceSensorDefinitions != null)
      {
         forceSensorDefinitions = new ForceSensorDefinition[other.forceSensorDefinitions.length];

         for (int i = 0; i < other.forceSensorDefinitions.length; i++)
         {
            ForceSensorDefinition otherForceSensorDefinition = other.forceSensorDefinitions[i];
            RigidBodyBasics body = MultiBodySystemTools.findRigidBody(elevator, otherForceSensorDefinition.getRigidBody().getName());
            ReferenceFrame sensorFrame = ForceSensorDefinition.createSensorFrame(otherForceSensorDefinition.getSensorName(),
                                                                                 body,
                                                                                 otherForceSensorDefinition.getSensorFrame().getTransformToParent());
            forceSensorDefinitions[i] = new ForceSensorDefinition(otherForceSensorDefinition.getSensorName(), body, sensorFrame);
         }
      }

      if (other.cameraJoints != null && other.cameraFrames != null)
      {
         cameraJoints = new HashMap<>();
         cameraFrames = new HashMap<>();

         for (Entry<String, ReferenceFrame> entry : other.cameraFrames.entrySet())
         {
            JointBasics joint = MultiBodySystemTools.findJoint(elevator, other.cameraJoints.get(entry.getKey()).getName());
            cameraJoints.put(entry.getKey(), joint);
            cameraFrames.put(entry.getKey(), new FixedReferenceFrame(entry.getKey(), joint.getFrameAfterJoint(), entry.getValue().getTransformToParent()));
         }
      }

      if (other.lidarJoints != null && other.lidarBaseToSensorTransform != null)
      {
         lidarJoints = new HashMap<>();
         lidarBaseToSensorTransform = new HashMap<>();

         for (Entry<String, RigidBodyTransform> entry : other.lidarBaseToSensorTransform.entrySet())
         {
            JointBasics joint = MultiBodySystemTools.findJoint(elevator, other.lidarJoints.get(entry.getKey()).getName());
            lidarJoints.put(entry.getKey(), joint);
            lidarBaseToSensorTransform.put(entry.getKey(), new RigidBodyTransform(entry.getValue()));
         }
      }
   }

   public FullRobotModelWrapper(RobotDefinition robotDefinition, JointNameMap<?> jointNameMap, boolean enforceUniqueReferenceFrames)
   {
      this(robotDefinition.newInstance(ReferenceFrame.getWorldFrame()), enforceUniqueReferenceFrames);
      setupJointNameMap(jointNameMap);
      setupRobotDefinition(robotDefinition);
   }

   public FullRobotModelWrapper(RobotDescription robotDescription, JointNameMap<?> jointNameMap, boolean enforceUniqueReferenceFrames)
   {
      this(instantiateRobot(robotDescription, ReferenceFrame.getWorldFrame()), enforceUniqueReferenceFrames);
      setupJointNameMap(jointNameMap);
      setupRobotDescription(robotDescription);
   }

   public FullRobotModelWrapper(RigidBodyBasics elevator, boolean enforceUniqueReferenceFrames)
   {
      this.elevator = elevator;
      this.enforceUniqueReferenceFrames = enforceUniqueReferenceFrames;

      if (enforceUniqueReferenceFrames)
         elevator.getBodyFixedFrame().setNameRestrictionLevel(FrameNameRestrictionLevel.FRAME_NAME);

      if (elevator.getChildrenJoints().size() != 1)
         throw new IllegalArgumentException("Unexpected number of root joints: " + elevator.getChildrenJoints());
      if (!(elevator.getChildrenJoints().get(0) instanceof FloatingJointBasics))
         throw new IllegalArgumentException("Unexpected root joint type: " + elevator.getChildrenJoints().get(0));

      rootJoint = (FloatingJointBasics) elevator.getChildrenJoints().get(0);
      rootBody = rootJoint.getSuccessor();
      oneDoFJoints = collectOneDoFJoints(elevator, null).toArray(new OneDoFJointBasics[0]);
      jointNameToOneDoFJointMap = Arrays.stream(oneDoFJoints).collect(Collectors.toMap(JointReadOnly::getName, Function.identity()));

      totalMass = TotalMassCalculator.computeSubTreeMass(elevator);
   }

   /**
    * Similar to {@link MultiBodySystemTools#collectSubtreeJoints(RigidBodyBasics...)}, collects the
    * 1-DoF joints in the subtree that starts at {@code start}.
    * <p>
    * However, this method collects the joints in a different order where joints of a limbs tend to be
    * successive in the resulting list. Using this method for backward compatibility as changing the
    * ordering of the joints notably changing the computed hash-code of the robot.
    * </p>
    */
   private static List<OneDoFJointBasics> collectOneDoFJoints(RigidBodyBasics start, List<OneDoFJointBasics> jointsToPack)
   {
      if (start == null || !start.hasChildrenJoints())
         return jointsToPack;
      if (jointsToPack == null)
         jointsToPack = new ArrayList<>();

      for (JointBasics childJoint : start.getChildrenJoints())
      {
         // Quickfix to handle kinematic loops
         if (jointsToPack.contains(childJoint))
            break;
         if (childJoint instanceof OneDoFJointBasics)
            jointsToPack.add((OneDoFJointBasics) childJoint);
         collectOneDoFJoints(childJoint.getSuccessor(), jointsToPack);
      }
      return jointsToPack;
   }

   /**
    * This method configures the sensor features, such as frames and joints.
    *
    * @param robotDefinition
    */
   protected void setupRobotDefinition(RobotDefinition robotDefinition)
   {
      cameraJoints = new HashMap<>();
      cameraFrames = new HashMap<>();
      lidarJoints = new HashMap<>();
      lidarBaseToSensorTransform = new HashMap<>();
      List<IMUDefinition> imuDefinitionList = new ArrayList<>();
      List<ForceSensorDefinition> forceSensorDefinitionList = new ArrayList<>();

      for (JointDefinition jointDefinition : robotDefinition.getAllJoints())
      {
         JointBasics joint = MultiBodySystemTools.findJoint(elevator, jointDefinition.getName());

         for (SensorDefinition sensorDefinition : jointDefinition.getSensorDefinitions())
         {
            if (sensorDefinition instanceof IMUSensorDefinition)
            {
               imuDefinitionList.add(new IMUDefinition(sensorDefinition.getName(), joint.getSuccessor(), sensorDefinition.getTransformToJoint()));
            }
            else if (sensorDefinition instanceof WrenchSensorDefinition)
            {
               ReferenceFrame sensorFrame = ForceSensorDefinition.createSensorFrame(sensorDefinition.getName(),
                                                                                    joint.getSuccessor(),
                                                                                    new RigidBodyTransform(sensorDefinition.getTransformToJoint()));
               forceSensorDefinitionList.add(new ForceSensorDefinition(sensorDefinition.getName(), joint.getSuccessor(), sensorFrame));
            }
            else if (sensorDefinition instanceof CameraSensorDefinition)
            {
               cameraJoints.put(sensorDefinition.getName(), joint);
               cameraFrames.put(sensorDefinition.getName(),
                                new FixedReferenceFrame(sensorDefinition.getName(), joint.getFrameAfterJoint(), sensorDefinition.getTransformToJoint()));
            }
            else if (sensorDefinition instanceof LidarSensorDefinition)
            {
               lidarJoints.put(sensorDefinition.getName(), joint);
               lidarBaseToSensorTransform.put(sensorDefinition.getName(), new RigidBodyTransform(sensorDefinition.getTransformToJoint()));
            }
         }
      }

      imuDefinitions = imuDefinitionList.toArray(new IMUDefinition[0]);
      forceSensorDefinitions = forceSensorDefinitionList.toArray(new ForceSensorDefinition[0]);
   }

   /**
    * This method configures the sensor features, such as frames and joints.
    *
    * @param robotDescription
    */
   protected void setupRobotDescription(RobotDescription robotDescription)
   {
      cameraJoints = new HashMap<>();
      cameraFrames = new HashMap<>();
      lidarJoints = new HashMap<>();
      lidarBaseToSensorTransform = new HashMap<>();
      List<IMUDefinition> imuDefinitionList = new ArrayList<>();
      List<ForceSensorDefinition> forceSensorDefinitionList = new ArrayList<>();

      for (JointDescription jointDescription : collectAllJointDescriptions(robotDescription))
      {
         JointBasics joint = MultiBodySystemTools.findJoint(elevator, jointDescription.getName());

         for (IMUSensorDescription imuSensorDescription : jointDescription.getIMUSensors())
         {
            imuDefinitionList.add(new IMUDefinition(imuSensorDescription.getName(), joint.getSuccessor(), imuSensorDescription.getTransformToJoint()));
         }
         for (ForceSensorDescription forceSensorDescription : jointDescription.getForceSensors())
         {
            ReferenceFrame sensorFrame = ForceSensorDefinition.createSensorFrame(forceSensorDescription.getName(),
                                                                                 joint.getSuccessor(),
                                                                                 forceSensorDescription.getTransformToJoint());
            forceSensorDefinitionList.add(new ForceSensorDefinition(forceSensorDescription.getName(), joint.getSuccessor(), sensorFrame));
         }
         for (CameraSensorDescription cameraSensorDescription : jointDescription.getCameraSensors())
         {
            cameraJoints.put(cameraSensorDescription.getName(), joint);
            cameraFrames.put(cameraSensorDescription.getName(),
                             new FixedReferenceFrame(cameraSensorDescription.getName(),
                                                     joint.getFrameAfterJoint(),
                                                     cameraSensorDescription.getTransformToJoint()));
         }
         for (LidarSensorDescription lidarSensorDescription : jointDescription.getLidarSensors())
         {
            lidarJoints.put(lidarSensorDescription.getName(), joint);
            lidarBaseToSensorTransform.put(lidarSensorDescription.getName(), new RigidBodyTransform(lidarSensorDescription.getTransformToJoint()));
         }
      }

      imuDefinitions = imuDefinitionList.toArray(new IMUDefinition[0]);
      forceSensorDefinitions = forceSensorDefinitionList.toArray(new ForceSensorDefinition[0]);
   }

   /**
    * This method configures the following fields:
    * <ul>
    * <li>head
    * <li>end-effectors
    * <li>spine joints
    * <li>neck joints
    * <li>robot specific joint names
    * </ul>
    *
    * @param jointNameMap
    */
   protected void setupJointNameMap(JointNameMap<?> jointNameMap)
   {
      this.jointNameMap = jointNameMap;

      headBody = MultiBodySystemTools.findRigidBody(elevator, jointNameMap.getHeadName());

      endEffectors = new HashMap<>();

      for (String endEffectorParentJointName : jointNameMap.getEndEffectorJoints())
      {
         JointBasics endEffectorParentJoint = MultiBodySystemTools.findJoint(elevator, endEffectorParentJointName);
         if (endEffectorParentJoint == null)
            continue;
         Enum<?> robotSegment = jointNameMap.getEndEffectorsRobotSegment(endEffectorParentJointName);
         endEffectors.put(robotSegment, endEffectorParentJoint.getSuccessor());
      }

      neckJoints = new EnumMap<>(NeckJointName.class);
      spineJoints = new EnumMap<>(SpineJointName.class);

      for (OneDoFJointBasics oneDoFJoint : oneDoFJoints)
      {
         String jointName = oneDoFJoint.getName();
         JointRole jointRole = jointNameMap.getJointRole(jointName);
         if (jointRole == null)
            continue;
         switch (jointRole)
         {
            case NECK:
               neckJoints.put(jointNameMap.getNeckJointName(jointName), oneDoFJoint);
               break;
            case SPINE:
               spineJoints.put(jointNameMap.getSpineJointName(jointName), oneDoFJoint);
               break;
            default:
               break;
         }
      }
   }

   @Override
   public RigidBodyBasics getElevator()
   {
      return elevator;
   }

   @Override
   public FloatingJointBasics getRootJoint()
   {
      return rootJoint;
   }

   @Override
   public RigidBodyBasics getRootBody()
   {
      return rootBody;
   }

   @Override
   public OneDoFJointBasics[] getOneDoFJoints()
   {
      return oneDoFJoints;
   }

   @Override
   public Map<String, OneDoFJointBasics> getOneDoFJointsAsMap()
   {
      return jointNameToOneDoFJointMap;
   }

   @Override
   public RobotSpecificJointNames getRobotSpecificJointNames()
   {
      return jointNameMap;
   }

   @Override
   public RigidBodyBasics getEndEffector(Enum<?> segmentEnum)
   {
      return endEffectors.get(segmentEnum);
   }

   @Override
   public OneDoFJointBasics getSpineJoint(SpineJointName spineJointName)
   {
      return spineJoints.get(spineJointName);
   }

   @Override
   public OneDoFJointBasics getNeckJoint(NeckJointName neckJointName)
   {
      return neckJoints.get(neckJointName);
   }

   @Override
   public RigidBodyBasics getHead()
   {
      return headBody;
   }

   @Override
   public ReferenceFrame getCameraFrame(String name)
   {
      return cameraFrames.get(name);
   }

   @Override
   public JointBasics getLidarJoint(String lidarName)
   {
      return lidarJoints.get(lidarName);
   }

   @Override
   public RigidBodyTransform getLidarBaseToSensorTransform(String name)
   {
      return lidarBaseToSensorTransform.get(name);
   }

   @Override
   public IMUDefinition[] getIMUDefinitions()
   {
      return imuDefinitions;
   }

   @Override
   public ForceSensorDefinition[] getForceSensorDefinitions()
   {
      return forceSensorDefinitions;
   }

   @Override
   public double getTotalMass()
   {
      return TotalMassCalculator.computeSubTreeMass(elevator);
   }

   public static RigidBodyBasics instantiateRobot(RobotDescription robotDescription, ReferenceFrame rootFrame)
   {
      RigidBodyBasics elevator = new RigidBody("elevator", rootFrame);
      for (JointDescription rootJoint : robotDescription.getRootJoints())
         addJointRecursive(rootJoint, elevator);
      for (JointDescription rootJoint : robotDescription.getRootJoints())
         addLoopClosureConstraintRecursive(rootJoint, elevator);
      return elevator;
   }

   public static JointBasics addJointRecursive(JointDescription jointDescription, RigidBodyBasics predecessor)
   {
      JointBasics joint = createJoint(jointDescription, predecessor);
      RigidBodyBasics rigidBody = createRigidBody(jointDescription.getLink(), joint);

      for (JointDescription child : jointDescription.getChildrenJoints())
         addJointRecursive(child, rigidBody);
      return joint;
   }

   public static JointBasics createJoint(JointDescription jointDescription, RigidBodyBasics predecessor)
   {
      if (jointDescription instanceof FloatingJointDescription)
         return createSixDoFJoint((FloatingJointDescription) jointDescription, predecessor);
      if (jointDescription instanceof OneDoFJointDescription)
         return createOneDoFJoint((OneDoFJointDescription) jointDescription, predecessor);
      return null;
   }

   public static SixDoFJoint createSixDoFJoint(FloatingJointDescription jointDescription, RigidBodyBasics predecessor)
   {
      return new SixDoFJoint(jointDescription.getName(), predecessor, new RigidBodyTransform(new Quaternion(), jointDescription.getOffsetFromParentJoint()));
   }

   public static OneDoFJointBasics createOneDoFJoint(OneDoFJointDescription jointDescription, RigidBodyBasics predecessor)
   {
      if (jointDescription instanceof CrossFourBarJointDescription)
         return createCrossFourBarJoint((CrossFourBarJointDescription) jointDescription, predecessor);
      if (jointDescription instanceof PinJointDescription)
         return createRevoluteJoint((PinJointDescription) jointDescription, predecessor);
      if (jointDescription instanceof SliderJointDescription)
         return createPrismaticJoint((SliderJointDescription) jointDescription, predecessor);
      return null;
   }

   public static RevoluteJoint createRevoluteJoint(PinJointDescription jointDescription, RigidBodyBasics predecessor)
   {
      RevoluteJoint revoluteJoint = new RevoluteJoint(jointDescription.getName(),
                                                      predecessor,
                                                      jointDescription.getOffsetFromParentJoint(),
                                                      jointDescription.getJointAxis());
      if (jointDescription.containsLimitStops())
         revoluteJoint.setJointLimits(jointDescription.getLowerLimit(), jointDescription.getUpperLimit());
      revoluteJoint.setVelocityLimit(jointDescription.getVelocityLimit());
      revoluteJoint.setEffortLimit(jointDescription.getEffortLimit());
      return revoluteJoint;
   }

   public static PrismaticJoint createPrismaticJoint(SliderJointDescription jointDescription, RigidBodyBasics predecessor)
   {
      PrismaticJoint prismaticJoint = new PrismaticJoint(jointDescription.getName(),
                                                         predecessor,
                                                         jointDescription.getOffsetFromParentJoint(),
                                                         jointDescription.getJointAxis());
      if (jointDescription.containsLimitStops())
         prismaticJoint.setJointLimits(jointDescription.getLowerLimit(), jointDescription.getUpperLimit());
      prismaticJoint.setVelocityLimit(jointDescription.getVelocityLimit());
      prismaticJoint.setEffortLimit(jointDescription.getEffortLimit());
      return prismaticJoint;
   }

   public static CrossFourBarJoint createCrossFourBarJoint(CrossFourBarJointDescription jointDescription, RigidBodyBasics predecessor)
   {
      return new CrossFourBarJoint(jointDescription.getName(),
                                   predecessor,
                                   jointDescription.getJointNameA(),
                                   jointDescription.getJointNameB(),
                                   jointDescription.getJointNameC(),
                                   jointDescription.getJointNameD(),
                                   jointDescription.getBodyDA().getName(),
                                   jointDescription.getBodyBC().getName(),
                                   jointDescription.getTransformAToPredecessor(),
                                   jointDescription.getTransformBToPredecessor(),
                                   jointDescription.getTransformDToA(),
                                   jointDescription.getTransformCToB(),
                                   jointDescription.getBodyDA().getMomentOfInertiaCopy(),
                                   jointDescription.getBodyBC().getMomentOfInertiaCopy(),
                                   jointDescription.getBodyDA().getMass(),
                                   jointDescription.getBodyBC().getMass(),
                                   new RigidBodyTransform(new Quaternion(), jointDescription.getBodyDA().getCenterOfMassOffset()),
                                   new RigidBodyTransform(new Quaternion(), jointDescription.getBodyBC().getCenterOfMassOffset()),
                                   jointDescription.getActuatedJointIndex(),
                                   jointDescription.getLoopClosureJointIndex(),
                                   jointDescription.getJointAxis());
   }

   public static void addLoopClosureConstraintRecursive(JointDescription jointDescription, RigidBodyBasics parentBody)
   {
      JointBasics joint = parentBody.getChildrenJoints().stream().filter(child -> child.getName().equals(jointDescription.getName())).findFirst().get();
      RigidBodyBasics constraintPredecessor = joint.getSuccessor();

      List<LoopClosureConstraintDescription> constraintDescriptions = jointDescription.getChildrenConstraintDescriptions();

      for (LoopClosureConstraintDescription constraintDescription : constraintDescriptions)
      {
         RigidBodyBasics constraintSuccessor = MultiBodySystemTools.getRootBody(parentBody).subtreeStream()
                                                                   .filter(body -> body.getName().equals(constraintDescription.getLink().getName())).findFirst()
                                                                   .get();
         createLoopClosureJoint(constraintPredecessor, constraintSuccessor, constraintDescription);
      }

      for (JointDescription childJoint : jointDescription.getChildrenJoints())
         addLoopClosureConstraintRecursive(childJoint, constraintPredecessor);
   }

   public static RevoluteJoint createLoopClosureJoint(RigidBodyBasics predecessor,
                                                      RigidBodyBasics successor,
                                                      LoopClosureConstraintDescription constraintDescription)
   {
      RevoluteJoint constraintJoint;
      String name = constraintDescription.getName();
      Vector3DBasics offsetFromParentJoint = constraintDescription.getOffsetFromParentJoint();
      Vector3DBasics offsetFromLinkParentJoint = constraintDescription.getOffsetFromLinkParentJoint();

      if (constraintDescription instanceof LoopClosurePinConstraintDescription)
      {
         Vector3DBasics axis = ((LoopClosurePinConstraintDescription) constraintDescription).getAxis();
         if (!constraintDescription.getParentJoint().getName().equals(predecessor.getParentJoint().getName()))
            throw new RuntimeException("Constraint predecessor mismatch.");
         if (!constraintDescription.getLink().getName().equals(successor.getName()))
            throw new RuntimeException("Constraint successor mismatch.");
         constraintJoint = new RevoluteJoint(name, predecessor, offsetFromParentJoint, axis);
         constraintJoint.setupLoopClosure(successor, new RigidBodyTransform(new Quaternion(), offsetFromLinkParentJoint));
      }
      else
      {
         LogTools.error("The constraint type {} is not handled, skipping it.", constraintDescription.getClass().getSimpleName());
         constraintJoint = null;
      }
      return constraintJoint;
   }

   public static RigidBodyBasics createRigidBody(LinkDescription linkDescription, JointBasics parentJoint)
   {
      return new RigidBody(linkDescription.getName(),
                           parentJoint,
                           linkDescription.getMomentOfInertiaCopy(),
                           linkDescription.getMass(),
                           new RigidBodyTransform(new Quaternion(), linkDescription.getCenterOfMassOffset()));
   }

   public static List<JointDescription> collectAllJointDescriptions(RobotDescription robotDescription)
   {
      List<JointDescription> joints = new ArrayList<>();
      for (JointDescription rootJoint : robotDescription.getRootJoints())
         collectAllJointDescriptionsRecursive(rootJoint, joints);
      return joints;
   }

   private static void collectAllJointDescriptionsRecursive(JointDescription jointDescription, List<JointDescription> jointsToPack)
   {
      jointsToPack.add(jointDescription);
      for (JointDescription child : jointDescription.getChildrenJoints())
         collectAllJointDescriptionsRecursive(child, jointsToPack);
   }
}
