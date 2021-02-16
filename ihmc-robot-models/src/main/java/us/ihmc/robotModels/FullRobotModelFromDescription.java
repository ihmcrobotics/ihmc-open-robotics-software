package us.ihmc.robotModels;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.description.InvertedFourBarJointDescription;
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
import us.ihmc.robotics.screwTheory.InvertedFourBarJoint;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;

public class FullRobotModelFromDescription implements FullRobotModel
{
   protected final RobotDescription description;

   protected final JointNameMap sdfJointNameMap;
   protected final EnumMap<NeckJointName, OneDoFJointBasics> neckJoints = new EnumMap<>(NeckJointName.class);
   protected final EnumMap<SpineJointName, OneDoFJointBasics> spineJoints = new EnumMap<>(SpineJointName.class);
   protected final String[] sensorLinksToTrack;
   //   protected final SDFLinkHolder rootLink;
   protected RigidBodyBasics head;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final RigidBodyBasics elevator;
   protected final SixDoFJoint rootJoint;
   private final RigidBodyBasics rootLink;
   private final LinkedHashMap<String, OneDoFJointBasics> oneDoFJoints = new LinkedHashMap<String, OneDoFJointBasics>();
   private final OneDoFJointBasics[] oneDoFJointsAsArray;
   private final ArrayList<IMUDefinition> imuDefinitions = new ArrayList<IMUDefinition>();
   private final ArrayList<ForceSensorDefinition> forceSensorDefinitions = new ArrayList<ForceSensorDefinition>();
   private final HashMap<String, ReferenceFrame> cameraFrames = new HashMap<String, ReferenceFrame>();
   private final HashMap<String, ReferenceFrame> lidarBaseFrames = new HashMap<String, ReferenceFrame>();
   private final HashMap<String, RigidBodyTransform> lidarBaseToSensorTransform = new HashMap<String, RigidBodyTransform>();
   private final HashMap<String, JointBasics> lidarJoints = new HashMap<>();
   private final HashMap<String, ReferenceFrame> sensorFrames = new HashMap<String, ReferenceFrame>();
   private double totalMass = 0.0;
   private final boolean alignReferenceFramesWithJoints;

   private final Map<Enum<?>, RigidBodyBasics> endEffectors = new HashMap<>();

   public FullRobotModelFromDescription(FullRobotModelFromDescription modelToCopy)
   {
      this(modelToCopy.description, modelToCopy.sdfJointNameMap, modelToCopy.sensorLinksToTrack);
   }

   public FullRobotModelFromDescription(RobotDescription description, JointNameMap sdfJointNameMap, String[] sensorLinksToTrack)
   {
      this(description, sdfJointNameMap, sensorLinksToTrack, false);
   }

   public FullRobotModelFromDescription(RobotDescription description, JointNameMap sdfJointNameMap, String[] sensorLinksToTrack,
                                        boolean makeReferenceFramesAlignWithTheJoints)
   {
      super();
      this.description = description;

      if (description.getRootJoints().size() != 1)
      {
         throw new RuntimeException("Must be exactly one root joint and it must be a FloatingJoint!");
      }

      FloatingJointDescription rootJointDescription = (FloatingJointDescription) description.getRootJoints().get(0);

      this.sdfJointNameMap = sdfJointNameMap;
      this.sensorLinksToTrack = sensorLinksToTrack;
      this.alignReferenceFramesWithJoints = makeReferenceFramesAlignWithTheJoints;

      /*
       * Create root object
       */
      elevator = new RigidBody("elevator", worldFrame);
      rootJoint = new SixDoFJoint(rootJointDescription.getName(), elevator);

      //      System.out.println("Adding rigid body " + rootLink.getName() + "; Mass: " + rootLink.getMass() + "; ixx: " + rootLink.getInertia().m00 + "; iyy: " + rootLink.getInertia().m11
      //            + "; izz: " + rootLink.getInertia().m22 + "; COM Offset: " + rootLink.getCoMOffset());
      LinkDescription rootLinkDescription = rootJointDescription.getLink();
      rootLink = new RigidBody(rootJointDescription.getName(),
                               rootJoint,
                               rootLinkDescription.getMomentOfInertiaCopy(),
                               rootLinkDescription.getMass(),
                               rootLinkDescription.getCenterOfMassOffset());

      checkLinkIsNeededForSensor(rootJoint, rootJointDescription);
      addSensorDefinitions(rootJoint, rootJointDescription);

      if (rootLink.getName().equals(sdfJointNameMap.getHeadName()))
      {
         head = rootLink;
      }

      totalMass = rootJointDescription.getLink().getMass();
      for (JointDescription jointDescription : rootJointDescription.getChildrenJoints())
      {
         addJointsRecursively((OneDoFJointDescription) jointDescription, rootLink);
      }

      for (JointDescription jointDescription : rootJointDescription.getChildrenJoints())
      {
         addLoopClosureConstraintRecursive(jointDescription, rootLink);
      }

      oneDoFJointsAsArray = new OneDoFJointBasics[oneDoFJoints.size()];
      oneDoFJoints.values().toArray(oneDoFJointsAsArray);
   }

   @Override
   public RigidBodyBasics getEndEffector(Enum<?> segmentEnum)
   {
      return endEffectors.get(segmentEnum);
   }

//   public String getModelName()
//   {
//      return sdfJointNameMap.getModelName();
//   }

   protected void addSensorDefinitions(JointBasics joint, JointDescription jointDescription)
   {
      List<IMUSensorDescription> imuSensors = jointDescription.getIMUSensors();

      // The linkRotation transform is to make sure that the linkToSensor is in a zUpFrame.
//      RigidBodyTransform linkRotation = new RigidBodyTransform(child.getTransformFromModelReferenceFrame());
//      linkRotation.setTranslation(0.0, 0.0, 0.0);
//      RigidBodyTransform linkToSensorInZUp = new RigidBodyTransform();
//      linkToSensorInZUp.multiply(linkRotation, SDFConversionsHelper.poseToTransform(sensor.getPose()));

      for (IMUSensorDescription imuSensor : imuSensors)
      {
         IMUDefinition imuDefinition = new IMUDefinition(imuSensor.getName(), joint.getSuccessor(), imuSensor.getTransformToJoint());
         imuDefinitions.add(imuDefinition);
      }

      for (CameraSensorDescription cameraSensor : jointDescription.getCameraSensors())
      {
         ReferenceFrame cameraFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(cameraSensor.getName(),
                                                                                                        joint.getFrameAfterJoint(),
                                                                                                        cameraSensor.getTransformToJoint());
         cameraFrames.put(cameraFrame.getName(), cameraFrame);
      }

      for (LidarSensorDescription lidarSensor : jointDescription.getLidarSensors())
      {
         ReferenceFrame lidarFrame = joint.getFrameAfterJoint();
         lidarBaseFrames.put(lidarSensor.getName(), lidarFrame);
         lidarBaseToSensorTransform.put(lidarSensor.getName(), lidarSensor.getTransformToJoint());
         lidarJoints.put(lidarSensor.getName(), joint);
      }

   }

   /** {@inheritDoc} */
   @Override
   public RobotSpecificJointNames getRobotSpecificJointNames()
   {
      return sdfJointNameMap;
   }

   /** {@inheritDoc} */
   @Override
   public synchronized void updateFrames()
   {
      elevator.updateFramesRecursively();
   }

   /** {@inheritDoc} */
   @Override
   public MovingReferenceFrame getElevatorFrame()
   {
      return elevator.getBodyFixedFrame();
   }

   /** {@inheritDoc} */
   @Override
   public SixDoFJoint getRootJoint()
   {
      return rootJoint;
   }

   /** {@inheritDoc} */
   @Override
   public RigidBodyBasics getElevator()
   {
      return elevator;
   }

   /** {@inheritDoc} */
   @Override
   public OneDoFJointBasics getSpineJoint(SpineJointName spineJointName)
   {
      return spineJoints.get(spineJointName);
   }

   /** {@inheritDoc} */
   @Override
   public OneDoFJointBasics getNeckJoint(NeckJointName neckJointName)
   {
      return neckJoints.get(neckJointName);
   }

   /** {@inheritDoc} */
   @Override
   public JointBasics getLidarJoint(String lidarName)
   {
      return lidarJoints.get(lidarName);
   }

   /** {@inheritDoc} */
   @Override
   public RigidBodyBasics getRootBody()
   {
      return rootLink;
   }

   /** {@inheritDoc} */
   @Override
   public RigidBodyBasics getHead()
   {
      return head;
   }

   /** {@inheritDoc} */
   @Override
   public OneDoFJointBasics[] getOneDoFJoints()
   {
      return oneDoFJointsAsArray;
   }

   /** {@inheritDoc} */
   @Override
   public void getOneDoFJoints(List<OneDoFJointBasics> oneDoFJointsToPack)
   {
      Collection<OneDoFJointBasics> values = oneDoFJoints.values();
      oneDoFJointsToPack.addAll(values);
   }

   /** {@inheritDoc} */
   @Override
   public OneDoFJointBasics[] getControllableOneDoFJoints()
   {
      return getOneDoFJoints();
   }

   /** {@inheritDoc} */
   @Override
   public void getControllableOneDoFJoints(List<OneDoFJointBasics> oneDoFJointsToPack)
   {
      getOneDoFJoints(oneDoFJointsToPack);
   }

   @Override
   public Map<String, OneDoFJointBasics> getOneDoFJointsAsMap()
   {
      return Collections.unmodifiableMap(oneDoFJoints);
   }

   @Override
   public OneDoFJointBasics getOneDoFJointByName(String name)
   {
      return oneDoFJoints.get(name);
   }

   /** {@inheritDoc} */
   @Override
   public IMUDefinition[] getIMUDefinitions()
   {
      IMUDefinition[] imuDefinitions = new IMUDefinition[this.imuDefinitions.size()];
      this.imuDefinitions.toArray(imuDefinitions);
      return imuDefinitions;
   }

   /** {@inheritDoc} */
   @Override
   public ForceSensorDefinition[] getForceSensorDefinitions()
   {
      return this.forceSensorDefinitions.toArray(new ForceSensorDefinition[this.forceSensorDefinitions.size()]);
   }

   @Override
   public List<String> getLidarSensorNames()
   {
      return new ArrayList<>(lidarBaseFrames.keySet());
   }

   @Override
   public List<String> getCameraSensorNames()
   {
      return new ArrayList<>(cameraFrames.keySet());
   }

   @Override
   public ReferenceFrame getCameraFrame(String name)
   {
      return cameraFrames.get(name);
   }

   @Override
   public ReferenceFrame getLidarBaseFrame(String name)
   {
      //      for(String st : lidarBaseFrames.keySet())
      //      {
      //         System.out.println(st);
      //      }
      return lidarBaseFrames.get(name);
   }

   @Override
   public RigidBodyTransform getLidarBaseToSensorTransform(String name)
   {
      return lidarBaseToSensorTransform.get(name);
   }

   @Override
   public ReferenceFrame getHeadBaseFrame()
   {
      if (head != null)
      {
         JointBasics headJoint = head.getParentJoint();
         return headJoint.getFrameAfterJoint();
      }
      return null;
   }

   protected void checkLinkIsNeededForSensor(JointBasics joint, JointDescription jointDescription)
   {
      if (sensorLinksToTrack != null)
      {
         for (int i = 0; i < sensorLinksToTrack.length; i++)
         {
            if (sensorLinksToTrack[i].equalsIgnoreCase(jointDescription.getName()))
            {
               sensorFrames.put(jointDescription.getName(), joint.getFrameAfterJoint());
            }
         }
      }
   }

   public ReferenceFrame getSensorReferenceFrameByLink(String linkName)
   {
      if (linkName == null)
      {
         System.err.println("SDFFullRobotModel getSensorReferenceFrameByLink: Passed in NULL link name for Sensor pose");
         return null;
      }
      linkName = linkName.replace("/", "");
      if (!sensorFrames.containsKey(linkName))
      {
         System.err.println("SDFFullRobotModel getSensorReferenceFrameByLink: got null for linkName: " + linkName);
      }
      return sensorFrames.get(linkName);
   }

   protected void addJointsRecursively(OneDoFJointDescription jointDescription, RigidBodyBasics parentBody)
   {
      OneDoFJointBasics joint = createOneDoFJoint(jointDescription, parentBody);

      oneDoFJoints.put(jointDescription.getName(), joint);

      checkLinkIsNeededForSensor(joint, jointDescription);

      RigidBodyBasics rigidBody = createRigidBody(jointDescription.getLink(), joint);

      totalMass += rigidBody.getInertia().getMass();

      mapRigidBody(jointDescription, joint, rigidBody);
      addSensorDefinitions(joint, jointDescription);

      for (ForceSensorDescription sdfForceSensor : jointDescription.getForceSensors())
      {
         ReferenceFrame sensorFrame = ForceSensorDefinition.createSensorFrame(sdfForceSensor.getName(),
                                                                              joint.getSuccessor(),
                                                                              sdfForceSensor.getTransformToJoint());
         ForceSensorDefinition forceSensorDefinition = new ForceSensorDefinition(sdfForceSensor.getName(), joint.getSuccessor(), sensorFrame);
         forceSensorDefinitions.add(forceSensorDefinition);
      }

      for (JointDescription sdfJoint : jointDescription.getChildrenJoints())
      {
         addJointsRecursively((OneDoFJointDescription) sdfJoint, rigidBody);
      }
   }

   public static RigidBodyBasics createRigidBody(LinkDescription linkDescription, OneDoFJointBasics parentJoint)
   {
      double mass = linkDescription.getMass();
      Vector3D comOffset = new Vector3D(linkDescription.getCenterOfMassOffset());
      Matrix3D inertia = linkDescription.getMomentOfInertiaCopy();
      return new RigidBody(linkDescription.getName(), parentJoint, inertia, mass, comOffset);
   }

   public static OneDoFJointBasics createOneDoFJoint(OneDoFJointDescription jointDescription, RigidBodyBasics predecessor)
   {
      if (jointDescription instanceof InvertedFourBarJointDescription)
      {
         return createInvertedFourBarJoint(jointDescription, predecessor);
      }
      else
      {
         Vector3D jointAxis = new Vector3D();
         jointDescription.getJointAxis(jointAxis);

         Vector3D offset = new Vector3D();
         jointDescription.getOffsetFromParentJoint(offset);

         OneDoFJointBasics inverseDynamicsJoint;

         if (jointDescription instanceof PinJointDescription)
         {
            inverseDynamicsJoint = new RevoluteJoint(jointDescription.getName(), predecessor, offset, jointAxis);
         }
         else if (jointDescription instanceof SliderJointDescription)
         {
            inverseDynamicsJoint = new PrismaticJoint(jointDescription.getName(), predecessor, offset, jointAxis);
         }
         else
         {
            throw new RuntimeException("Must be either Pin or Slider here!");
         }

         inverseDynamicsJoint.setEffortLimits(-jointDescription.getEffortLimit(), jointDescription.getEffortLimit());
         inverseDynamicsJoint.setVelocityLimits(-jointDescription.getVelocityLimit(), jointDescription.getVelocityLimit());

         if (jointDescription.containsLimitStops())
         {
            double[] limitStopParameters = jointDescription.getLimitStopParameters();
            inverseDynamicsJoint.setJointLimitLower(limitStopParameters[0]);
            inverseDynamicsJoint.setJointLimitUpper(limitStopParameters[1]);
         }

         return inverseDynamicsJoint;
      }
   }

   private static OneDoFJointBasics createInvertedFourBarJoint(OneDoFJointDescription jointDescription, RigidBodyBasics predecessor)
   {
      InvertedFourBarJointDescription fourBarDescription = (InvertedFourBarJointDescription) jointDescription;

      RevoluteJoint jointA = null, jointB = null, jointC = null, jointD = null;
      PinJointDescription jointADescription = null, jointBDescription = null, jointCDescription = null, jointDDescription = null;

      PinJointDescription[] fourBarJoints = fourBarDescription.getFourBarJoints();
      PinJointDescription masterJointDescription = fourBarDescription.getFourBarJoints()[fourBarDescription.getMasterJointIndex()];

      for (PinJointDescription pinJointDescription : fourBarJoints)
      {
         if (pinJointDescription.getParentJoint().getLink().getName().equals(predecessor.getName()))
         {
            if (jointA == null)
            {
               jointADescription = pinJointDescription;
               jointA = (RevoluteJoint) createOneDoFJoint(jointADescription, predecessor);
            }
            else
            {
               jointBDescription = pinJointDescription;
               jointB = (RevoluteJoint) createOneDoFJoint(jointBDescription, predecessor);
            }
         }
      }

      for (PinJointDescription pinJointDescription : fourBarJoints)
      {
         if (pinJointDescription.getParentJoint() == jointADescription)
            jointDDescription = pinJointDescription;
         else if (pinJointDescription.getParentJoint() == jointBDescription)
            jointCDescription = pinJointDescription;
      }

      RigidBodyBasics bodyAD = createRigidBody(jointADescription.getLink(), jointA);
      RigidBodyBasics bodyCD = null;

      if (jointDDescription != null)
      {
         jointD = (RevoluteJoint) createOneDoFJoint(jointDDescription, bodyAD);
         bodyCD = createRigidBody(jointDDescription.getLink(), jointD);
      }

      RigidBodyBasics bodyBC = createRigidBody(jointBDescription.getLink(), jointB);

      if (jointCDescription != null)
      {
         jointC = (RevoluteJoint) createOneDoFJoint(jointCDescription, bodyBC);
         bodyCD = createRigidBody(jointCDescription.getLink(), jointC);
      }

      if ((jointC == null) == (jointD == null))
         throw new IllegalStateException("Unexpected four-bar configuration");

      if (jointC == null)
      {
         LoopClosurePinConstraintDescription jointCConstraint = fourBarDescription.getFourBarClosure();
         if (jointCConstraint.getParentJoint() == jointDDescription)
            jointC = createLoopClosureJoint(bodyCD, bodyBC, jointCConstraint);
         else
            jointC = createLoopClosureJoint(bodyBC, bodyCD, jointCConstraint);
      }
      else
      {
         LoopClosurePinConstraintDescription jointDConstraint = fourBarDescription.getFourBarClosure();
         if (jointDConstraint.getParentJoint() == jointCDescription)
            jointD = createLoopClosureJoint(bodyCD, bodyAD, jointDConstraint);
         else
            jointD = createLoopClosureJoint(bodyAD, bodyCD, jointDConstraint);
      }

      RevoluteJoint[] fourBarRevoluteJoints = new RevoluteJoint[] {jointA, jointB, jointC, jointD};
      int masterJointIndex = -1;
      if (masterJointDescription == jointADescription)
         masterJointIndex = 0;
      else if (masterJointDescription == jointBDescription)
         masterJointIndex = 1;
      else if (masterJointDescription == jointCDescription)
         masterJointIndex = 2;
      else if (masterJointDescription == jointDDescription)
         masterJointIndex = 3;
      return new InvertedFourBarJoint(fourBarDescription.getName(), fourBarRevoluteJoints, masterJointIndex);
   }

   protected void addLoopClosureConstraintRecursive(JointDescription jointDescription, RigidBodyBasics parentBody)
   {
      JointBasics joint = parentBody.getChildrenJoints().stream().filter(child -> child.getName().equals(jointDescription.getName())).findFirst().get();
      RigidBodyBasics constraintPredecessor = joint.getSuccessor();

      List<LoopClosureConstraintDescription> constraintDescriptions = jointDescription.getChildrenConstraintDescriptions();

      for (LoopClosureConstraintDescription constraintDescription : constraintDescriptions)
      {
         RigidBodyBasics constraintSuccessor = MultiBodySystemTools.getRootBody(parentBody).subtreeStream()
                                                                   .filter(body -> body.getName().equals(constraintDescription.getLink().getName())).findFirst()
                                                                   .get();
         RevoluteJoint constraintJoint = createLoopClosureJoint(constraintPredecessor, constraintSuccessor, constraintDescription);
         if (constraintJoint != null)
            oneDoFJoints.put(constraintJoint.getName(), constraintJoint);
      }

      for (JointDescription childJoint : jointDescription.getChildrenJoints())
         addLoopClosureConstraintRecursive(childJoint, constraintPredecessor);
   }

   public static RevoluteJoint createLoopClosureJoint(RigidBodyBasics predecessor, RigidBodyBasics successor,
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

   protected void mapRigidBody(JointDescription joint, OneDoFJointBasics inverseDynamicsJoint, RigidBodyBasics rigidBody)
   {
      if (rigidBody.getName().equals(sdfJointNameMap.getHeadName()))
      {
         head = rigidBody;
      }

      Set<String> endEffectorJoints = sdfJointNameMap.getEndEffectorJoints();
      if (endEffectorJoints != null && endEffectorJoints.contains(inverseDynamicsJoint.getName()))
      {
         Enum<?> endEffectorRobotSegment = sdfJointNameMap.getEndEffectorsRobotSegment(inverseDynamicsJoint.getName());
         endEffectors.put(endEffectorRobotSegment, rigidBody);
      }

      JointRole jointRole = sdfJointNameMap.getJointRole(joint.getName());
      if (jointRole != null)
      {
         switch (jointRole)
         {
            //TODO: Should armJointLists use legJoingName.first or armJointName.first?? looks backwards
            case NECK:
               NeckJointName neckJointName = sdfJointNameMap.getNeckJointName(joint.getName());
               neckJoints.put(neckJointName, inverseDynamicsJoint);
               break;
            case SPINE:
               SpineJointName spineJointName = sdfJointNameMap.getSpineJointName(joint.getName());
               spineJoints.put(spineJointName, inverseDynamicsJoint);
               break;
            default:
               break;
         }
      }
   }

   @Override
   public void getOneDoFJointsFromRootToHere(OneDoFJointBasics oneDoFJointAtEndOfChain, List<OneDoFJointBasics> oneDoFJointsToPack)
   {
      oneDoFJointsToPack.clear();
      JointBasics parent = oneDoFJointAtEndOfChain;

      while (parent != rootJoint)
      {
         if (parent instanceof OneDoFJointBasics)
         {
            oneDoFJointsToPack.add((OneDoFJointBasics) parent);
         }

         parent = parent.getPredecessor().getParentJoint();
      }

      Collections.reverse(oneDoFJointsToPack);
   }

   @Override
   public double getTotalMass()
   {
      return totalMass;
   }

}