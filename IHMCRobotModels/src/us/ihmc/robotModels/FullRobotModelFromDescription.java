package us.ihmc.robotModels;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.partNames.*;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotDescription.*;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.robotics.sensors.ContactSensorDefinition;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.tools.containers.ContainerTools;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import java.util.*;

public class FullRobotModelFromDescription implements FullRobotModel
{
   protected final RobotDescription description;

   protected final JointNameMap sdfJointNameMap;
   protected final EnumMap<NeckJointName, OneDoFJoint> neckJoints = ContainerTools.createEnumMap(NeckJointName.class);
   protected final EnumMap<SpineJointName, OneDoFJoint> spineJoints = ContainerTools.createEnumMap(SpineJointName.class);
   protected final String[] sensorLinksToTrack;
//   protected final SDFLinkHolder rootLink;
   protected RigidBody chest;
   protected RigidBody head;

   private final RigidBody pelvis;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final RigidBody elevator;
   private final SixDoFJoint rootJoint;
   private final LinkedHashMap<String, OneDoFJoint> oneDoFJoints = new LinkedHashMap<String, OneDoFJoint>();
   private final ArrayList<IMUDefinition> imuDefinitions = new ArrayList<IMUDefinition>();
   private final ArrayList<ForceSensorDefinition> forceSensorDefinitions = new ArrayList<ForceSensorDefinition>();
   private final ArrayList<ContactSensorDefinition> contactSensorDefinitions = new ArrayList<ContactSensorDefinition>();
   private final HashMap<String, ReferenceFrame> cameraFrames = new HashMap<String, ReferenceFrame>();
   private final HashMap<String, ReferenceFrame> lidarBaseFrames = new HashMap<String, ReferenceFrame>();
   private final HashMap<String, RigidBodyTransform> lidarBaseToSensorTransform = new HashMap<String, RigidBodyTransform>();
   private final HashMap<String, InverseDynamicsJoint> lidarJoints = new HashMap<>();
   private final HashMap<String, ReferenceFrame> sensorFrames = new HashMap<String, ReferenceFrame>();
   private double totalMass = 0.0;
   private final boolean alignReferenceFramesWithJoints;

   private final Map<Enum<?>, RigidBody> endEffectors = new HashMap<>();

   public FullRobotModelFromDescription(FullRobotModelFromDescription modelToCopy)
   {
      this(modelToCopy.description, modelToCopy.sdfJointNameMap, modelToCopy.sensorLinksToTrack);
   }
   
   public FullRobotModelFromDescription(RobotDescription description, JointNameMap sdfJointNameMap, String[] sensorLinksToTrack)
   {
      this(description, sdfJointNameMap, sensorLinksToTrack, false);
   }

   public FullRobotModelFromDescription(RobotDescription description, JointNameMap sdfJointNameMap, String[] sensorLinksToTrack, boolean makeReferenceFramesAlignWithTheJoints)
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
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      elevator = new RigidBody("elevator", elevatorFrame);
      rootJoint = new SixDoFJoint(rootJointDescription.getName(), elevator, elevatorFrame);
      if (!rootJointDescription.getName().equals(sdfJointNameMap.getPelvisName()))
      {
         throw new RuntimeException("Pelvis joint is assumed to be the root joint");
      }

      //      System.out.println("Adding rigid body " + rootLink.getName() + "; Mass: " + rootLink.getMass() + "; ixx: " + rootLink.getInertia().m00 + "; iyy: " + rootLink.getInertia().m11
      //            + "; izz: " + rootLink.getInertia().m22 + "; COM Offset: " + rootLink.getCoMOffset());
      LinkDescription rootLinkDescription = rootJointDescription.getLink();
      pelvis = ScrewTools.addRigidBody(rootJointDescription.getName(), rootJoint, rootLinkDescription.getMomentOfInertiaCopy(), rootLinkDescription.getMass(), rootLinkDescription.getCenterOfMassOffset());

      checkLinkIsNeededForSensor(rootJoint, rootJointDescription);
      addSensorDefinitions(rootJoint, rootJointDescription);

      if (pelvis.getName().equals(sdfJointNameMap.getHeadName()))
      {
         head = pelvis;
      }

      totalMass = rootJointDescription.getLink().getMass();
      for (JointDescription jointDescription : rootJointDescription.getChildrenJoints())
      {
         addJointsRecursively((OneDoFJointDescription) jointDescription, pelvis);
      }
   }

   @Override
   public RigidBody getEndEffector(Enum<?> segmentEnum)
   {
      return endEffectors.get(segmentEnum);
   }

//   public String getModelName()
//   {
//      return sdfJointNameMap.getModelName();
//   }

   protected void addSensorDefinitions(InverseDynamicsJoint joint, JointDescription jointDescription)
   {
      ArrayList<IMUSensorDescription> imuSensors = jointDescription.getIMUSensors();

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
         ReferenceFrame cameraFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent(cameraSensor.getName(), joint.getFrameAfterJoint(), cameraSensor.getTransformToJoint());
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
   public ReferenceFrame getWorldFrame()
   {
      return worldFrame;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getElevatorFrame()
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
   public RigidBody getElevator()
   {
      return elevator;
   }

   /** {@inheritDoc} */
   @Override
   public OneDoFJoint getSpineJoint(SpineJointName spineJointName)
   {
      return spineJoints.get(spineJointName);
   }

   /** {@inheritDoc} */
   @Override
   public OneDoFJoint getNeckJoint(NeckJointName neckJointName)
   {
      return neckJoints.get(neckJointName);
   }

   /** {@inheritDoc} */
   @Override
   public InverseDynamicsJoint getLidarJoint(String lidarName)
   {
      return lidarJoints.get(lidarName);
   }

   /** {@inheritDoc} */
   @Override
   public RigidBody getPelvis()
   {
      return pelvis;
   }

   /** {@inheritDoc} */
   @Override
   public RigidBody getChest()
   {
      return chest;
   }

   /** {@inheritDoc} */
   @Override
   public RigidBody getHead()
   {
      return head;
   }

   /** {@inheritDoc} */
   @Override
   public OneDoFJoint[] getOneDoFJoints()
   {
      OneDoFJoint[] oneDoFJointsAsArray = new OneDoFJoint[oneDoFJoints.size()];
      oneDoFJoints.values().toArray(oneDoFJointsAsArray);
      return oneDoFJointsAsArray;
   }

   /** {@inheritDoc} */
   @Override
   public void getOneDoFJoints(ArrayList<OneDoFJoint> oneDoFJointsToPack)
   {
      Collection<OneDoFJoint> values = oneDoFJoints.values();
      oneDoFJointsToPack.addAll(values);
   }

   /** {@inheritDoc} */
   @Override
   public OneDoFJoint[] getControllableOneDoFJoints()
   {
      return getOneDoFJoints();
   }

   /** {@inheritDoc} */
   @Override
   public void getControllableOneDoFJoints(ArrayList<OneDoFJoint> oneDoFJointsToPack)
   {
      getOneDoFJoints(oneDoFJointsToPack);
   }

   @Override
   public Map<String, OneDoFJoint> getOneDoFJointsAsMap()
   {
      return Collections.unmodifiableMap(oneDoFJoints);
   }

   @Override
   public OneDoFJoint getOneDoFJointByName(String name)
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

   /** {@inheritDoc} */
   @Override
   public ContactSensorDefinition[] getContactSensorDefinitions()
   {
      return this.contactSensorDefinitions.toArray(new ContactSensorDefinition[this.contactSensorDefinitions.size()]);
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
      return head.getParentJoint().getFrameAfterJoint();
   }

   protected void checkLinkIsNeededForSensor(InverseDynamicsJoint joint, JointDescription jointDescription)
   {
      if(sensorLinksToTrack != null)
      {
         for(int i = 0; i < sensorLinksToTrack.length; i++)
         {
            if(sensorLinksToTrack[i].equalsIgnoreCase(jointDescription.getName()));
            {
               sensorFrames.put(jointDescription.getName(),joint.getFrameAfterJoint());
            }
         }         
      }
   }

   public ReferenceFrame getSensorReferenceFrameByLink(String linkName)
   {
      if(linkName == null)
      {
         System.err.println("SDFFullRobotModel getSensorReferenceFrameByLink: Passed in NULL link name for Sensor pose" );
         return null;
      }
      linkName = linkName.replace("/", "");
      if(!sensorFrames.containsKey(linkName))
      {
         System.err.println("SDFFullRobotModel getSensorReferenceFrameByLink: got null for linkName: " + linkName);
      }
      return sensorFrames.get(linkName);
   }

   protected void addJointsRecursively(OneDoFJointDescription joint, RigidBody parentBody)
   {
      Vector3d jointAxis = new Vector3d();
      joint.getJointAxis(jointAxis);

      Vector3d offset = new Vector3d();
      joint.getOffsetFromParentJoint(offset);

      OneDoFJoint inverseDynamicsJoint;

      if (joint instanceof PinJointDescription)
      {
         inverseDynamicsJoint = ScrewTools.addRevoluteJoint(joint.getName(), parentBody, offset, jointAxis);
      }
      else if (joint instanceof SliderJointDescription)
      {
         inverseDynamicsJoint = ScrewTools.addPrismaticJoint(joint.getName(), parentBody, offset, jointAxis);
      }
      else
      {
         throw new RuntimeException("Must be either Pin or Slider here!");
      }

      inverseDynamicsJoint.setEffortLimits(-joint.getEffortLimit(), joint.getEffortLimit());

      if (joint.containsLimitStops())
      {
         double[] limitStopParameters = joint.getLimitStopParameters();
         inverseDynamicsJoint.setJointLimitLower(limitStopParameters[0]);
         inverseDynamicsJoint.setJointLimitUpper(limitStopParameters[1]);
      }

      oneDoFJoints.put(joint.getName(), inverseDynamicsJoint);

      LinkDescription childLink = joint.getLink();

      checkLinkIsNeededForSensor(inverseDynamicsJoint, joint);

      double mass = childLink.getMass();
      totalMass += mass;
      Vector3d comOffset = new Vector3d(childLink.getCenterOfMassOffset());
      Matrix3d inertia = childLink.getMomentOfInertiaCopy();

      RigidBody rigidBody = ScrewTools.addRigidBody(childLink.getName(), inverseDynamicsJoint, inertia, mass,
            comOffset);
      //      System.out.println("Adding rigid body " + childLink.getName() + "; Mass: " + childLink.getMass() + "; ixx: " + childLink.getInertia().m00 + "; iyy: " + childLink.getInertia().m11
      //            + "; izz: " + childLink.getInertia().m22 + "; COM Offset: " + childLink.getCoMOffset());


      mapRigidBody(joint, inverseDynamicsJoint, rigidBody);
      addSensorDefinitions(inverseDynamicsJoint, joint);


      for(ForceSensorDescription sdfForceSensor : joint.getForceSensors())
      {
         ForceSensorDefinition forceSensorDefinition = new ForceSensorDefinition(sdfForceSensor.getName(), inverseDynamicsJoint.getSuccessor(), sdfForceSensor.getTransformToJoint());
         forceSensorDefinitions.add(forceSensorDefinition);
      }

//      for(SDFContactSensor sdfContactSensor : joint.getContactSensors())
//      {
//         ContactSensorDefinition contactSensorDefinition = new ContactSensorDefinition(sdfContactSensor.getName(), inverseDynamicsJoint.getSuccessor(),sdfContactSensor.getSensorType());
//         contactSensorDefinitions.add(contactSensorDefinition);
//      }

      for (JointDescription sdfJoint : joint.getChildrenJoints())
      {
         addJointsRecursively((OneDoFJointDescription) sdfJoint, rigidBody);
      }
   }

   protected void mapRigidBody(JointDescription joint, OneDoFJoint inverseDynamicsJoint, RigidBody rigidBody)
   {
      if (rigidBody.getName().equals(sdfJointNameMap.getChestName()))
      {
         chest = rigidBody;
      }

      if (rigidBody.getName().equals(sdfJointNameMap.getHeadName()))
      {
         head = rigidBody;
      }

      Set<String> lastSimulatedJoints = sdfJointNameMap.getLastSimulatedJoints();
      if(lastSimulatedJoints != null && lastSimulatedJoints.contains(inverseDynamicsJoint.getName()))
      {
         Enum<?> endEffectorRobotSegment = sdfJointNameMap.getEndEffectorsRobotSegment(inverseDynamicsJoint.getName());
         endEffectors.put(endEffectorRobotSegment, rigidBody);
      }

      JointRole jointRole = sdfJointNameMap.getJointRole(joint.getName());
      if(jointRole != null)
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
   public void getOneDoFJointsFromRootToHere(OneDoFJoint oneDoFJointAtEndOfChain, ArrayList<OneDoFJoint> oneDoFJointsToPack)
   {
      oneDoFJointsToPack.clear();
      InverseDynamicsJoint parent = oneDoFJointAtEndOfChain;

      while (parent != rootJoint)
      {
         if (parent instanceof OneDoFJoint)
         {
            oneDoFJointsToPack.add((OneDoFJoint) parent);
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