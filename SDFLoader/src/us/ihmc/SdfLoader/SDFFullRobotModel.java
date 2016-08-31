package us.ihmc.SdfLoader;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.SdfLoader.partNames.JointRole;
import us.ihmc.SdfLoader.partNames.NeckJointName;
import us.ihmc.SdfLoader.partNames.RobotSpecificJointNames;
import us.ihmc.SdfLoader.partNames.SpineJointName;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Camera;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.IMU;
import us.ihmc.robotics.geometry.InertiaTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.sensors.ContactSensorDefinition;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.tools.containers.ContainerTools;

public class SDFFullRobotModel implements FullRobotModel
{

   protected final SDFJointNameMap sdfJointNameMap;
   protected final EnumMap<NeckJointName, OneDoFJoint> neckJoints = ContainerTools.createEnumMap(NeckJointName.class);
   protected final EnumMap<SpineJointName, OneDoFJoint> spineJoints = ContainerTools.createEnumMap(SpineJointName.class);
   protected final String[] sensorLinksToTrack;
   protected final SDFLinkHolder rootLink;
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
   
   public SDFFullRobotModel(SDFLinkHolder rootLink, SDFJointNameMap sdfJointNameMap, String[] sensorLinksToTrack)
   {
      this(rootLink, sdfJointNameMap, sensorLinksToTrack, false);
   }
   
   public SDFFullRobotModel(SDFLinkHolder rootLink, SDFJointNameMap sdfJointNameMap, String[] sensorLinksToTrack, boolean makeReferenceFramesAlignWithTheJoints)
   {
      super();
      this.rootLink = rootLink;
      this.sdfJointNameMap = sdfJointNameMap;
      this.sensorLinksToTrack = sensorLinksToTrack;
      this.alignReferenceFramesWithJoints = makeReferenceFramesAlignWithTheJoints;
      
      /*
       * Create root object
       */
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      elevator = new RigidBody("elevator", elevatorFrame);
      rootJoint = new SixDoFJoint(rootLink.getName(), elevator, elevatorFrame);
      if (!rootLink.getName().equals(sdfJointNameMap.getPelvisName()))
      {
         throw new RuntimeException("Pelvis joint is assumed to be the root joint");
      }

      //      System.out.println("Adding rigid body " + rootLink.getName() + "; Mass: " + rootLink.getMass() + "; ixx: " + rootLink.getInertia().m00 + "; iyy: " + rootLink.getInertia().m11
      //            + "; izz: " + rootLink.getInertia().m22 + "; COM Offset: " + rootLink.getCoMOffset());
      pelvis = ScrewTools.addRigidBody(rootLink.getName(), rootJoint, rootLink.getInertia(), rootLink.getMass(), rootLink.getCoMOffset());

      checkLinkIsNeededForSensor(rootJoint, rootLink);
      addSensorDefinitions(rootJoint, rootLink);
      
      if (pelvis.getName().equals(sdfJointNameMap.getHeadName()))
      {
         head = pelvis;
      }

      totalMass = rootLink.getMass();
      for (SDFJointHolder sdfJoint : rootLink.getChildren())
      {
         addJointsRecursively(sdfJoint, pelvis);
      }
   }

   public RigidBody getEndEffector(Enum<?> segmentEnum)
   {
      return endEffectors.get(segmentEnum);
   }
   
   public String getModelName()
   {
      return sdfJointNameMap.getModelName();
   }

   protected void addSensorDefinitions(InverseDynamicsJoint joint, SDFLinkHolder child)
   {
      if (child.getSensors() != null)
      {
         for (SDFSensor sensor : child.getSensors())
         {
            // The linkRotation transform is to make sure that the linkToSensor is in a zUpFrame.
            RigidBodyTransform linkRotation = new RigidBodyTransform(child.getTransformFromModelReferenceFrame());
            linkRotation.setTranslation(0.0, 0.0, 0.0);
            RigidBodyTransform linkToSensorInZUp = new RigidBodyTransform();
            linkToSensorInZUp.multiply(linkRotation, SDFConversionsHelper.poseToTransform(sensor.getPose()));
            if ("imu".equals(sensor.getType()))
            {
               final IMU imu = sensor.getImu();
   
               if (imu != null)
               {
                  IMUDefinition imuDefinition = new IMUDefinition(child.getName() + "_" + sensor.getName(), joint.getSuccessor(), linkToSensorInZUp);
   
                  imuDefinitions.add(imuDefinition);
               }
               else
               {
                  System.err.println("JAXB loader: No imu section defined for imu sensor " + sensor.getName() + ", ignoring sensor.");
               }
            }
            else if("multicamera".equals(sensor.getType()) || "camera".equals(sensor.getType()))
            {
               List<Camera> cameras = sensor.getCamera();
               if(cameras != null)
               {
                  ReferenceFrame sensorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent(sensor.getName(), joint.getFrameAfterJoint(), linkToSensorInZUp);
                  for(Camera camera : cameras)
                  {
                     RigidBodyTransform cameraTransform = SDFConversionsHelper.poseToTransform(camera.getPose()); 
   
                     ReferenceFrame cameraFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent(sensor.getName() + "_" + camera.getName(), sensorFrame, cameraTransform);
                     cameraFrames.put(cameraFrame.getName(), cameraFrame);
                  }
               }
               else
               {
                  System.err.println("JAXB loader: No camera section defined for camera sensor " + sensor.getName() + ", ignoring sensor.");
               }
   
            }
            else if("ray".equals(sensor.getType()) || "gpu_ray".equals(sensor.getType()))
            {
               if(joint instanceof RevoluteJoint)
               {
                  ReferenceFrame lidarFrame = joint.getFrameBeforeJoint();
                  lidarBaseFrames.put(sensor.getName(), lidarFrame);
                  lidarBaseToSensorTransform.put(sensor.getName(), linkToSensorInZUp);
                  lidarJoints.put(sensor.getName(), joint);
               }
               else if (sensor.getPose() != null)
               {
                  ReferenceFrame lidarFrame = ReferenceFrame.constructFrameWithUnchangingTransformFromParent(sensor.getName(), joint.getFrameAfterJoint(), linkToSensorInZUp);
                  lidarBaseFrames.put(sensor.getName(), lidarFrame);
                  lidarBaseToSensorTransform.put(sensor.getName(), new RigidBodyTransform());
                  lidarJoints.put(sensor.getName(), joint);
               }
               else
               {
                  System.err.println("Not supporting lidar not connected to a revolute joint or without a <pose> tag");
               }
            }
   
         }
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

   public Map<String, OneDoFJoint> getOneDoFJointsAsMap()
   {
      return Collections.unmodifiableMap(oneDoFJoints);
   }

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

   public ReferenceFrame getCameraFrame(String name)
   {
      return cameraFrames.get(name);
   }

   public ReferenceFrame getLidarBaseFrame(String name)
   {
   //      for(String st : lidarBaseFrames.keySet())
   //      {
   //         System.out.println(st);
   //      }
         return lidarBaseFrames.get(name);
      }

   public RigidBodyTransform getLidarBaseToSensorTransform(String name)
   {
      return lidarBaseToSensorTransform.get(name);
   }

   public ReferenceFrame getHeadBaseFrame()
   {
      return head.getParentJoint().getFrameAfterJoint();
   }

   protected void checkLinkIsNeededForSensor(InverseDynamicsJoint joint, SDFLinkHolder link)
   {
      for(int i = 0; i < sensorLinksToTrack.length; i++)
      {
         if(sensorLinksToTrack[i].equalsIgnoreCase(link.getName()));
         {  
            sensorFrames.put(link.getName(),joint.getFrameAfterJoint());
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
   
   protected void addJointsRecursively(SDFJointHolder joint, RigidBody parentBody)
   {
   
      Vector3d jointAxis = new Vector3d(joint.getAxisInModelFrame());
      Vector3d offset = new Vector3d(joint.getOffsetFromParentJoint());
      RigidBodyTransform transformToParentJoint = joint.getTransformToParentJoint();
   
      RigidBodyTransform visualTransform = new RigidBodyTransform();
      visualTransform.setRotation(joint.getLinkRotation());
   
      OneDoFJoint inverseDynamicsJoint;
   
      switch(joint.getType())
      {
      case REVOLUTE:
         if(alignReferenceFramesWithJoints)
         {
            inverseDynamicsJoint = ScrewTools.addRevoluteJoint(joint.getName(), parentBody, transformToParentJoint, jointAxis);
         }
         else
         {
            inverseDynamicsJoint = ScrewTools.addRevoluteJoint(joint.getName(), parentBody, offset, jointAxis);
         }
         break;
      case PRISMATIC:
         
         if(alignReferenceFramesWithJoints)
         {
            inverseDynamicsJoint = ScrewTools.addPrismaticJoint(joint.getName(), parentBody, transformToParentJoint, jointAxis);
         }
         else
         {
            inverseDynamicsJoint = ScrewTools.addPrismaticJoint(joint.getName(), parentBody, offset, jointAxis);
         }
         break;
      default:
         throw new RuntimeException("Joint type not implemented: " + joint.getType());
      }
   
      inverseDynamicsJoint.setEffortLimit(joint.getEffortLimit());
      inverseDynamicsJoint.setJointLimitLower(joint.getLowerLimit());
      inverseDynamicsJoint.setJointLimitUpper(joint.getUpperLimit());
   
      oneDoFJoints.put(joint.getName(), inverseDynamicsJoint);
   
      SDFLinkHolder childLink = joint.getChildLinkHolder();
   
      checkLinkIsNeededForSensor(inverseDynamicsJoint, childLink);
      
      double mass = childLink.getMass();
      totalMass += mass;
      Vector3d comOffset = new Vector3d(childLink.getCoMOffset());
      Matrix3d inertia = InertiaTools.rotate(visualTransform, childLink.getInertia());
      visualTransform.transform(comOffset);
   
      RigidBody rigidBody = ScrewTools.addRigidBody(childLink.getName(), inverseDynamicsJoint, inertia, mass,
            comOffset);
      //      System.out.println("Adding rigid body " + childLink.getName() + "; Mass: " + childLink.getMass() + "; ixx: " + childLink.getInertia().m00 + "; iyy: " + childLink.getInertia().m11
      //            + "; izz: " + childLink.getInertia().m22 + "; COM Offset: " + childLink.getCoMOffset());
   
   
      mapRigidBody(joint, inverseDynamicsJoint, rigidBody);
      addSensorDefinitions(inverseDynamicsJoint, childLink);
   
   
      for(SDFForceSensor sdfForceSensor : joint.getForceSensors())
      {
         ForceSensorDefinition forceSensorDefinition = new ForceSensorDefinition(sdfForceSensor.getName(), inverseDynamicsJoint.getSuccessor(), sdfForceSensor.getTransform());
         forceSensorDefinitions.add(forceSensorDefinition);
      }
      
      for(SDFContactSensor sdfContactSensor : joint.getContactSensors())
      {
         ContactSensorDefinition contactSensorDefinition = new ContactSensorDefinition(sdfContactSensor.getName(), inverseDynamicsJoint.getSuccessor(),sdfContactSensor.getSensorType());
         contactSensorDefinitions.add(contactSensorDefinition);
      }
   
      for (SDFJointHolder sdfJoint : childLink.getChildren())
      {
         addJointsRecursively(sdfJoint, rigidBody);
      }
   }

   protected void mapRigidBody(SDFJointHolder joint, OneDoFJoint inverseDynamicsJoint, RigidBody rigidBody)
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

   public double getTotalMass()
   {
      return totalMass;
   }

}