package us.ihmc.SdfLoader;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFJointNameMap.JointRole;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Camera;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.IMU;
import us.ihmc.utilities.IMUDefinition;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.utilities.containers.ContainerTools;
import us.ihmc.robotics.humanoidRobot.model.ContactSensorDefinition;
import us.ihmc.robotics.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.robotics.humanoidRobot.model.FullRobotModel;
import us.ihmc.robotics.humanoidRobot.partNames.ArmJointName;
import us.ihmc.robotics.humanoidRobot.partNames.FingerName;
import us.ihmc.robotics.humanoidRobot.partNames.LegJointName;
import us.ihmc.robotics.humanoidRobot.partNames.LimbName;
import us.ihmc.robotics.humanoidRobot.partNames.NeckJointName;
import us.ihmc.robotics.humanoidRobot.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.humanoidRobot.partNames.SpineJointName;
import us.ihmc.robotics.geometry.InertiaTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

public class SDFFullRobotModel implements FullRobotModel
{
   private final SDFJointNameMap sdfJointNameMap;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final RigidBody elevator;
   private final SixDoFJoint rootJoint;

   //   private final ArrayList<RevoluteJoint> revoluteJoints = new ArrayList<RevoluteJoint>();
   private final LinkedHashMap<String, OneDoFJoint> oneDoFJoints = new LinkedHashMap<String, OneDoFJoint>();

   private final EnumMap<NeckJointName, OneDoFJoint> neckJoints = ContainerTools.createEnumMap(NeckJointName.class);
   private final EnumMap<SpineJointName, OneDoFJoint> spineJoints = ContainerTools.createEnumMap(SpineJointName.class);
   private final SideDependentList<EnumMap<ArmJointName, OneDoFJoint>> armJointLists = SideDependentList.createListOfEnumMaps(ArmJointName.class);
   private final SideDependentList<EnumMap<LegJointName, OneDoFJoint>> legJointLists = SideDependentList.createListOfEnumMaps(LegJointName.class);
   
   public final SideDependentList< ArrayList<OneDoFJoint>> armJointIDsList =  SideDependentList.createListOfArrayLists();
   public final SideDependentList< ArrayList<OneDoFJoint>> legJointIDsList =  SideDependentList.createListOfArrayLists();
   
   private final RigidBody pelvis;
   private RigidBody chest;
   private RigidBody head;

   private final SideDependentList<RigidBody> feet = new SideDependentList<RigidBody>();
   private final SideDependentList<RigidBody> hands = new SideDependentList<RigidBody>();
   private final ArrayList<IMUDefinition> imuDefinitions = new ArrayList<IMUDefinition>();
   private final ArrayList<ForceSensorDefinition> forceSensorDefinitions = new ArrayList<ForceSensorDefinition>();
   private final ArrayList<ContactSensorDefinition> contactSensorDefinitions = new ArrayList<ContactSensorDefinition>();
   private final HashMap<String, ReferenceFrame> cameraFrames = new HashMap<String, ReferenceFrame>();
   private final HashMap<String, ReferenceFrame> lidarBaseFrames = new HashMap<String, ReferenceFrame>();
   private final HashMap<String, RigidBodyTransform> lidarBaseToSensorTransform = new HashMap<String, RigidBodyTransform>();
   private final HashMap<String, FrameVector> lidarAxis = new HashMap<String, FrameVector>();
   private final HashMap<String, OneDoFJoint> lidarJoints = new HashMap<>();

   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> attachmentPlateFrames = new SideDependentList<>();
   private final HashMap<String, ReferenceFrame> sensorFrames = new HashMap<String, ReferenceFrame>();
   private final String[] sensorLinksToTrack;
   
   private final SDFLinkHolder rootLink;

   // copy constructor
   public SDFFullRobotModel(SDFFullRobotModel modelToCopy)
   {
      this( modelToCopy.rootLink, modelToCopy.sdfJointNameMap, modelToCopy.sensorLinksToTrack );
   }

   public SDFFullRobotModel(SDFLinkHolder rootLink, SDFJointNameMap sdfJointNameMap, String[] sensorLinksToTrack)
   {
      this.rootLink = rootLink;
      this.sdfJointNameMap = sdfJointNameMap;
      this.sensorLinksToTrack = sensorLinksToTrack;

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
      for (SDFJointHolder sdfJoint : rootLink.getChildren())
      {
         addJointsRecursively(sdfJoint, pelvis);
      }
      
      for(RobotSide robotSide : RobotSide.values)
      {
         RigidBodyTransform soleToAnkleTransform = sdfJointNameMap.getSoleToAnkleFrameTransform(robotSide);
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         ReferenceFrame soleFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(sidePrefix + "Sole", 
        		 getEndEffectorFrame(robotSide, LimbName.LEG), 
        		 soleToAnkleTransform);
         soleFrames.put(robotSide, soleFrame); 

         RigidBodyTransform handAttachmentPlaeToWristTransform = sdfJointNameMap.getHandControlFrameToWristTransform(robotSide);

         if (handAttachmentPlaeToWristTransform != null)
         {
            ReferenceFrame attachmentPlateFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(sidePrefix + "HandControlFrame",
            		getEndEffectorFrame(robotSide, LimbName.ARM), 
            		handAttachmentPlaeToWristTransform);
            attachmentPlateFrames.put(robotSide, attachmentPlateFrame);
         }
         else
         {
            attachmentPlateFrames.put(robotSide, null);
         }
      }
   }

   public String getModelName()
   {
      return sdfJointNameMap.getModelName();
   }

   private void addSensorDefinitions(InverseDynamicsJoint joint, SDFLinkHolder child)
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
                  lidarAxis.put(sensor.getName(), ((RevoluteJoint) joint).getJointAxis());
                  lidarJoints.put(sensor.getName(), (OneDoFJoint) joint);
               }
               else
               {
                  System.err.println("Not supporting lidar not connected to a revolute joint");
               }
            }

         }
      }
   }

   private void addJointsRecursively(SDFJointHolder joint, RigidBody parentBody)
   {

      Vector3d jointAxis = new Vector3d(joint.getAxisInModelFrame());
      Vector3d offset = new Vector3d(joint.getOffsetFromParentJoint());

      RigidBodyTransform visualTransform = new RigidBodyTransform();
      visualTransform.setRotation(joint.getLinkRotation());

      OneDoFJoint inverseDynamicsJoint;

      //      ReferenceFrame parentFrame;
      //      if (parentBody.isRootBody())
      //         parentFrame = parentBody.getBodyFixedFrame();
      //      else
      //         parentFrame = parentBody.getParentJoint().getFrameAfterJoint();

      //      ReferenceFrame frameBeforeJoint = ScrewTools.createOffsetFrame(parentFrame, joint.getTransformToParentJoint(), "bla");
      //      FrameVector jointAxisFrame = new FrameVector(ReferenceFrame.getWorldFrame(), jointAxis);
      //      jointAxisFrame.changeFrame(frameBeforeJoint);


      switch(joint.getType())
      {
      case REVOLUTE:
         inverseDynamicsJoint = ScrewTools.addRevoluteJoint(joint.getName(), parentBody, offset, jointAxis);
         break;
      case PRISMATIC:
         inverseDynamicsJoint = ScrewTools.addPrismaticJoint(joint.getName(), parentBody, offset, jointAxis);
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
      Vector3d comOffset = new Vector3d(childLink.getCoMOffset());
      Matrix3d inertia = InertiaTools.rotate(visualTransform, childLink.getInertia());
      visualTransform.transform(comOffset);

      RigidBody rigidBody = ScrewTools.addRigidBody(childLink.getName(), inverseDynamicsJoint, inertia, mass,
            comOffset);
      //      System.out.println("Adding rigid body " + childLink.getName() + "; Mass: " + childLink.getMass() + "; ixx: " + childLink.getInertia().m00 + "; iyy: " + childLink.getInertia().m11
      //            + "; izz: " + childLink.getInertia().m22 + "; COM Offset: " + childLink.getCoMOffset());


      if (rigidBody.getName().equals(sdfJointNameMap.getChestName()))
      {
         chest = rigidBody;
      }

      if (rigidBody.getName().equals(sdfJointNameMap.getHeadName()))
      {
         head = rigidBody;
      }

      ImmutablePair<RobotSide, LimbName> limbSideAndName = sdfJointNameMap.getLimbName(childLink.getName());
      if(limbSideAndName != null)
      {
         RobotSide limbSide = limbSideAndName.getLeft();
         LimbName limbName = limbSideAndName.getRight();
         switch (limbName)
         {
         case ARM:
            hands.put(limbSide, rigidBody);
            break;
         case LEG:
            feet.put(limbSide, rigidBody);
            break;
         }
      }

      JointRole jointRole = sdfJointNameMap.getJointRole(joint.getName());
      if(jointRole != null)
      {
         switch (jointRole)
         {
         //TODO: Should armJointLists use legJoingName.first or armJointName.first?? looks backwards
         case ARM:
            ImmutablePair<RobotSide, ArmJointName> armJointName = sdfJointNameMap.getArmJointName(joint.getName());
            armJointLists.get(armJointName.getLeft()).put(armJointName.getRight(), inverseDynamicsJoint);
            armJointIDsList.get(armJointName.getLeft()).add( inverseDynamicsJoint );
            break;
         case LEG:
            ImmutablePair<RobotSide, LegJointName> legJointName = sdfJointNameMap.getLegJointName(joint.getName());
            legJointLists.get(legJointName.getLeft()).put(legJointName.getRight(), inverseDynamicsJoint);
            legJointIDsList.get(legJointName.getLeft()).add( inverseDynamicsJoint );
            break;
         case NECK:
            NeckJointName neckJointName = sdfJointNameMap.getNeckJointName(joint.getName());
            neckJoints.put(neckJointName, inverseDynamicsJoint);
            break;
         case SPINE:
            SpineJointName spineJointName = sdfJointNameMap.getSpineJointName(joint.getName());
            spineJoints.put(spineJointName, inverseDynamicsJoint);
            break;
         }
      }

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
   
   public void setJointAngles(RobotSide side, LimbName limb, double[] q) 
   {
	   int i = 0;
		if( limb == LimbName.ARM){
			for ( OneDoFJoint jnt : armJointIDsList.get( side ) )
			{
				jnt.setQ( q[i] );
				i++;
			}
		}
		else if( limb == LimbName.LEG){
			for ( OneDoFJoint jnt : legJointIDsList.get( side ) )
			{
				jnt.setQ( q[i] );
				i++;
			}
		}
	}
   
   public void getJointAngles(RobotSide side, LimbName limb, double[] q) 
   {
      int i = 0;
      if( limb == LimbName.ARM){
         for ( OneDoFJoint jnt : armJointIDsList.get( side ) )
         {
            q[i] = jnt.getQ( );
            i++;
         }
      }
      else if( limb == LimbName.LEG){
         for ( OneDoFJoint jnt : legJointIDsList.get( side ) )
         {
            q[i] = jnt.getQ( );
            i++;
         }
      }
   }

   public void copyJointAnglesAcrossSide(RobotSide sourceSide, LimbName limb)
   {
      SideDependentList<ArrayList<OneDoFJoint>> joints;
      if (limb == LimbName.ARM)
      {
         joints = armJointIDsList;
      }
      else if (limb == LimbName.LEG)
      {
         joints = legJointIDsList;
      }
      else
      {
         return;
      }
      
      ArrayList<OneDoFJoint> sourceJoints = joints.get(sourceSide);
      ArrayList<OneDoFJoint> destJoints = joints.get(sourceSide.getOppositeSide());
      
      for(int i = 0; i < sourceJoints.size(); i++)
      {
         destJoints.get(i).setQ(sourceJoints.get(i).getQ());
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
   synchronized public void updateFrames()
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
   public ReferenceFrame getFrameAfterLegJoint(RobotSide robotSide, LegJointName legJointName)
   {
      return getLegJoint(robotSide, legJointName).getFrameAfterJoint();
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
   public OneDoFJoint getLegJoint(RobotSide robotSide, LegJointName legJointName)
   {
      return legJointLists.get(robotSide).get(legJointName);
   }

   /** {@inheritDoc} */
   @Override
   public OneDoFJoint getArmJoint(RobotSide robotSide, ArmJointName armJointName)
   {
      return armJointLists.get(robotSide).get(armJointName);
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
   public RigidBody getFoot(RobotSide robotSide)
   {
      return getEndEffector(robotSide, LimbName.LEG);
   }

   /** {@inheritDoc} */
   @Override
   public RigidBody getHand(RobotSide robotSide)
   {
      return getEndEffector(robotSide, LimbName.ARM);
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
   public RigidBody getEndEffector(RobotSide robotSide, LimbName limbName)
   {
      switch (limbName)
      {
      case ARM:
         return hands.get(robotSide);
      case LEG:
         return feet.get(robotSide);
      default:
         throw new RuntimeException("Unkown end effector");
      }
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getEndEffectorFrame(RobotSide robotSide, LimbName limbName)
   {
      return getEndEffector(robotSide, limbName).getParentJoint().getFrameAfterJoint();
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint getStaticWristToFingerOffset(RobotSide robotSide, FingerName fingerName)
   {
      // TODO Auto-generated method stub
      return null;
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

   public FrameVector getLidarJointAxis(String name)
   {
      return lidarAxis.get(name);
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getSoleFrame(RobotSide robotSide)
   {
      return soleFrames.get(robotSide);
   }

   public SideDependentList<ReferenceFrame> getSoleFrames()
   {
      return soleFrames;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getHandControlFrame(RobotSide robotSide)
   {
      return attachmentPlateFrames.get(robotSide);
   }

   public ReferenceFrame getHeadBaseFrame()
   {
      return head.getParentJoint().getFrameAfterJoint();
   }
   
   private void checkLinkIsNeededForSensor(InverseDynamicsJoint joint, SDFLinkHolder link)
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

   public void copyAllJointsButKeepOneFootFixed( OneDoFJoint[] jointsToCopyFrom, RobotSide sideOfSole )
   {
      HashMap<String, Double> newJointAngles = new HashMap<String, Double>();     
      int numJoints = getOneDoFJoints().length;

      for (int i=0; i<numJoints; i++)  {
         newJointAngles.put( jointsToCopyFrom[i].getName() ,  jointsToCopyFrom[i].getQ() ); 
      }
      updateJointsAngleButKeepOneFootFixed( newJointAngles, sideOfSole);
   }
   
   public void copyAllJointsButKeepOneFootFixed( ArrayList<OneDoFJoint> jointsToCopyFrom, RobotSide sideOfSole )
   {
      HashMap<String, Double> newJointAngles = new HashMap<String, Double>();     

      for (int i=0; i<jointsToCopyFrom.size(); i++)  {
         newJointAngles.put( jointsToCopyFrom.get(i).getName() ,  jointsToCopyFrom.get(i).getQ() ); 
      }
      updateJointsAngleButKeepOneFootFixed( newJointAngles, sideOfSole);
   }

   public void updateJointsAngleButKeepOneFootFixed( Map<String, Double> jointsAngles, RobotSide sideOfSole )
   {
      updateJointsStateButKeepOneFootFixed( jointsAngles,null, sideOfSole);
   }
   
   public void updateJointsStateButKeepOneFootFixed( Map<String, Double> jointsAngles, Map<String, Double> jointsVelocities, RobotSide sideOfSole )
   {   
      // next line of code must be executed BEFORE modifying the model
      RigidBodyTransform worldToFoot  = getSoleFrame(sideOfSole).getTransformToWorldFrame();
      
      if( jointsAngles != null)
      {
         for ( Map.Entry<String, Double> entry: jointsAngles.entrySet() )
         {
             getOneDoFJointByName( entry.getKey() ).setQ( entry.getValue() );
         }
      }
      
      if( jointsVelocities != null)
      {
         for ( Map.Entry<String, Double> entry: jointsVelocities.entrySet() )
         {
             getOneDoFJointByName( entry.getKey() ).setQd( entry.getValue() );
         }
      }
      
      this.updateFrames();
      
      ReferenceFrame footFrame   = getSoleFrame(sideOfSole);
      ReferenceFrame pelvisFrame = getRootJoint().getFrameAfterJoint();
      
      RigidBodyTransform footToPelvis = pelvisFrame.getTransformToDesiredFrame( footFrame );

      RigidBodyTransform worldToPelvis = new RigidBodyTransform();
      worldToPelvis.multiply( worldToFoot, footToPelvis );

      getRootJoint().setPositionAndRotation(worldToPelvis);
      
      this.updateFrames();

   }
   
}
