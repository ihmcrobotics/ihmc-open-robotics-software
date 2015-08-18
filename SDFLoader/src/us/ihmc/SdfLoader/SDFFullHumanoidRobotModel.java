package us.ihmc.SdfLoader;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.humanoidRobotics.model.FullHumanoidRobotModel;
import us.ihmc.humanoidRobotics.partNames.ArmJointName;
import us.ihmc.humanoidRobotics.partNames.FingerName;
import us.ihmc.humanoidRobotics.partNames.JointRole;
import us.ihmc.humanoidRobotics.partNames.LegJointName;
import us.ihmc.humanoidRobotics.partNames.LimbName;
import us.ihmc.humanoidRobotics.partNames.NeckJointName;
import us.ihmc.humanoidRobotics.partNames.SpineJointName;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class SDFFullHumanoidRobotModel extends SDFFullRobotModel implements FullHumanoidRobotModel
{
   private SideDependentList<EnumMap<ArmJointName, OneDoFJoint>> armJointLists;
   private SideDependentList<EnumMap<LegJointName, OneDoFJoint>> legJointLists;
    
   private  SideDependentList< ArrayList<OneDoFJoint>> armJointIDsList;
   private  SideDependentList< ArrayList<OneDoFJoint>> legJointIDsList;
    
   private SideDependentList<RigidBody> feet;
   private SideDependentList<RigidBody> hands;
   
   private boolean initialized = false;
   
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> attachmentPlateFrames = new SideDependentList<>();
   
   // copy constructor
   public SDFFullHumanoidRobotModel(SDFFullHumanoidRobotModel modelToCopy)
   {
      this( modelToCopy.rootLink, modelToCopy.sdfJointNameMap, modelToCopy.sensorLinksToTrack );
   }

   public SDFFullHumanoidRobotModel(SDFLinkHolder rootLink, SDFJointNameMap sdfJointNameMap, String[] sensorLinksToTrack)
   {
      super(rootLink, sdfJointNameMap, sensorLinksToTrack);
      
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
   public ReferenceFrame getFrameAfterLegJoint(RobotSide robotSide, LegJointName legJointName)
   {
      return getLegJoint(robotSide, legJointName).getFrameAfterJoint();
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
   
   @Override
   protected void mapRigidBody(SDFJointHolder joint, OneDoFJoint inverseDynamicsJoint, RigidBody rigidBody)
   {
      initializeLists();
      
      SDFLinkHolder childLink = joint.getChildLinkHolder();
      
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
   }

   private void initializeLists()
   {
      if(!initialized)
      {
         armJointLists = SideDependentList.createListOfEnumMaps(ArmJointName.class);
         legJointLists = SideDependentList.createListOfEnumMaps(LegJointName.class);
         
         armJointIDsList =  SideDependentList.createListOfArrayLists();
         legJointIDsList =  SideDependentList.createListOfArrayLists();
         
         feet = new SideDependentList<RigidBody>();
         hands = new SideDependentList<RigidBody>();
         initialized = true;
      }
   }
   
   public ArrayList<OneDoFJoint> getArmJointIDs(RobotSide side)
   {
      return armJointIDsList.get(side);
   }
   
}
