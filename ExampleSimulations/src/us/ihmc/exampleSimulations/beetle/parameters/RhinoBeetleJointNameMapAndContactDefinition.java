package us.ihmc.exampleSimulations.beetle.parameters;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.ContactPointDefinitionHolder;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSextant;
import us.ihmc.robotics.robotSide.SegmentDependentList;

public class RhinoBeetleJointNameMapAndContactDefinition implements JointNameMap, ContactPointDefinitionHolder
{
   private final String modelName = RhinoBeetleSDFParameters.SDF_MODEL_NAME;
   private final String rootJoint = "BODY";
   
   private final List<ImmutablePair<String, Vector3D>> jointNameGroundContactPointMap = new ArrayList<ImmutablePair<String, Vector3D>>();
   private final SegmentDependentList<RobotSextant, HashMap<LegJointName, RhinoBeetleJointName>> jointIdsBySextant = new SegmentDependentList<>(RobotSextant.class);
   private final HashSet<String> lastSimulatedJoints = new HashSet<>();
   private final String[] jointNamesBeforeFeet = new String[6];

   public RhinoBeetleJointNameMapAndContactDefinition()
   {
      for(RhinoBeetleJointName jointId : RhinoBeetleJointName.values)
      {
         RobotSextant sextant = jointId.getSextant();
         LegJointName legJointName = jointId.getLegJointName();
         
         //map legs
         if(sextant != null)
         {
            //add to maps for each leg
            HashMap<LegJointName, RhinoBeetleJointName> legJointIds = jointIdsBySextant.get(sextant);
            if(legJointIds == null)
            {
               legJointIds = new HashMap<>();
               jointIdsBySextant.set(sextant, legJointIds);
            }
            legJointIds.put(legJointName, jointId);
            
            if(legJointName == LegJointName.KNEE_PITCH)
            {
               //lastSimulated Joints
               lastSimulatedJoints.add(jointId.toString());
               
               //contact points
               Vector3D contactPoint = RhinoBeetlePhysicalProperties.getOffsetFromJointBeforeFootToSoleAlignedWithWorld(sextant);
               jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3D>(jointId.toString(), contactPoint));
            }
         }
         else
         {
            //gaster_pitch
         }
      }
      lastSimulatedJoints.toArray(jointNamesBeforeFeet);
   }
   
   @Override
   public String getModelName()
   {
      return modelName ;
   }

   @Override
   public JointRole getJointRole(String jointName)
   {
      return JointRole.LEG;
   }

   @Override
   public String getPelvisName()
   {
      return rootJoint;
   }

   @Override
   public String getUnsanitizedRootJointInSdf()
   {
      return rootJoint;
   }

   @Override
   public List<ImmutablePair<String, Vector3D>> getJointNameGroundContactPointMap()
   {
      return jointNameGroundContactPointMap;
   }

   @Override
   public boolean isTorqueVelocityLimitsEnabled()
   {
      return false;
   }
   
   public String getJointNameBeforeFoot(RobotSextant robotSextant)
   {
      return jointIdsBySextant.get(robotSextant).get(LegJointName.KNEE_PITCH).getName();
   }
   
   @Override
   public Set<String> getLastSimulatedJoints()
   {
      return lastSimulatedJoints;
   }

   @Override
   public String[] getJointNamesBeforeFeet()
   {
      return jointNamesBeforeFeet;
   }

   @Override
   public Enum<?>[] getRobotSegments()
   {
      return RobotSextant.values;
   }

   @Override
   public LegJointName[] getLegJointNames()
   {
      return null;
   }

   @Override
   public ArmJointName[] getArmJointNames()
   {
      return null;
   }

   @Override
   public SpineJointName[] getSpineJointNames()
   {
      return null;
   }

   @Override
   public NeckJointName[] getNeckJointNames()
   {
      return null;
   }

   @Override
   @Deprecated
   public NeckJointName getNeckJointName(String jointName)
   {
      return null;
   }

   @Override
   @Deprecated
   public SpineJointName getSpineJointName(String jointName)
   {
      return null;
   }
   
   @Override
   @Deprecated
   public String getChestName()
   {
      return null;
   }

   @Override
   @Deprecated
   public String getHeadName()
   {
      return null;
   }

   @Override
   public Enum<?> getEndEffectorsRobotSegment(String joineNameBeforeEndEffector)
   {
      return RhinoBeetleJointName.valueOf(joineNameBeforeEndEffector).getSextant();
   }

   public String getLegJointName(RobotSextant robotSextant, LegJointName legJointName)
   {
      return jointIdsBySextant.get(robotSextant).get(legJointName).toString();
   }
}
