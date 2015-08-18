package us.ihmc.SdfLoader;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Set;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJointHolder;
import us.ihmc.tools.FormattingTools;




public class SDFHumanoidRobot extends SDFRobot
{
   
   private final SideDependentList<String> jointsBeforeFeet = new SideDependentList<String>();

   private final SideDependentList<ArrayList<GroundContactPoint>> footGroundContactPoints = new SideDependentList<ArrayList<GroundContactPoint>>();

   private final SideDependentList<ArrayList<GroundContactPoint>> handGroundContactPoints = new SideDependentList<ArrayList<GroundContactPoint>>();
   
   public SDFHumanoidRobot(GeneralizedSDFRobotModel generalizedSDFRobotModel, SDFJointNameMap sdfJointNameMap, boolean useCollisionMeshes)
   {
      this(generalizedSDFRobotModel, sdfJointNameMap, useCollisionMeshes, true, true);
   }

   public SDFHumanoidRobot(GeneralizedSDFRobotModel generalizedSDFRobotModel, SDFJointNameMap sdfJointNameMap, boolean useCollisionMeshes,
         boolean enableTorqueVelocityLimits, boolean enableDamping)
   {
      super(generalizedSDFRobotModel, sdfJointNameMap, useCollisionMeshes, enableTorqueVelocityLimits, enableDamping);
      
      for (RobotSide robotSide : RobotSide.values)
      {
         footGroundContactPoints.put(robotSide, new ArrayList<GroundContactPoint>());
         handGroundContactPoints.put(robotSide, new ArrayList<GroundContactPoint>());
         if(sdfJointNameMap != null)
         {
            jointsBeforeFeet.put(robotSide, sdfJointNameMap.getJointBeforeFootName(robotSide));            
         }
      }
      
      for(Joint joint : getOneDoFJoints())
      {
         for(RobotSide robotSide : RobotSide.values)
         {
            ArrayList<GroundContactPoint> contactPointsForJoint = jointToGroundContactPointsMap.get(joint);
            
            if(contactPointsForJoint != null)
            {
               String jointName = joint.getName();
               if (jointName.equals(sdfJointNameMap.getJointBeforeFootName(robotSide)))
               {
                  footGroundContactPoints.get(robotSide).addAll(contactPointsForJoint);
               }
               else if (jointName.equals(sdfJointNameMap.getJointBeforeHandName(robotSide)))
               {
                  handGroundContactPoints.get(robotSide).addAll(contactPointsForJoint);
               }
            }
         }
      }
   }

   public List<GroundContactPoint> getFootGroundContactPoints(RobotSide robotSide)
   {
      return footGroundContactPoints.get(robotSide);
   }

   public List<GroundContactPoint> getHandGroundContactPoints(RobotSide robotSide)
   {
      return handGroundContactPoints.get(robotSide);
   }
   
   public SideDependentList<String> getJointNamesBeforeFeet()
   {
      return jointsBeforeFeet;
   }

}
