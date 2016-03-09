package us.ihmc.SdfLoader;

import us.ihmc.SdfLoader.models.FullQuadrupedRobotModel;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class SDFFullQuadrupedRobotModel extends SDFFullRobotModel implements FullQuadrupedRobotModel
{
   private QuadrantDependentList<RigidBody> feet;
   private SDFQuadrupedJointNameMap quadrupedJointNameMap;

   public SDFFullQuadrupedRobotModel(SDFFullQuadrupedRobotModel modelToCopy)
   {
      this(modelToCopy.rootLink, modelToCopy.quadrupedJointNameMap, modelToCopy.sensorLinksToTrack );
   }
   
   public SDFFullQuadrupedRobotModel(SDFLinkHolder rootLink, SDFQuadrupedJointNameMap sdfJointNameMap, String[] sensorLinksToTrack)
   {
      super(rootLink, sdfJointNameMap, sensorLinksToTrack);
   }
   
   @Override
   protected void mapRigidBody(SDFJointHolder joint, OneDoFJoint inverseDynamicsJoint, RigidBody rigidBody)
   {
      if(feet == null)
      {
         feet = new QuadrantDependentList<RigidBody>();
      }
      
      super.mapRigidBody(joint, inverseDynamicsJoint, rigidBody);
      
      SDFQuadrupedJointNameMap jointMap = (SDFQuadrupedJointNameMap) sdfJointNameMap;
      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String jointBeforeFootName = jointMap.getJointBeforeFootName(robotQuadrant);
         
         if(jointBeforeFootName.equals(joint.getName()))
         {
            feet.set(robotQuadrant, rigidBody);
         }
      }
   }

   @Override
   public RigidBody getFoot(RobotQuadrant robotQuadrant)
   {
      return feet.get(robotQuadrant);
   }
}
