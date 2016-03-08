package us.ihmc.SdfLoader;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.SdfLoader.models.FullQuadrupedRobotModel;
import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.SdfLoader.partNames.JointRole;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.LimbName;
import us.ihmc.SdfLoader.partNames.NeckJointName;
import us.ihmc.SdfLoader.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class SDFFullQuadrupedRobotModel extends SDFFullRobotModel implements FullQuadrupedRobotModel
{
   private final QuadrantDependentList<RigidBody> feet = new QuadrantDependentList<RigidBody>();

   public SDFFullQuadrupedRobotModel(SDFFullQuadrupedRobotModel modelToCopy)
   {
      this( modelToCopy.rootLink, modelToCopy.sdfJointNameMap, modelToCopy.sensorLinksToTrack );
   }
   
   public SDFFullQuadrupedRobotModel(SDFLinkHolder rootLink, SDFJointNameMap sdfJointNameMap, String[] sensorLinksToTrack)
   {
      super(rootLink, sdfJointNameMap, sensorLinksToTrack);
      
      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
      }
   }
   
   @Override
   protected void mapRigidBody(SDFJointHolder joint, OneDoFJoint inverseDynamicsJoint, RigidBody rigidBody)
   {
      super.mapRigidBody(joint, inverseDynamicsJoint, rigidBody);
      
      SDFLinkHolder childLink = joint.getChildLinkHolder();
      String linkName = childLink.getName();
//      sdfJointNameMap.get
      
      
      ImmutablePair<RobotSide, LimbName> limbSideAndName = sdfJointNameMap.getLimbName(linkName);
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
   }

   @Override
   public RigidBody getFoot(RobotQuadrant robotQuadrant)
   {
      return feet.get(robotQuadrant);
   }
}
