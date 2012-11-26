package us.ihmc.darpaRoboticsChallenge.initialSetup;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFRobot;

public class SquaredUpDRCRobotInitialSetup implements DRCRobotInitialSetup
{

   public void initializeRobot(SDFRobot robot)
   {
//      robot.getJoint("l_arm_shx").setQ(-1.4);
//      robot.getJoint("r_arm_shx").setQ(1.4);
      
      robot.getJoint("l_leg_lhy").setQ(-0.4);
      robot.getJoint("r_leg_lhy").setQ(-0.4);
      
      robot.getJoint("l_leg_kny").setQ(0.8);
      robot.getJoint("r_leg_kny").setQ(0.8);
      
      robot.getJoint("l_leg_uay").setQ(-0.4);
      robot.getJoint("r_leg_uay").setQ(-0.4);
      
      
      Transform3D rootToWorld = new Transform3D();
      robot.getRootJointToWorldTransform(rootToWorld);
      Vector3d offset = new Vector3d();
      rootToWorld.get(offset);
      
      offset.setZ(offset.getZ() - 0.04);
      
      robot.setPositionInWorld(offset);
      
   }

}
