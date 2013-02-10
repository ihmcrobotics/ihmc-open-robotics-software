package us.ihmc.darpaRoboticsChallenge.initialSetup;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.projectM.R2Sim02.initialSetup.RobotInitialSetup;

public class SquaredUpDRCRobotInitialSetup implements RobotInitialSetup<SDFRobot>
{
   private final double groundZ;
   
   public SquaredUpDRCRobotInitialSetup()
   {
      this(0.0);
   }
   
   public SquaredUpDRCRobotInitialSetup(double groundZ)
   {
      this.groundZ = groundZ;
   }
   
   public void initializeRobot(SDFRobot robot)
   {
      
      
      // Avoid singularities at startup
      robot.getPinJoint("l_arm_ely").setQ(1.57);
      robot.getPinJoint("l_arm_elx").setQ(1.57);
      
      robot.getPinJoint("r_arm_ely").setQ(1.57);
      robot.getPinJoint("r_arm_elx").setQ(-1.57);
      
      robot.getPinJoint("l_leg_lhy").setQ(-0.4);
      robot.getPinJoint("r_leg_lhy").setQ(-0.4);
      
      robot.getPinJoint("l_leg_kny").setQ(0.8);
      robot.getPinJoint("r_leg_kny").setQ(0.8);
      
      robot.getPinJoint("l_leg_uay").setQ(-0.4);
      robot.getPinJoint("r_leg_uay").setQ(-0.4);
      
      
      Transform3D rootToWorld = new Transform3D();
      robot.getRootJointToWorldTransform(rootToWorld);
      Vector3d offset = new Vector3d();
      rootToWorld.get(offset);
      
      offset.setZ(offset.getZ() - 0.04 + groundZ); //TODO: What is this 0.04 magic number? Name it a variable.
      
      robot.setPositionInWorld(offset);
      
   }

}
