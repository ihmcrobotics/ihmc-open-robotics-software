package us.ihmc.darpaRoboticsChallenge.initialSetup;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.projectM.R2Sim02.initialSetup.RobotInitialSetup;

public class SquaredUpDRCRobotInitialSetup implements RobotInitialSetup<SDFRobot>
{
   private final double groundZ;
   private Transform3D rootToWorld = new Transform3D();
   protected Vector3d offset = new Vector3d();
   
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
      robot.getOneDoFJoint("l_arm_ely").setQ(1.57);
      robot.getOneDoFJoint("l_arm_elx").setQ(1.57);
      
      robot.getOneDoFJoint("r_arm_ely").setQ(1.57);
      robot.getOneDoFJoint("r_arm_elx").setQ(-1.57);
      
      robot.getOneDoFJoint("l_leg_lhy").setQ(-0.4);
      robot.getOneDoFJoint("r_leg_lhy").setQ(-0.4);
      
      robot.getOneDoFJoint("l_leg_kny").setQ(0.8);
      robot.getOneDoFJoint("r_leg_kny").setQ(0.8);
      
      robot.getOneDoFJoint("l_leg_uay").setQ(-0.4);
      robot.getOneDoFJoint("r_leg_uay").setQ(-0.4);
      
      
      robot.getRootJointToWorldTransform(rootToWorld);
      
      rootToWorld.get(offset);
      offset.setZ(offset.getZ() - 0.04 + groundZ); //TODO: What is this 0.04 magic number? Name it a variable.
      offset.add(robot.getPositionInWorld());
      robot.setPositionInWorld(offset);
      
   }

}
