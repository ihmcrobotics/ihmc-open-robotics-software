package us.ihmc.darpaRoboticsChallenge.initialSetup;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;

public class SquaredUpDRCRobotInitialSetup implements DRCRobotInitialSetup<SDFRobot>
{
   private final double groundZ;
   private final Transform3D rootToWorld = new Transform3D();
   private final Vector3d offset = new Vector3d();

   public SquaredUpDRCRobotInitialSetup()
   {
      this(0.0);
   }

   public SquaredUpDRCRobotInitialSetup(double groundZ)
   {
      this.groundZ = groundZ;
   }

   public void initializeRobot(SDFRobot robot, DRCRobotJointMap jointMap)
   {
      setArmJointPositions(robot);
      setLegJointPositions(robot);
      setPositionInWorld(robot, jointMap);
   }
   
   protected void setPositionInWorld(SDFRobot robot, DRCRobotJointMap jointMap)
   {
      robot.update();
      robot.getRootJointToWorldTransform(rootToWorld);
      rootToWorld.get(offset);
      
      double pelvisToFoot = jointMap.getPelvisToFoot();
      offset.setZ(groundZ + pelvisToFoot);
      robot.setPositionInWorld(offset);
   }

   protected void setArmJointPositions(SDFRobot robot)
   {
      // Avoid singularities at startup

      //      robot.getOneDoFJoint(jointNames[l_arm_ely]).setQ(1.57);
//      robot.getOneDoFJoint(jointNames[l_arm_elx]).setQ(1.57);
//
//      robot.getOneDoFJoint(jointNames[r_arm_ely]).setQ(1.57);
//      robot.getOneDoFJoint(jointNames[r_arm_elx]).setQ(-1.57);
   }

   protected void setLegJointPositions(SDFRobot robot)
   {
//      try{
//         robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_hpy]).setQ(-0.4);
//         robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_hpy]).setQ(-0.4);
//   
//         robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_kny]).setQ(0.8);
//         robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_kny]).setQ(0.8);
//   
//         robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_aky]).setQ(-0.4);
//         robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_aky]).setQ(-0.4);
//      } catch(Exception e)
//      {
//         System.err.println("Hard Coded joint positions for wrong model! FIXME - SquaredUpDrcRobotInitialSetUp");
//      }
   }

   public void getOffset(Vector3d offsetToPack)
   {
      offsetToPack.set(offset);
   }

   public void setOffset(Vector3d offset)
   {
      this.offset.set(offset);
   }
}
