package us.ihmc.avatar.initialSetup;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class SquaredUpDRCRobotInitialSetup implements DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>
{
   private double groundZ;
   private final RigidBodyTransform rootToWorld = new RigidBodyTransform();
   private final Vector3D offset = new Vector3D();

   public SquaredUpDRCRobotInitialSetup()
   {
      this(0.0);
   }

   public SquaredUpDRCRobotInitialSetup(double groundZ)
   {
      this.groundZ = groundZ;
   }

   public void initializeRobot(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      setArmJointPositions(robot);
      setLegJointPositions(robot);
      setPositionInWorld(robot);
   }
   
   protected void setPositionInWorld(HumanoidFloatingRootJointRobot robot)
   {
      robot.update();
      robot.getRootJointToWorldTransform(rootToWorld);
      rootToWorld.getTranslation(offset);
      Vector3D positionInWorld = new Vector3D();
      
      rootToWorld.getTranslation(positionInWorld);
      
      GroundContactPoint gc1 = robot.getFootGroundContactPoints(RobotSide.LEFT).get(0);
      double pelvisToFoot = positionInWorld.getZ() - gc1.getPositionPoint().getZ();
      
      offset.setZ(groundZ + pelvisToFoot);
      robot.setPositionInWorld(offset);
   }

   protected void setArmJointPositions(FloatingRootJointRobot robot)
   {
      // Avoid singularities at startup

      //      robot.getOneDoFJoint(jointNames[l_arm_ely]).setQ(1.57);
//      robot.getOneDoFJoint(jointNames[l_arm_elx]).setQ(1.57);
//
//      robot.getOneDoFJoint(jointNames[r_arm_ely]).setQ(1.57);
//      robot.getOneDoFJoint(jointNames[r_arm_elx]).setQ(-1.57);
   }

   protected void setLegJointPositions(FloatingRootJointRobot robot)
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

   public void getOffset(Vector3D offsetToPack)
   {
      offsetToPack.set(offset);
   }

   public void setOffset(Vector3D offset)
   {
      this.offset.set(offset);
   }

   public void setInitialYaw(double yaw)
   {
   }

   public void setInitialGroundHeight(double groundHeight)
   {
      groundZ = groundHeight;
   }

   @Override
   public double getInitialYaw()
   {
      return 0;
   }

   @Override
   public double getInitialGroundHeight()
   {
      return groundZ;
   }
}
