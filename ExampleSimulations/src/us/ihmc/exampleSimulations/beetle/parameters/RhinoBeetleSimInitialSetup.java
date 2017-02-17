package us.ihmc.exampleSimulations.beetle.parameters;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSextant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;

public class RhinoBeetleSimInitialSetup
{
   private double groundZ;
   private double initialYaw;
   private final RigidBodyTransform rootToWorld = new RigidBodyTransform();
   private final Vector3D positionInWorld = new Vector3D();
   private final Quaternion rotation = new Quaternion();
   private boolean robotInitialized = false;

   public RhinoBeetleSimInitialSetup()
   {
      this(0.0, 0.0);
   }

   public RhinoBeetleSimInitialSetup(double groundZ, double initialYaw)
   {
      this.groundZ = groundZ;
      this.initialYaw = initialYaw;
   }

   public void initializeRobot(FloatingRootJointRobot robot, RhinoBeetleJointNameMap jointMap)
   {
      if (!robotInitialized)
      {
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(RobotSextant.FRONT_LEFT, LegJointName.HIP_YAW)).setQ(-0.7);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(RobotSextant.FRONT_RIGHT, LegJointName.HIP_YAW)).setQ(0.7);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(RobotSextant.HIND_LEFT, LegJointName.HIP_YAW)).setQ(0.7);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(RobotSextant.HIND_RIGHT, LegJointName.HIP_YAW)).setQ(-0.7);
         
         for (RobotSextant robotSextant : RobotSextant.values)
         {
            RobotSide robotSide = robotSextant.getRobotSide();
            robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSextant, LegJointName.HIP_PITCH)).setQ(robotSide.negateIfLeftSide(-0.7));
            robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSextant, LegJointName.KNEE_PITCH)).setQ(robotSide.negateIfLeftSide(1.8));
         }

         robot.update();
         robot.getRootJointToWorldTransform(rootToWorld);
         rootToWorld.get(rotation, positionInWorld);

         positionInWorld.setZ(groundZ + 0.10);

         robot.setPositionInWorld(positionInWorld);

         FrameOrientation frameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), rotation);
         double[] yawPitchRoll = frameOrientation.getYawPitchRoll();
         yawPitchRoll[0] = initialYaw;
         frameOrientation.setYawPitchRoll(yawPitchRoll);

         robot.setOrientation(frameOrientation.getQuaternionCopy());
         robot.update();
         robotInitialized = true;
      }
   }
}
