package us.ihmc.exampleSimulations.beetle.parameters;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSextant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.QuaternionDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;

public class RhinoBeetleSimInitialSetup
{
   private final double groundZ;
   private final double initialYaw;

   public RhinoBeetleSimInitialSetup()
   {
      this(0.0, 0.0);
   }

   public RhinoBeetleSimInitialSetup(double groundZ, double initialYaw)
   {
      this.groundZ = groundZ;
      this.initialYaw = initialYaw;
   }

   public void initializeRobotDefinition(RobotDefinition robotDefinition, RhinoBeetleJointNameMapAndContactDefinition jointMap)
   {
      robotDefinition.getOneDoFJointDefinition(jointMap.getLegJointName(RobotSextant.FRONT_LEFT, LegJointName.HIP_YAW)).setInitialJointState(-0.7);
      robotDefinition.getOneDoFJointDefinition(jointMap.getLegJointName(RobotSextant.FRONT_RIGHT, LegJointName.HIP_YAW)).setInitialJointState(0.7);
      robotDefinition.getOneDoFJointDefinition(jointMap.getLegJointName(RobotSextant.HIND_LEFT, LegJointName.HIP_YAW)).setInitialJointState(0.7);
      robotDefinition.getOneDoFJointDefinition(jointMap.getLegJointName(RobotSextant.HIND_RIGHT, LegJointName.HIP_YAW)).setInitialJointState(-0.7);

      for (RobotSextant robotSextant : RobotSextant.values)
      {
         RobotSide robotSide = robotSextant.getRobotSide();
         robotDefinition.getOneDoFJointDefinition(jointMap.getLegJointName(robotSextant, LegJointName.HIP_PITCH))
                        .setInitialJointState(robotSide.negateIfLeftSide(-0.7));
         robotDefinition.getOneDoFJointDefinition(jointMap.getLegJointName(robotSextant, LegJointName.KNEE_PITCH))
                        .setInitialJointState(robotSide.negateIfLeftSide(1.8));
      }

      SixDoFJointDefinition rootJoint = robotDefinition.getFloatingRootJointDefinition();
      QuaternionDefinition orientation = new QuaternionDefinition();
      orientation.set(new YawPitchRoll(initialYaw, 0.0, 0.0));

      SixDoFJointState initialState = new SixDoFJointState();
      initialState.setConfiguration(orientation, new Point3D(0.0, 0.0, groundZ + 0.10));
      rootJoint.setInitialJointState(initialState);
   }
}
