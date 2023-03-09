package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.*;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.geometry.*;
import us.ihmc.scs2.definition.robot.*;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class LIPMWalkerRobotDefinition extends RobotDefinition
{
   private static final String name = "LIPMWalker";

   // OFFSETS
   public static final double HIP_OFFSET_FROM_BODY = 0.4;
   public static final double ANKLE_JOINT_OFF = 0.0254;

   // Foot
   public static final double FOOT_FORWARD = 0.1524;
   public static final double FOOT_BACK = 0.051;
   public static final double FOOT_LENGTH = FOOT_BACK + FOOT_FORWARD;
   public static final double FOOT_WIDTH_SCALE_FACTOR = 1.5;  // This scale is used to make the feet wider so it is easier to keep the robot balance in single support.
   public static final double FOOT_WIDTH = FOOT_WIDTH_SCALE_FACTOR * 0.0889;
   public static final double FOOT_HEIGHT = 0.051;
   public static final double FOOT_MASS = 0.414988;
   public static final Vector3DReadOnly FOOT_COM = new Vector3D(0.050700, 0.0, -0.025500);
   public static final Vector3DReadOnly FOOT_I = new Vector3D(0.00036326, 0.00152067, 0.00170404);

   // Retinaculum
   public static final double RETINACULUM_MASS = 0.250041;
   public static final Vector3DReadOnly RETINACULUM_COM = new Vector3D(0.0, 0.0, 0.0);
   public static final Vector3DReadOnly RETINACULUM_I = new Vector3D(0.000260143, 0.000260143, 0.000260143);

   // main body
   public static double BODY_X_RADIUS = 0.15, BODY_HEIGHT = 0.8;
   public static double BODY_RADIUS_OF_GYRATION_Y = 0.2;
   public static double BODY_RADIUS_OF_GYRATION_Z = 0.2;
   public static double BODY_RADIUS_OF_GYRATION_X = 0.2;
   public static final double BODY_MASS = 10;
   public static final Vector3DReadOnly BODY_COM = new Vector3D(0, 0, BODY_HEIGHT);
   public static final Vector3DReadOnly BODY_I = new Vector3D(Math.pow(BODY_RADIUS_OF_GYRATION_X, 2) * BODY_MASS,
                                                              Math.pow(BODY_RADIUS_OF_GYRATION_Y, 2) * BODY_MASS,
                                                              Math.pow(BODY_RADIUS_OF_GYRATION_Z, 2) * BODY_MASS);
   // Thigh
   public static final double THIGH_MASS = 0.2;
   public static double thighRadiusOfGyrationX = 0.01;
   public static double thighRadiusOfGyrationY = 0.01;
   public static double thighRadiusOfGyrationZ = 0.01;
   public static double thighLength = 0.6;
   public static double THIGH_R = 0.05;

   // Shin
   public static double SHIN_R_OF_GYRATION_X = 0.01;
   public static double SHIN_R_OF_GYRATION_Y = 0.01;
   public static double SHIN_R_OF_GYRATION_Z = 0.01;
   public static double SHIN_LEN = 0.6;
   public static final double SHIN_R = 0.051;
   public static final double SHIN_MASS = 0.05;
   public static final Vector3DReadOnly L_SHIN_COM = new Vector3D(0, BODY_X_RADIUS + THIGH_R, FOOT_HEIGHT + SHIN_LEN / 2);
   public static final Vector3DReadOnly R_SHIN_COM = new Vector3D(0, -BODY_X_RADIUS - THIGH_R, FOOT_HEIGHT + SHIN_LEN / 2);
   public static final Vector3DReadOnly SHIN_I = new Vector3D(Math.pow(SHIN_R_OF_GYRATION_X, 2) * SHIN_MASS,
                                                              Math.pow(SHIN_R_OF_GYRATION_Y, 2) * SHIN_MASS,
                                                              Math.pow(SHIN_R_OF_GYRATION_Z, 2) * SHIN_MASS);
   // Knee
   public static final double KNEE_LOWER_LIMIT = -5 * SHIN_LEN;
   public static final double KNEE_UPPER_LIMIT = SHIN_LEN;
   public static final double KNEE_SPRING = 10;
   public static final double KNEE_DAMPING = 8;



   public static final Vector3DReadOnly L_THIGH_COM = new Vector3D(0, BODY_X_RADIUS + THIGH_R, FOOT_HEIGHT + SHIN_LEN);
   public static final Vector3DReadOnly R_THIGH_COM = new Vector3D(0, -BODY_X_RADIUS - THIGH_R, FOOT_HEIGHT + SHIN_LEN);
   public static final Vector3DReadOnly THIGH_I = new Vector3D(Math.pow(thighRadiusOfGyrationX, 2) * THIGH_MASS,
                                                               Math.pow(thighRadiusOfGyrationY, 2) * THIGH_MASS,
                                                               Math.pow(thighRadiusOfGyrationZ, 2) * THIGH_MASS);

   // Hip
   private static double HIP_WIDTH = BODY_X_RADIUS * 2;



   public SideDependentList<RevoluteJointDefinition> footParentJoints = new SideDependentList<>();
   public final boolean onlyMain = false;



   // NOTE: constructor
   public LIPMWalkerRobotDefinition()
   {
      super(name);

      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      setRootBodyDefinition(elevator);

      // Define and add a floating joint to the robot base
      SixDoFJointDefinition floatingJoint = new SixDoFJointDefinition(getRootJointName());

      //The robot needs to start standing on the ground because the controller will start in the "standing"-state and will expect ground contact
      floatingJoint.setInitialJointState(new SixDoFJointState(null, new Vector3D(0.0, 0.0, 0.35)));
      elevator.addChildJoint(floatingJoint);

      RigidBodyDefinition mainBody = createBody();
      floatingJoint.setSuccessor(mainBody);

      if (!onlyMain)
      {
         // Right Leg
         // NOTE: right hip joint
         OneDoFJointDefinition[] rightHipJoints = createUniversalJoint("right_hip_yaw",
                                                                      "right_hip_pitch",
                                                                      "right_hip_roll",
                                                                      new Vector3D(0.0, -HIP_WIDTH / 2.0, BODY_HEIGHT / 2),
                                                                      Axis3D.Z,
                                                                      Axis3D.Y,
                                                                      Axis3D.X,
                                                                      mainBody);

         // right hip body (thigh)
         RigidBodyDefinition rightThigh = createThigh(RobotSide.RIGHT, 0,0,0,"right_thigh_body");
         rightHipJoints[2].setSuccessor(rightThigh);

         PrismaticJointDefinition rightKneeJoint = new PrismaticJointDefinition("right_knee_joint", new Vector3D(0, 0,-thighLength / 2), new Vector3D(0.0, 0.0, -1.0));
         rightKneeJoint.setPositionLimits(KNEE_LOWER_LIMIT, KNEE_UPPER_LIMIT);
         rightKneeJoint.setKpSoftLimitStop(KNEE_SPRING);
         rightKneeJoint.setKdSoftLimitStop(KNEE_DAMPING);
         rightThigh.addChildJoint(rightKneeJoint);

         RigidBodyDefinition rightShinBody = createShin(RobotSide.RIGHT,0,0,0,"right_shin_body", ColorDefinitions.Green());
         rightKneeJoint.setSuccessor(rightShinBody);

         RevoluteJointDefinition rightAnkleRoll = new RevoluteJointDefinition("right_ankle_roll", new Vector3D(0.0, 0.0, -SHIN_LEN / 2), Axis3D.X);
         RigidBodyDefinition rightRetinaculumBody = retinaculum("right");
         rightShinBody.addChildJoint(rightAnkleRoll);
         rightAnkleRoll.setSuccessor(rightRetinaculumBody);

         RevoluteJointDefinition rightAnklePitch = new RevoluteJointDefinition("right_ankle_pitch", new Vector3D(0.0, 0.0, -ANKLE_JOINT_OFF), Axis3D.Y);
         RigidBodyDefinition rightFootBody = foot(RobotSide.RIGHT);
         rightRetinaculumBody.addChildJoint(rightAnklePitch);
         rightAnklePitch.setSuccessor(rightFootBody);

         footParentJoints.put(RobotSide.RIGHT, rightAnklePitch);

         /*
          * Here are defined the contact points that the simulation can use to make the `ground` interact with
          * the robot. Without them the robot would fall through the ground.
          */
         GroundContactPointDefinition right_toe_in = new GroundContactPointDefinition("gc_right_toe_in",
                                                                                      new Vector3D(FOOT_FORWARD, FOOT_WIDTH / 2.0, -FOOT_HEIGHT));
         GroundContactPointDefinition right_toe_out = new GroundContactPointDefinition("gc_right_toe_out",
                                                                                       new Vector3D(FOOT_FORWARD, -FOOT_WIDTH / 2.0, -FOOT_HEIGHT));
         GroundContactPointDefinition right_heel_in = new GroundContactPointDefinition("gc_right_heel_in",
                                                                                       new Vector3D(-FOOT_BACK, FOOT_WIDTH / 2.0, -FOOT_HEIGHT));
         GroundContactPointDefinition right_heel_out = new GroundContactPointDefinition("gc_right_heel_out",
                                                                                        new Vector3D(-FOOT_BACK, -FOOT_WIDTH / 2.0, -FOOT_HEIGHT));

         rightAnklePitch.addGroundContactPointDefinition(right_toe_in);
         rightAnklePitch.addGroundContactPointDefinition(right_toe_out);
         rightAnklePitch.addGroundContactPointDefinition(right_heel_in);
         rightAnklePitch.addGroundContactPointDefinition(right_heel_out);

         for (GroundContactPointDefinition contactPoint : rightAnklePitch.getGroundContactPointDefinitions())
         {
            rightFootBody.addVisualDefinition(new VisualDefinition(contactPoint.getTransformToParent().getTranslation(),
                                                                  new Sphere3DDefinition(0.05),
                                                                  new MaterialDefinition(ColorDefinitions.Yellow())));
         }

         // LEFT LEG
         // NOTE: left hip joint
         OneDoFJointDefinition[] leftHipJoints = createUniversalJoint("left_hip_yaw",
                                                                      "left_hip_pitch",
                                                                      "left_hip_roll",
                                                                      new Vector3D(0.0, HIP_WIDTH / 2.0, BODY_HEIGHT / 2),
                                                                      Axis3D.Z,
                                                                      Axis3D.Y,
                                                                      Axis3D.X,
                                                                      mainBody);

//         OneDoFJointDefinition[] leftHipJoints = createUniversalJoint2DOF("left_hip_yaw",
//                                                                      "left_hip_pitch",
//                                                                      new Vector3D(0.0, hipWidth / 2.0, bodyHeight / 2),
//                                                                      Axis3D.Z,
//                                                                      Axis3D.Y,
//                                                                      mainBody);
         // left hip body (thigh)
         RigidBodyDefinition leftThigh = createThigh(RobotSide.LEFT,0,0,0,"left_thigh_body");
         leftHipJoints[2].setSuccessor(leftThigh);

         PrismaticJointDefinition leftKneeJoint = new PrismaticJointDefinition("left_knee_joint", new Vector3D(0, 0, -thighLength / 2), new Vector3D(0.0, 0.0, -1.0));
         leftKneeJoint.setPositionLimits(KNEE_LOWER_LIMIT, KNEE_UPPER_LIMIT);
         leftKneeJoint.setKpSoftLimitStop(KNEE_SPRING);
         leftKneeJoint.setKdSoftLimitStop(KNEE_DAMPING);
         leftThigh.addChildJoint(leftKneeJoint);

         RigidBodyDefinition leftShinBody = createShin(RobotSide.LEFT,0,0,0,"left_shin_body", ColorDefinitions.Red());
         leftKneeJoint.setSuccessor(leftShinBody);

         RevoluteJointDefinition leftAnkleRoll = new RevoluteJointDefinition("left_ankle_roll", new Vector3D(0.0, 0.0, -SHIN_LEN / 2), Axis3D.X);
         RigidBodyDefinition leftRetinaculumBody = retinaculum("left");
         leftShinBody.addChildJoint(leftAnkleRoll);
         leftAnkleRoll.setSuccessor(leftRetinaculumBody);

         RevoluteJointDefinition leftAnklePitch = new RevoluteJointDefinition("left_ankle_pitch", new Vector3D(0.0, 0.0, -ANKLE_JOINT_OFF), Axis3D.Y);
         RigidBodyDefinition leftFootBody = foot(RobotSide.LEFT);
         leftRetinaculumBody.addChildJoint(leftAnklePitch);
         leftAnklePitch.setSuccessor(leftFootBody);

         footParentJoints.put(RobotSide.LEFT, leftAnklePitch);

         /*
          * Here are defined the contact points that the simulation can use to make the ground interact with
          * the robot. Without them the robot would fall through the ground.
          */
         GroundContactPointDefinition left_toe_in = new GroundContactPointDefinition("gc_left_toe_in",
                                                                                     new Vector3D(FOOT_FORWARD, FOOT_WIDTH / 2.0, -FOOT_HEIGHT));
         GroundContactPointDefinition left_toe_out = new GroundContactPointDefinition("gc_left_toe_out",
                                                                                      new Vector3D(FOOT_FORWARD, -FOOT_WIDTH / 2.0, -FOOT_HEIGHT));
         GroundContactPointDefinition left_heel_in = new GroundContactPointDefinition("gc_left_heel_in",
                                                                                      new Vector3D(-FOOT_BACK, FOOT_WIDTH / 2.0, -FOOT_HEIGHT));
         GroundContactPointDefinition left_heel_out = new GroundContactPointDefinition("gc_left_heel_out",
                                                                                       new Vector3D(-FOOT_BACK, -FOOT_WIDTH / 2.0, -FOOT_HEIGHT));

         leftAnklePitch.addGroundContactPointDefinition(left_toe_in);
         leftAnklePitch.addGroundContactPointDefinition(left_toe_out);
         leftAnklePitch.addGroundContactPointDefinition(left_heel_in);
         leftAnklePitch.addGroundContactPointDefinition(left_heel_out);

         for (GroundContactPointDefinition contactPoint : leftAnklePitch.getGroundContactPointDefinitions())
         {
            leftFootBody.addVisualDefinition(new VisualDefinition(contactPoint.getTransformToParent().getTranslation(),
                                                              new Sphere3DDefinition(0.05),
                                                              new MaterialDefinition(ColorDefinitions.Yellow())));
         }


//          Set damping to 0.0 (due to bug in OneDoFJointDefinition which initializes the damping to -1)
         forEachOneDoFJointDefinition(j -> j.setDamping(0.0));
      }

   }

   // cylindrical main body
   private RigidBodyDefinition createBody()
   {
      // define body
      RigidBodyDefinition mainBody = new RigidBodyDefinition("mainBody");
      mainBody.setMass(BODY_MASS);
      mainBody.setCenterOfMassOffset(new Vector3D(BODY_COM));
      mainBody.getMomentOfInertia().setToDiagonal(BODY_I.getX(),
                                                  BODY_I.getY(),
                                                  BODY_I.getZ());

      // add graphics definition.
      GeometryDefinition capsule = new Capsule3DDefinition(BODY_HEIGHT, BODY_X_RADIUS);
      RigidBodyTransform capsulePose = new RigidBodyTransform();
      capsulePose.appendTranslation(0.0, 0.0, BODY_COM.getZ());

      MaterialDefinition materialDefinition = new MaterialDefinition(ColorDefinitions.LightBlue());

      mainBody.addVisualDefinition(new VisualDefinition(capsulePose, capsule, materialDefinition));

      return mainBody;
   }

   private RigidBodyDefinition createThigh(RobotSide side, double offSetX, double offSetY, double offSetZ, String name)
   {
      RigidBodyDefinition thigh = new RigidBodyDefinition(name);
      thigh.setMass(THIGH_MASS);

      Vector3DReadOnly thighCOMToUse = side == RobotSide.LEFT ? L_THIGH_COM : R_THIGH_COM;
      thigh.setCenterOfMassOffset(new Vector3D(thighCOMToUse));
      thigh.getMomentOfInertia().setToDiagonal(THIGH_I.getX(),
                                               THIGH_I.getY(),
                                               THIGH_I.getZ());

//      GeometryDefinition geometryDefinition = new Cylinder3DDefinition(thighLength, thighRadius);
      GeometryDefinition geometryDefinition = new Sphere3DDefinition(THIGH_R);
      RigidBodyTransform thighPose = new RigidBodyTransform();
      thighPose.appendTranslation(offSetX, offSetY, offSetZ);
      MaterialDefinition materialDefinition = new MaterialDefinition(ColorDefinitions.DarkGrey());

      thigh.addVisualDefinition(new VisualDefinition(thighPose, geometryDefinition, materialDefinition));
      return thigh;
   }

   private RigidBodyDefinition createShin(RobotSide side, double offSetX, double offSetY, double offSetZ, String name, ColorDefinition color)
   {
      RigidBodyDefinition shin = new RigidBodyDefinition(name);
      shin.setMass(SHIN_MASS);
      Vector3DReadOnly shinCOMToUse = side == RobotSide.LEFT ? L_SHIN_COM : R_SHIN_COM;
      shin.setCenterOfMassOffset(new Vector3D(shinCOMToUse));

      shin.getMomentOfInertia().setToDiagonal(SHIN_R_OF_GYRATION_X * SHIN_MASS,
                                              SHIN_R_OF_GYRATION_Y * SHIN_MASS,
                                              SHIN_R_OF_GYRATION_Z * SHIN_MASS);
      GeometryDefinition geometryDefinition = new Cylinder3DDefinition(SHIN_LEN, SHIN_R);
      RigidBodyTransform shinPose = new RigidBodyTransform();
      shinPose.appendTranslation(offSetX, offSetY, offSetZ);
      MaterialDefinition materialDefinition = new MaterialDefinition(color);
      shin.addVisualDefinition(new VisualDefinition(shinPose, geometryDefinition, materialDefinition));

      return shin;
   }

   public String getRootJointName()
   {
      return "rootJoint";
   }

   public OneDoFJointDefinition[] createUniversalJoint(String jname1,
                                                       String jname2,
                                                       String jname3,
                                                       Tuple3DReadOnly offset,
                                                       Vector3DReadOnly firstAxis,
                                                       Vector3DReadOnly secondAxis,
                                                       Vector3DReadOnly thirdAxis,
                                                       RigidBodyDefinition predecessor)
   {
      RevoluteJointDefinition joint1 = new RevoluteJointDefinition(jname1);
      RevoluteJointDefinition joint2 = new RevoluteJointDefinition(jname2);
      RevoluteJointDefinition joint3 = new RevoluteJointDefinition(jname3);

      RigidBodyDefinition joint1Body = createNullBody(jname1 + "Body");
      RigidBodyDefinition joint2Body = createNullBody(jname2 + "Body");
      RigidBodyDefinition joint3Body = createNullBody(jname3 + "Body");

      joint1.getTransformToParent().getTranslation().set(offset);
      joint1.getAxis().set(firstAxis);
      joint2.getAxis().set(secondAxis);
      joint3.getAxis().set(thirdAxis);

      predecessor.getChildrenJoints().add(joint1);
      joint1.setSuccessor(joint1Body);
      joint1Body.getChildrenJoints().add(joint2);
      joint2.setSuccessor(joint2Body);
      joint2Body.getChildrenJoints().add(joint3);
      joint3.setSuccessor(joint3Body);

      return new OneDoFJointDefinition[] {joint1, joint2, joint3};
   }

   public OneDoFJointDefinition[] createUniversalJoint2DOF(String jname1,
                                                       String jname2,
                                                       Tuple3DReadOnly offset,
                                                       Vector3DReadOnly firstAxis,
                                                       Vector3DReadOnly secondAxis,
                                                       RigidBodyDefinition predecessor)
   {
      RevoluteJointDefinition joint1 = new RevoluteJointDefinition(jname1);
      RevoluteJointDefinition joint2 = new RevoluteJointDefinition(jname2);

      RigidBodyDefinition joint1Body = createNullBody(jname1 + "Body");
      RigidBodyDefinition joint2Body = createNullBody(jname2 + "Body");

      joint1.getTransformToParent().getTranslation().set(offset);
      joint1.getAxis().set(firstAxis);
      joint2.getAxis().set(secondAxis);

      predecessor.getChildrenJoints().add(joint1);
      joint1.setSuccessor(joint1Body);
      joint1Body.getChildrenJoints().add(joint2);
      joint2.setSuccessor(joint2Body);

      return new OneDoFJointDefinition[] {joint1, joint2};
   }

   private RigidBodyDefinition createNullBody(String name)
   {
      RigidBodyDefinition nullBody = new RigidBodyDefinition(name);
      nullBody.setMass(1.0e-12);
      nullBody.getMomentOfInertia().setToDiagonal(1.0e-12, 1.0e-12, 1.0e-12);
      return nullBody;
   }

   private RigidBodyDefinition retinaculum(String side)
   {
      RigidBodyDefinition ret = new RigidBodyDefinition(side + "_retinaculum");
      ret.setMass(RETINACULUM_MASS);
      ret.setCenterOfMassOffset(new Vector3D(RETINACULUM_COM));
      ret.getMomentOfInertia().setToDiagonal(RETINACULUM_I.getX(), RETINACULUM_I.getY(), RETINACULUM_I.getZ());

      GeometryDefinition geometryDefinition = new Sphere3DDefinition(SHIN_R);
      MaterialDefinition materialDefinition = new MaterialDefinition(ColorDefinitions.White());

      ret.addVisualDefinition(new VisualDefinition(geometryDefinition, materialDefinition));

      return ret;
   }

   private RigidBodyDefinition foot(RobotSide robotSide)
   {
      RigidBodyDefinition ret = new RigidBodyDefinition(robotSide.getCamelCaseName() + "foot");
      ret.setMass(FOOT_MASS);
      ret.setCenterOfMassOffset(new Vector3D(FOOT_COM));
      ret.getMomentOfInertia().setToDiagonal(FOOT_I.getX(), FOOT_I.getY(), FOOT_I.getZ());

      GeometryDefinition geometryDefinition = new Box3DDefinition(FOOT_LENGTH, FOOT_WIDTH, FOOT_HEIGHT);
      RigidBodyTransform geometryPose = new RigidBodyTransform();
      geometryPose.appendTranslation(FOOT_FORWARD - 0.5 * FOOT_LENGTH, 0.0, -0.5 * FOOT_HEIGHT);

      MaterialDefinition materialDefinition = new MaterialDefinition(ColorDefinitions.DarkCyan());

      ret.addVisualDefinition(new VisualDefinition(geometryPose, geometryDefinition, materialDefinition));

      return ret;
   }

   public RevoluteJointDefinition getFootParentJoint(RobotSide robotSide)
   {
      return footParentJoints.get(robotSide);
   }
}
