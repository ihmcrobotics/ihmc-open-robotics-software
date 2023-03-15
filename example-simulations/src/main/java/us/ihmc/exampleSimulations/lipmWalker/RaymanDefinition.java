package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.geometry.*;
import us.ihmc.scs2.definition.robot.*;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class RaymanDefinition extends RobotDefinition
{
   private static final String name = "Rayman";

   // robot parameters
   public static final double HEIGHT = 0.8;
   public static final Vector3DReadOnly LEFT_HIP_JOINT_OFFSET = new Vector3D(0, 0.3, 0);
   public static final Vector3DReadOnly RIGHT_HIP_JOINT_OFFSET = new Vector3D(0, -0.3, 0);
   public static final Vector3DReadOnly LEFT_KNEE_JOINT_OFFSET = new Vector3D(0, 0, -0.4);
   public static final Vector3DReadOnly RIGHT_KNEE_JOINT_OFFSET = new Vector3D(0, 0, -0.4);
   public static final Vector3DReadOnly LEFT_ANKLE_JOINT_OFFSET = new Vector3D(0, 0, -0.4);
   public static final Vector3DReadOnly RIGHT_ANKLE_JOINT_OFFSET = new Vector3D(0, 0, -0.4);

   // body
   public static final double BODY_MASS = 10;
   public static final double BODY_R = 0.2;
   public static double BODY_R_GYRATION = 0.1;
   public static final Vector3DReadOnly BODY_COM = new Vector3D(0, 0, 0);
   public static final Vector3DReadOnly BODY_I = new Vector3D(Math.pow(BODY_R_GYRATION, 2) * BODY_MASS,
                                                              Math.pow(BODY_R_GYRATION, 2) * BODY_MASS,
                                                              Math.pow(BODY_R_GYRATION, 2) * BODY_MASS);

   // Retinaculum
   public static final double RETINACULUM_MASS = 0.250041;
   public static final double RETINACULUM_R = 0.05;
   public static final Vector3DReadOnly RETINACULUM_COM = new Vector3D(0.0, 0.0, 0.0);
   public static final Vector3DReadOnly RETINACULUM_I = new Vector3D(0.000260143, 0.000260143, 0.000260143);

   // foot
   public static final double FOOT_FORWARD = 0.1524;
   public static final double FOOT_BACK = 0.051;
   public static final double FOOT_LENGTH = FOOT_BACK + FOOT_FORWARD;
   public static final double FOOT_WIDTH = 0.15;
   public static final double FOOT_HEIGHT = 0.051;
//   public static final double FOOT_MASS = 1.0;
//   public static final Vector3DReadOnly FOOT_COM = new Vector3D(0.050700, 0.0, -FOOT_HEIGHT / 2);
//   public static final Vector3DReadOnly FOOT_I = new Vector3D(0.00036326, 0.00152067, 0.00170404);
//   public static final Vector3DReadOnly FOOT_COM = new Vector3D(FOOT_LENGTH / 2, 0.0, -FOOT_HEIGHT / 2);
//   public static final Vector3DReadOnly FOOT_I = new Vector3D(0.001, 0.0035, 0.004);

   public static final double FOOT_MASS = 0.414988;
   public static final Vector3DReadOnly FOOT_COM = new Vector3D(0.050700, 0.0, -0.025500);
   public static final Vector3DReadOnly FOOT_I = new Vector3D(0.00036326, 0.00152067, 0.00170404);

   //etc
   public SideDependentList<RevoluteJointDefinition> footParentJoints = new SideDependentList<>();
   public static final double KNEE_SPRING = 100;
   public static final double KNEE_DAMPING = 80;

   // NOTE: constructor
   public RaymanDefinition()
   {
      super(name);

      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      setRootBodyDefinition(elevator);

      // Define and add a floating joint to the robot base
      SixDoFJointDefinition floatingJoint = new SixDoFJointDefinition(getRootJointName());

      //The robot needs to start standing on the ground because the controller will start in the "standing"-state and will expect ground contact
      floatingJoint.setInitialJointState(new SixDoFJointState(null, new Vector3D(0.0, 0.0, 0.95)));
      elevator.addChildJoint(floatingJoint);

      // main body
      RigidBodyDefinition mainBody = createBody();
      floatingJoint.setSuccessor(mainBody);

      /*
      LEFT SIDE
       */

      // left hip joint
      OneDoFJointDefinition[] leftHipJoints = createUniversalJoint("left_hip_yaw",
                                                                    "left_hip_pitch",
                                                                    "left_hip_roll",
                                                                    LEFT_HIP_JOINT_OFFSET,
                                                                    Axis3D.Z,
                                                                    Axis3D.Y,
                                                                    Axis3D.X,
                                                                    mainBody);
//      RigidBodyDefinition nullLeftHip = createNullBody("null_left_hip");
      RigidBodyDefinition nullLeftHip = createNullCapsule("leftHip", 0.05, new Vector3D(), ColorDefinitions.LightCoral());
      leftHipJoints[2].setSuccessor(nullLeftHip);

      // left prismatic (knee) joint
      PrismaticJointDefinition leftKneeJoint = new PrismaticJointDefinition("left_knee_joint", LEFT_KNEE_JOINT_OFFSET, Axis3D.Z.negated());
      leftKneeJoint.setPositionLimits(-5, 5);   // TODO: temp limits for now
      leftKneeJoint.setKpSoftLimitStop(KNEE_SPRING);
      leftKneeJoint.setKdSoftLimitStop(KNEE_DAMPING);
      nullLeftHip.addChildJoint(leftKneeJoint);

//      RigidBodyDefinition nullLeftShin = createNullBody("null_left_shin");
      RigidBodyDefinition nullLeftShin = createNullCapsule("null_left_shin", 0.05, new Vector3D(), ColorDefinitions.Red());
      leftKneeJoint.setSuccessor(nullLeftShin);

      // left ankle roll
      RevoluteJointDefinition leftAnkleRoll = new RevoluteJointDefinition("left_ankle_roll", LEFT_ANKLE_JOINT_OFFSET, Axis3D.X);
      RigidBodyDefinition leftRetinaculum = retinaculum("left");
      nullLeftShin.addChildJoint(leftAnkleRoll);
      leftAnkleRoll.setSuccessor(leftRetinaculum);

      // left ankle pitch
      RevoluteJointDefinition leftAnklePitch = new RevoluteJointDefinition("left_ankle_pitch", new Vector3D(), Axis3D.Y);
      RigidBodyDefinition leftFootBody = foot(RobotSide.LEFT);
      leftRetinaculum.addChildJoint(leftAnklePitch);
      leftAnklePitch.setSuccessor(leftFootBody);
      footParentJoints.put(RobotSide.LEFT, leftAnklePitch);

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

      /*
      RIGHT SIDE
       */

      // right hip joint
      OneDoFJointDefinition[] rightHipJoints = createUniversalJoint("right_hip_yaw",
                                                                   "right_hip_pitch",
                                                                   "right_hip_roll",
                                                                   RIGHT_HIP_JOINT_OFFSET,
                                                                   Axis3D.Z,
                                                                   Axis3D.Y,
                                                                   Axis3D.X,
                                                                   mainBody);
      // RigidBodyDefinition nullRightHip = createNullBody("null_right_hip");
      RigidBodyDefinition nullRightHip = createNullCapsule("rightHip", 0.05, new Vector3D(), ColorDefinitions.LightGreen());
      rightHipJoints[2].setSuccessor(nullRightHip);

      // right prismatic (knee) joint
      PrismaticJointDefinition rightKneeJoint = new PrismaticJointDefinition("right_knee_joint", RIGHT_KNEE_JOINT_OFFSET, Axis3D.Z.negated());
      rightKneeJoint.setPositionLimits(-5, 5);   // TODO: temp limits for now
      rightKneeJoint.setKpSoftLimitStop(KNEE_SPRING);
      rightKneeJoint.setKdSoftLimitStop(KNEE_DAMPING);
      nullRightHip.addChildJoint(rightKneeJoint);

//      RigidBodyDefinition nullRightShin = createNullBody("null_right_shin");
      RigidBodyDefinition nullRightShin = createNullCapsule("null_right_shin", 0.05, new Vector3D(), ColorDefinitions.Green());
      rightKneeJoint.setSuccessor(nullRightShin);

      // right ankle roll
      RevoluteJointDefinition rightAnkleRoll = new RevoluteJointDefinition("right_ankle_roll", RIGHT_ANKLE_JOINT_OFFSET, Axis3D.X);
      RigidBodyDefinition rightRetinaculum = retinaculum("right");
      nullRightShin.addChildJoint(rightAnkleRoll);
      rightAnkleRoll.setSuccessor(rightRetinaculum);

      // right ankle pitch
      RevoluteJointDefinition rightAnklePitch = new RevoluteJointDefinition("right_ankle_pitch", new Vector3D(), Axis3D.Y);
      RigidBodyDefinition rightFootBody = foot(RobotSide.RIGHT);
      rightRetinaculum.addChildJoint(rightAnklePitch);
      rightAnklePitch.setSuccessor(rightFootBody);
      footParentJoints.put(RobotSide.RIGHT, rightAnklePitch);

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


      // visualize contact points for each foot
      for (GroundContactPointDefinition contactPoint : leftAnklePitch.getGroundContactPointDefinitions())
      {
         leftFootBody.addVisualDefinition(new VisualDefinition(contactPoint.getTransformToParent().getTranslation(),
                                                               new Sphere3DDefinition(0.05),
                                                               new MaterialDefinition(ColorDefinitions.DarkRed())));
      }

      for (GroundContactPointDefinition contactPoint : rightAnklePitch.getGroundContactPointDefinitions())
      {
         rightFootBody.addVisualDefinition(new VisualDefinition(contactPoint.getTransformToParent().getTranslation(),
                                                               new Sphere3DDefinition(0.05),
                                                               new MaterialDefinition(ColorDefinitions.DarkGreen())));
      }


      // Set damping to 0.0 (due to bug in OneDoFJointDefinition which initializes the damping to -1)
      forEachOneDoFJointDefinition(j -> j.setDamping(0.0));
   }

   private RigidBodyDefinition createNullCapsule(String title, double radius, Vector3DReadOnly offset, ColorDefinition color)
   {
      // body
      RigidBodyDefinition sphere = new RigidBodyDefinition(title);
      sphere.setMass(1.0e-12);
      sphere.getMomentOfInertia().setToDiagonal(1.0e-12, 1.0e-12, 1.0e-12);
      // graphics
      GeometryDefinition geometryDefinition = new Capsule3DDefinition(1.4 * radius, radius);
      MaterialDefinition materialDefinition = new MaterialDefinition(color);
      sphere.addVisualDefinition(new VisualDefinition(new RigidBodyTransform(), geometryDefinition, materialDefinition));
      return sphere;
   }

   // cylindrical main body
   private RigidBodyDefinition createBody()
   {
      // define body
      RigidBodyDefinition body = new RigidBodyDefinition("body");
      body.setMass(BODY_MASS);
      body.setCenterOfMassOffset(new Vector3D(BODY_COM));
      body.getMomentOfInertia().setToDiagonal(BODY_I.getX(), BODY_I.getY(), BODY_I.getZ());

      // add graphics definition.
      GeometryDefinition geometryDefinition = new ArcTorus3DDefinition(0, 2 * Math.PI, BODY_R, 0.5 * BODY_R);
      RigidBodyTransform pose = new RigidBodyTransform();
      pose.appendTranslation(0.0, 0.0, BODY_COM.getZ());
      MaterialDefinition materialDefinition = new MaterialDefinition(ColorDefinitions.BlanchedAlmond());
      body.addVisualDefinition(new VisualDefinition(pose, geometryDefinition, materialDefinition));

      return body;
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

      joint1.setPositionLimits(-Math.PI / 2, Math.PI / 2);
      joint2.setPositionLimits(-Math.PI / 2, Math.PI / 2);
      joint3.setPositionLimits(-Math.PI / 2, Math.PI / 2);

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
      GeometryDefinition geometryDefinition = new Sphere3DDefinition(RETINACULUM_R);
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
