package us.ihmc.valkyrie.parameters;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionBox;
import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionCylinder;
import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionShape;
import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionSphere;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

/**
 * There is no collision provided in Valkyrie's SDF/URDF models, so it is created and hardcoded here.
 *
 */
public class ValkyrieCollisionBoxProvider implements CollisionBoxProvider
{
   private final Map<String, List<CollisionShape>> collisions = new HashMap<>();

   public ValkyrieCollisionBoxProvider(FullHumanoidRobotModel robotModel)
   {
      collisions.put(robotModel.getRootJoint().getName(), new ArrayList<>());
      for (OneDoFJoint joint : robotModel.getOneDoFJoints())
         collisions.put(joint.getName(), new ArrayList<>());

      for (RobotSide robotSide : RobotSide.values)
      {
         { // Shoulder roll
            String jointName = robotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_ROLL).getName();

            { // Shoulder
               AxisAngle rotation = new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0);
               Vector3D translation = new Vector3D(-0.01, 0.0, 0.0);
               RigidBodyTransform pose = new RigidBodyTransform(rotation, translation);
               collisions.get(jointName).add(new CollisionCylinder(pose, 0.09, 1.3 * 0.23));
            }
            { // Upper-arm
               AxisAngle rotation = new AxisAngle(1.0, 0.0, 0.0, Math.PI / 2.0);
               Vector3D translation = new Vector3D(0.0, robotSide.negateIfRightSide(0.16), 0.0);
               RigidBodyTransform pose = new RigidBodyTransform(rotation, translation);
               collisions.get(jointName).add(new CollisionCylinder(pose, 0.09, 0.32));
            }
         }

         { // Elbow
            String jointName = robotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH).getName();

            { // Elbow
               AxisAngle rotation = new AxisAngle();
               Vector3D translation = new Vector3D(0.0, 0.0, 0.0);
               RigidBodyTransform pose = new RigidBodyTransform(rotation, translation);
               collisions.get(jointName).add(new CollisionCylinder(pose, 0.06, 0.15));
            }

            { // Forearm
               AxisAngle rotation = new AxisAngle(1.0, 0.0, 0.0, Math.PI / 2.0);
               Vector3D translation = new Vector3D(-0.02, robotSide.negateIfRightSide(0.14), 0.0);
               RigidBodyTransform pose = new RigidBodyTransform(rotation, translation);
               collisions.get(jointName).add(new CollisionCylinder(pose, 0.09, 0.28));
            }
         }

         { // Wrist
            String jointName = robotModel.getHand(robotSide).getParentJoint().getName();

            { // Conservative sphere to wrap the hand plus fingers. Can be improved
               AxisAngle rotation = new AxisAngle();
               Vector3D translation = new Vector3D(0.0, robotSide.negateIfRightSide(0.07), 0.02);
               RigidBodyTransform pose = new RigidBodyTransform(rotation, translation);
               collisions.get(jointName).add(new CollisionSphere(pose, 1.3 * 0.12));
            }
         }

         { // Hip yaw
            String jointName = robotModel.getLegJoint(robotSide, LegJointName.HIP_YAW).getName();

            { // The coconut shells
               AxisAngle rotation = new AxisAngle();
               Vector3D translation = new Vector3D(0.0, robotSide.negateIfRightSide(0.00), -0.03);
               RigidBodyTransform pose = new RigidBodyTransform(rotation, translation);
               collisions.get(jointName).add(new CollisionSphere(pose, 0.18));
            }
         }

         { // Hip pitch
            String jointName = robotModel.getLegJoint(robotSide, LegJointName.HIP_PITCH).getName();

            { // Thigh
               AxisAngle rotation = new AxisAngle();
               Vector3D translation = new Vector3D(0.02, robotSide.negateIfRightSide(0.08), -0.20);
               RigidBodyTransform pose = new RigidBodyTransform(rotation, translation);
               collisions.get(jointName).add(new CollisionBox(pose, 0.13, 0.1, 0.29));
            }
         }

         { // Knee
            String jointName = robotModel.getLegJoint(robotSide, LegJointName.KNEE_PITCH).getName();

            { // Shin
               AxisAngle rotation = new AxisAngle();
               Vector3D translation = new Vector3D(-0.01, robotSide.negateIfRightSide(0.0), -0.18);
               RigidBodyTransform pose = new RigidBodyTransform(rotation, translation);
               collisions.get(jointName).add(new CollisionBox(pose, 0.12, 0.1, 0.20));
            }
         }

         { // Ankle
            String jointName = robotModel.getFoot(robotSide).getParentJoint().getName();

            { // Foot
               AxisAngle rotation = new AxisAngle();
               Vector3D translation = new Vector3D(0.05, robotSide.negateIfRightSide(0.0), -0.04);
               RigidBodyTransform pose = new RigidBodyTransform(rotation, translation);
               collisions.get(jointName).add(new CollisionBox(pose, 0.16, 0.09, 0.06));
            }
         }
      }

      { // Head
         String jointName = robotModel.getHead().getParentJoint().getName();

         { // Head
            AxisAngle rotation = new AxisAngle();
            Vector3D translation = new Vector3D(0.08, 0.0, 0.0);
            RigidBodyTransform pose = new RigidBodyTransform(rotation, translation);
            collisions.get(jointName).add(new CollisionSphere(pose, 0.18));
         }
      }

      { // Torso
         String jointName = robotModel.getChest().getParentJoint().getName();

         { // Torso
            AxisAngle rotation = new AxisAngle();
            Vector3D translation = new Vector3D(-0.08, 0.0, 0.2);
            RigidBodyTransform pose = new RigidBodyTransform(rotation, translation);
            collisions.get(jointName).add(new CollisionBox(pose, 1.3 * 0.23, 1.3 * 0.2, 0.22));
         }

         { // Cylinder to remove the ropes from the hoist.
            AxisAngle rotation = new AxisAngle();
            Vector3D translation = new Vector3D(-0.05, 0.0, 0.54);
            RigidBodyTransform pose = new RigidBodyTransform(rotation, translation);
            collisions.get(jointName).add(new CollisionCylinder(pose, 0.22, 0.60));
         }
      }

      { // Pelvis
         String jointName = robotModel.getPelvis().getParentJoint().getName();

         { // Pelvis pt1: The upper part
            AxisAngle rotation = new AxisAngle();
            Vector3D translation = new Vector3D(0.0, 0.0, -0.08);
            RigidBodyTransform pose = new RigidBodyTransform(rotation, translation);
            collisions.get(jointName).add(new CollisionBox(pose, 1.3 * 0.13, 0.18, 0.08));
         }

         { // Pelvis pt2: The crotch part
            AxisAngle rotation = new AxisAngle();
            Vector3D translation = new Vector3D(0.0, 0.0, -0.25);
            RigidBodyTransform pose = new RigidBodyTransform(rotation, translation);
            collisions.get(jointName).add(new CollisionBox(pose, 0.17, 0.05, 0.09));
         }
      }
   }

   @Override
   public List<CollisionShape> getCollisionMesh(String jointName)
   {
      return collisions.get(jointName);
   }
}
