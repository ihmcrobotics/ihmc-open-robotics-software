package us.ihmc.darpaRoboticsChallenge.reachabilityMapCalculator.example;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.tools.FormattingTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class RobotParameters
{
//   private static final RigidBodyTransform endEffectorTransformToWrist = new RigidBodyTransform(new AxisAngle4d(0.0, 1.0, 0.0, Math.PI / 2.0), new Vector3d(0.0, 0.0, -0.08));
   private static final RigidBodyTransform endEffectorTransformToWrist = new RigidBodyTransform();

   public RigidBodyTransform getEndEffectorTransformToWrist()
   {
      return endEffectorTransformToWrist;
   }

   public enum RobotArmJointParameters
   {
      SHOULDER_YAW, SHOULDER_PITCH, SHOULDER_ROLL, ELBOW_YAW, ELBOW_PITCH, WRIST_YAW, WRIST_PITCH;

      public static RobotArmJointParameters getRootJoint()
      {
         return SHOULDER_YAW;
      }

      public Vector3d getJointAxis()
      {
         String name = this.toString();
         if (name.endsWith("YAW"))
            return new Vector3d(0.0, 0.0, 1.0);
         else if (name.endsWith("PITCH"))
            return new Vector3d(0.0, 1.0, 0.0);
         else if (name.endsWith("ROLL"))
            return new Vector3d(1.0, 0.0, 0.0);
         else
            throw new RuntimeException("Should not get there.");
      }

      public String getJointName(boolean capitalizeFirstLetter)
      {
         return FormattingTools.underscoredToCamelCase(toString(), capitalizeFirstLetter);
      }

      public Vector3d getJointOffset()
      {
         switch (this)
         {
            case SHOULDER_YAW:   return new Vector3d(0.0, 0.0, 1.5);
            case SHOULDER_PITCH: return new Vector3d(0.0, 0.0, 0.0);
            case SHOULDER_ROLL:  return new Vector3d(0.0, 0.0, 0.0);
            case ELBOW_YAW:      return new Vector3d(0.0, 0.0, -0.5);
            case ELBOW_PITCH:    return new Vector3d(0.0, 0.0, 0.0);
            case WRIST_YAW:      return new Vector3d(0.0, 0.0, -0.5);
            case WRIST_PITCH:    return new Vector3d(0.0, 0.0, 0.0);
            default:             throw new RuntimeException("Should not get there.");
         }
      }

      public double getJointLowerLimit()
      {
         switch (this)
         {
            case SHOULDER_YAW:   return -Math.PI; // -1.57;
            case SHOULDER_PITCH: return -Math.PI;
            case SHOULDER_ROLL:  return -Math.PI; // 0.0;
            case ELBOW_YAW:      return -Math.PI; // -Math.PI;
            case ELBOW_PITCH:    return -Math.PI; // -2.356;
            case WRIST_YAW:      return -Math.PI; // 0.0;
            case WRIST_PITCH:    return -Math.PI; // -Math.PI / 2.0;
            default:             throw new RuntimeException("Should not get there.");
         }
      }

      public double getJointUpperLimit()
      {
         switch (this)
         {
            case SHOULDER_YAW:   return Math.PI; // 0.785;
            case SHOULDER_PITCH: return Math.PI;
            case SHOULDER_ROLL:  return Math.PI; // Math.PI;
            case ELBOW_YAW:      return Math.PI; // Math.PI;
            case ELBOW_PITCH:    return Math.PI; // 0.0;
            case WRIST_YAW:      return Math.PI; // Math.PI;
            case WRIST_PITCH:    return Math.PI; // Math.PI / 2.0;
            default:             throw new RuntimeException("Should not get there.");
         }
      }

      public RobotArmJointParameters getParentJoint()
      {
         switch (this)
         {
            case SHOULDER_YAW:   return null;
            case SHOULDER_PITCH: return SHOULDER_YAW;
            case SHOULDER_ROLL:  return SHOULDER_PITCH;
            case ELBOW_YAW:      return SHOULDER_ROLL;
            case ELBOW_PITCH:    return ELBOW_YAW;
            case WRIST_YAW:      return ELBOW_PITCH;
            case WRIST_PITCH:    return WRIST_YAW;
            default:             throw new RuntimeException("Should not get there.");
         }
      }

      public RobotArmJointParameters getChildJoint()
      {
         switch (this)
         {
            case SHOULDER_YAW:   return SHOULDER_PITCH;
            case SHOULDER_PITCH: return SHOULDER_ROLL;
            case SHOULDER_ROLL:  return ELBOW_YAW;
            case ELBOW_YAW:      return ELBOW_PITCH;
            case ELBOW_PITCH:    return WRIST_YAW;
            case WRIST_YAW:      return WRIST_PITCH;
            case WRIST_PITCH:    return null;
            default:             throw new RuntimeException("Should not get there.");
         }
      }

      public RobotArmLinkParameters getAttachedLink()
      {
         switch (this)
         {
            case SHOULDER_YAW:   return RobotArmLinkParameters.UPPER_SHOULDER;
            case SHOULDER_PITCH: return RobotArmLinkParameters.LOWER_SHOULDER;
            case SHOULDER_ROLL:  return RobotArmLinkParameters.UPPER_ARM;
            case ELBOW_YAW:      return RobotArmLinkParameters.ELBOW;
            case ELBOW_PITCH:    return RobotArmLinkParameters.LOWER_ARM;
            case WRIST_YAW:      return RobotArmLinkParameters.WRIST;
            case WRIST_PITCH:    return RobotArmLinkParameters.HAND;
            default:             throw new RuntimeException("Should not get there.");
         }
      }
   }

   public enum RobotArmLinkParameters
   {
      UPPER_SHOULDER, LOWER_SHOULDER, UPPER_ARM, ELBOW, LOWER_ARM, WRIST, HAND;

      public static RobotArmLinkParameters getEndEffector()
      {
         return HAND;
      }

      public String getLinkName()
      {
         return FormattingTools.underscoredToCamelCase(toString(), true) + "Link";
      }

      public Graphics3DObject getLinkGraphics()
      {
         switch (this)
         {
            case UPPER_SHOULDER:
               Graphics3DObject upperShoulderGraphics = new Graphics3DObject();
               upperShoulderGraphics.addSphere(0.05, YoAppearance.White());
               return upperShoulderGraphics;
            case LOWER_SHOULDER:
               Graphics3DObject lowerShoulderGraphics = new Graphics3DObject();
               lowerShoulderGraphics.addSphere(0.05, YoAppearance.White());
               return lowerShoulderGraphics;
            case UPPER_ARM:
               Graphics3DObject upperArmGraphics = new Graphics3DObject();
               double zOffsetUpperArm = getChildJoint().getJointOffset().getZ() / 2.0;
               upperArmGraphics.translate(0.0, 0.0, zOffsetUpperArm);
               upperArmGraphics.addEllipsoid(0.05, 0.05, Math.abs(zOffsetUpperArm), YoAppearance.DarkBlue());
               return upperArmGraphics;
            case ELBOW:
               Graphics3DObject elbowGraphics = new Graphics3DObject();
               elbowGraphics.addSphere(0.04, YoAppearance.White());
               return elbowGraphics;
            case LOWER_ARM:
               Graphics3DObject lowerArmGraphics = new Graphics3DObject();
               double zOffsetLowerArm = getChildJoint().getJointOffset().getZ() / 2.0;
               lowerArmGraphics.translate(0.0, 0.0, zOffsetLowerArm);
               lowerArmGraphics.addEllipsoid(0.05, 0.05, Math.abs(zOffsetLowerArm), YoAppearance.DarkGreen());
               return lowerArmGraphics;
            case WRIST:
               Graphics3DObject wristGraphics = new Graphics3DObject();
               wristGraphics.addSphere(0.03, YoAppearance.White());
               return wristGraphics;
            case HAND:
               Graphics3DObject handGraphics = new Graphics3DObject();
               handGraphics.transform(endEffectorTransformToWrist);
               handGraphics.addEllipsoid(0.08, 0.03, 0.06, YoAppearance.Gray());
               handGraphics.addCoordinateSystem(0.25);
               return handGraphics;
            default:
               throw new RuntimeException("Should not get there.");
         }
      }

      public RobotArmJointParameters getChildJoint()
      {
         switch (this)
         {
            case UPPER_SHOULDER: return RobotArmJointParameters.SHOULDER_PITCH;
            case LOWER_SHOULDER: return RobotArmJointParameters.SHOULDER_ROLL;
            case UPPER_ARM:      return RobotArmJointParameters.ELBOW_YAW;
            case ELBOW:          return RobotArmJointParameters.ELBOW_PITCH;
            case LOWER_ARM:      return RobotArmJointParameters.WRIST_YAW;
            case WRIST:          return RobotArmJointParameters.WRIST_PITCH;
            case HAND:           return null;
            default:             throw new RuntimeException("Should not get there.");
         }
      }
   }
}