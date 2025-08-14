package us.ihmc.avatar.reachabilityMap.AlexArm;

import java.util.List;

import com.google.common.base.CaseFormat;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;

public class AlexArmParameters

{
    //   private static final RigidBodyTransform endEffectorTransformToWrist = new RigidBodyTransform(new AxisAngle4d(0.0, 1.0, 0.0, Math.PI / 2.0), new Vector3d(0.0, 0.0, -0.08));
    private static final RigidBodyTransform endEffectorTransformToWrist = new RigidBodyTransform();
    private static final RigidBodyTransform palmTransformToWrist = new RigidBodyTransform();

    /*
    public RigidBodyTransform getEndEffectorTransformToWrist()
    {
        return endEffectorTransformToWrist;
    }
    */

    public RigidBodyTransform getPalmTransformToWrist()
    {
        palmTransformToWrist.appendTranslation(0,0,-0.059989);
        return palmTransformToWrist;
    }

    public enum AlexArmJointParameters
    {
        SHOULDER_PITCH, SHOULDER_ROLL, SHOULDER_YAW, ELBOW_PITCH, WRIST_YAW, WRIST_ROLL, HAND_YAW;

        public static AlexArmJointParameters getRootJoint()
        {
            return SHOULDER_PITCH;
        }

        public Vector3D getJointAxis()
        {
            String name = this.toString();
            if (name.endsWith("YAW"))
                return new Vector3D(0.0, 0.0, 1.0);
            else if (name.endsWith("SHOULDER_PITCH"))
                return new Vector3D(0.0, 0.128, 0.0908);
            else if (name.endsWith("PITCH"))
                return new Vector3D(0.0, 1.0, 0.0);
            else if (name.endsWith("ROLL"))
                return new Vector3D(1.0, 0.0, 0.0);
            else
                throw new RuntimeException("Should not get there.");
        }

        public String getJointName(boolean capitalizeFirstLetter)
        {
            return CaseFormat.UPPER_UNDERSCORE.to(capitalizeFirstLetter ? CaseFormat.UPPER_CAMEL : CaseFormat.LOWER_CAMEL, toString());
        }

        public Vector3D getJointOffset()
        {
            switch (this)
            // All offsets measured in mm and scaled down by a 1000

            // Closing Bias Configuration
            {
                case SHOULDER_PITCH:
                    return new Vector3D(0.0, 0.0, 1.5);
                case SHOULDER_ROLL:
                    return new Vector3D(0.0, 0.128, 0.0908);
                case SHOULDER_YAW:
                    return new Vector3D(0.0, 0.035, 0.0);
                case ELBOW_PITCH:
                    return new Vector3D(0.015, 0.0, -0.4);
                case WRIST_YAW:
                    return new Vector3D(-0.015, 0.0, 0.0);
                case WRIST_ROLL:
                    return new Vector3D(0.0, -0.013, -0.25);
                case HAND_YAW:
                    return new Vector3D(0.0, 0.0, 0.0);
                default:
                    throw new RuntimeException("Should not get there.");
            }

        }

        public double getJointLowerLimit()
        {
            switch (this)
            {
                case SHOULDER_PITCH:
                    return -Math.PI;
                case SHOULDER_ROLL:
                    return -0.34907;
                case SHOULDER_YAW:
                    return -1.22173;
                case ELBOW_PITCH:
                    return -2.35619;
                case WRIST_YAW:
                    return -2.61799;
                case WRIST_ROLL:
                    return -1.83260;
                case HAND_YAW:
                    return -2.61799;
                default:
                    throw new RuntimeException("Should not get there.");
            }
        }

        public double getJointUpperLimit()
        {
            switch (this)
            {
                case SHOULDER_PITCH:
                    return 1.22173;
                case SHOULDER_ROLL:
                    return 2.79253;
                case SHOULDER_YAW:
                    return 1.91986;
                case ELBOW_PITCH:
                    return 0.17453;
                case WRIST_YAW:
                    return 2.61799;
                case WRIST_ROLL:
                    return 0.61087;
                case HAND_YAW:
                    return 2.61799;
                default:
                    throw new RuntimeException("Should not get there.");
            }
        }

        public AlexArmJointParameters getParentJoint()
        {
            switch (this)
            {
                case SHOULDER_PITCH:
                    return null;
                case SHOULDER_ROLL:
                    return SHOULDER_PITCH;
                case SHOULDER_YAW:
                    return SHOULDER_ROLL;
                case ELBOW_PITCH:
                    return SHOULDER_YAW;
                case WRIST_YAW:
                    return ELBOW_PITCH;
                case WRIST_ROLL:
                    return WRIST_YAW;
                case HAND_YAW:
                    return WRIST_ROLL;
                default:
                    throw new RuntimeException("Should not get there.");
            }
        }

        public AlexArmJointParameters getChildJoint()
        {
            switch (this)
            {
                case SHOULDER_PITCH :
                    return SHOULDER_ROLL;
                case SHOULDER_ROLL:
                    return SHOULDER_YAW;
                case SHOULDER_YAW:
                    return ELBOW_PITCH;
                case ELBOW_PITCH:
                    return WRIST_YAW;
                case WRIST_YAW:
                    return WRIST_ROLL;
                case WRIST_ROLL:
                    return HAND_YAW;
                case HAND_YAW:
                    return null;
                default:
                    throw new RuntimeException("Should not get there.");
            }
        }

        public AlexArmLinkParameters getAttachedLink()
        {
            switch (this)
            {
                case SHOULDER_PITCH:
                    return AlexArmLinkParameters.UPPER_SHOULDER;
                case SHOULDER_ROLL:
                    return AlexArmLinkParameters.LOWER_SHOULDER;
                case SHOULDER_YAW:
                    return AlexArmLinkParameters.UPPER_ARM;
                case ELBOW_PITCH:
                    return AlexArmLinkParameters.LOWER_ARM;
                case WRIST_YAW:
                    return AlexArmLinkParameters.UPPER_WRIST;
                case WRIST_ROLL:
                    return AlexArmLinkParameters.LOWER_WRIST;
                case HAND_YAW:
                    return AlexArmLinkParameters.HAND;
                default:
                    throw new RuntimeException("Should not get there.");
            }
        }
    }

    public enum AlexArmLinkParameters
    {
        UPPER_SHOULDER, LOWER_SHOULDER, UPPER_ARM, LOWER_ARM, UPPER_WRIST, LOWER_WRIST, HAND;

        public static AlexArmLinkParameters getEndEffector()
        {
            return HAND;
        }

        public String getLinkName()
        {
            return CaseFormat.UPPER_UNDERSCORE.to(CaseFormat.UPPER_CAMEL, toString()) + "Link";
        }

        @Deprecated
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
                case LOWER_ARM:
                    Graphics3DObject lowerArmGraphics = new Graphics3DObject();
                    double zOffsetLowerArm = getChildJoint().getJointOffset().getZ() / 2.0;
                    lowerArmGraphics.translate(0.0, 0.0, zOffsetLowerArm);
                    lowerArmGraphics.addEllipsoid(0.05, 0.05, Math.abs(zOffsetLowerArm), YoAppearance.DarkGreen());
                    return lowerArmGraphics;
                case UPPER_WRIST:
                    Graphics3DObject upperWristGraphics = new Graphics3DObject();
                    upperWristGraphics.addSphere(0.03, YoAppearance.White());
                    return upperWristGraphics;
                case LOWER_WRIST:
                    Graphics3DObject lowerWristGraphics = new Graphics3DObject();
                    lowerWristGraphics.addSphere(0.03, YoAppearance.White());
                    return lowerWristGraphics;
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

        public List<VisualDefinition> getRigidBodyVisuals()
        {
            switch (this)
            {

                case UPPER_SHOULDER: // Child Joint = Shoulder_Roll
                    VisualDefinitionFactory upperShoulderGraphics = new VisualDefinitionFactory();
                    upperShoulderGraphics.addSphere(0.05, ColorDefinitions.Gray());
                    //double xOffsetUpperShoulder = getChildJoint().getJointOffset().getX() / 2.0;
                    double yOffsetUpperShoulder = getChildJoint().getJointOffset().getY() / 2.0;
                    double zOffsetUpperShoulder = getChildJoint().getJointOffset().getZ() / 2.0;
                    upperShoulderGraphics.appendTranslation(0.0, yOffsetUpperShoulder, zOffsetUpperShoulder);
                    upperShoulderGraphics.addEllipsoid(0.05, Math.abs(yOffsetUpperShoulder), Math.abs(zOffsetUpperShoulder), ColorDefinitions.Gray());
                    return upperShoulderGraphics.getVisualDefinitions();
                case LOWER_SHOULDER: // Child Joint = Shoulder_Yaw
                    VisualDefinitionFactory lowerShoulderGraphics = new VisualDefinitionFactory();
                    lowerShoulderGraphics.addSphere(0.05, ColorDefinitions.Gray());
                    //double xOffsetLowerShoulder = getChildJoint().getJointOffset().getX() / 2.0;
                    double yOffsetLowerShoulder = getChildJoint().getJointOffset().getY() / 2.0;
                    //double zOffsetLowerShoulder = getChildJoint().getJointOffset().getZ() / 2.0;
                    lowerShoulderGraphics.appendTranslation(0.0, yOffsetLowerShoulder, 0.0);
                    lowerShoulderGraphics.addEllipsoid(0.05, 0.05, 0.05, ColorDefinitions.Gray());
                    return lowerShoulderGraphics.getVisualDefinitions();
                case UPPER_ARM: // Child Joint = Elbow_Pitch
                    VisualDefinitionFactory upperArmGraphics = new VisualDefinitionFactory();
                    upperArmGraphics.addSphere(0.05, ColorDefinitions.Gray());
                    double xOffsetUpperArm = getChildJoint().getJointOffset().getX() / 2.0;
                    //double yOffsetUpperArm = getChildJoint().getJointOffset().getY() / 2.0;
                    double zOffsetUpperArm = getChildJoint().getJointOffset().getZ() / 2.0;
                    upperArmGraphics.appendTranslation(xOffsetUpperArm, 0.0, zOffsetUpperArm);
                    upperArmGraphics.addEllipsoid(0.05, 0.05, Math.abs(zOffsetUpperArm), ColorDefinitions.White());
                    return upperArmGraphics.getVisualDefinitions();
                case LOWER_ARM: // Child joint = Wrist_Yaw
                    VisualDefinitionFactory lowerArmGraphics = new VisualDefinitionFactory();
                    lowerArmGraphics.addSphere(0.05, ColorDefinitions.Gray());
                    double xOffsetLowerArm = getChildJoint().getJointOffset().getX() / 2.0;
                    //double yOffsetLowerArm = getChildJoint().getJointOffset().getY() / 2.0;
                    //double zOffsetLowerArm = getChildJoint().getJointOffset().getZ() / 2.0;
                    lowerArmGraphics.appendTranslation(xOffsetLowerArm, 0.0, 0.0);
                    //lowerArmGraphics.addEllipsoid(0.05, 0.05, 0.0, ColorDefinitions.DarkBlue());
                    return lowerArmGraphics.getVisualDefinitions();
                case UPPER_WRIST: // Child joint = Wrist_Roll
                    VisualDefinitionFactory upperWristGraphics = new VisualDefinitionFactory();
                    upperWristGraphics.addSphere(0.03, ColorDefinitions.Gray());
                    //double xOffsetUpperWrist = getChildJoint().getJointOffset().getX() / 2.0;
                    double yOffsetUpperWrist = getChildJoint().getJointOffset().getY() / 2.0;
                    double zOffsetUpperWrist = getChildJoint().getJointOffset().getZ() / 2.0;
                    upperWristGraphics.appendTranslation(0.0, yOffsetUpperWrist, zOffsetUpperWrist);
                    upperWristGraphics.addEllipsoid(0.05, 0.05, Math.abs(zOffsetUpperWrist), ColorDefinitions.DarkBlue());
                    return upperWristGraphics.getVisualDefinitions();
                case LOWER_WRIST: // Child joint = Hand_Yaw
                    VisualDefinitionFactory lowerWristGraphics = new VisualDefinitionFactory();
                    lowerWristGraphics.addSphere(0.03, ColorDefinitions.Gray());
                    //double xOffsetLowerWrist = getChildJoint().getJointOffset().getX() / 2.0;
                    //double yOffsetLowerWrist = getChildJoint().getJointOffset().getY() / 2.0;
                    //double zOffsetLowerWrist = getChildJoint().getJointOffset().getZ() / 2.0;
                    lowerWristGraphics.appendTranslation(0.0, 0.0, 0.0);
                    //lowerWristGraphics.addEllipsoid(0.05, 0.05, Math.abs(zOffsetLowerWrist), ColorDefinitions.DarkBlue());
                    return lowerWristGraphics.getVisualDefinitions();
                case HAND:
                    VisualDefinitionFactory handGraphics = new VisualDefinitionFactory();
                    handGraphics.addSphere(0.03, ColorDefinitions.Gray());
                    double zOffsetHand = -0.119978 / 2.0;
                    handGraphics.appendTranslation(0.0, 0.0, zOffsetHand);
                    handGraphics.addEllipsoid(0.03, 0.03, Math.abs(zOffsetHand), ColorDefinitions.Red());
                    handGraphics.appendTranslation(0.0, 0.0, zOffsetHand);
                    handGraphics.appendTransform(endEffectorTransformToWrist);
                    handGraphics.addEllipsoid(0.05, 0.03, 0.06, ColorDefinitions.Red());
                    handGraphics.addCoordinateSystem(0.25);
                    return handGraphics.getVisualDefinitions();
                default:
                    throw new RuntimeException("Should not get there.");
            }
        }

        public AlexArmJointParameters getChildJoint()
        {
            switch (this)
            {
                case UPPER_SHOULDER:
                    return AlexArmJointParameters.SHOULDER_ROLL;
                case LOWER_SHOULDER:
                    return AlexArmJointParameters.SHOULDER_YAW;
                case UPPER_ARM:
                    return AlexArmJointParameters.ELBOW_PITCH;
                case LOWER_ARM:
                    return AlexArmJointParameters.WRIST_YAW;
                case UPPER_WRIST:
                    return AlexArmJointParameters.WRIST_ROLL;
                case LOWER_WRIST:
                    return AlexArmJointParameters.HAND_YAW;
                case HAND:
                    return null;
                default:
                    throw new RuntimeException("Should not get there.");
            }
        }
    }
}