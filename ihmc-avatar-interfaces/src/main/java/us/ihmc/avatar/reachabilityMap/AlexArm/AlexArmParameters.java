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

    public RigidBodyTransform getEndEffectorTransformToWrist()
    {
        return endEffectorTransformToWrist;
    }

    public enum AlexArmJointParameters
    {
        SHOULDER_YAW, SHOULDER_PITCH, SHOULDER_ROLL, ELBOW_YAW, ELBOW_PITCH, WRIST_YAW, WRIST_ROLL, WRIST_PITCH;

        public static AlexArmJointParameters getRootJoint()
        {
            return SHOULDER_YAW;
        }

        public Vector3D getJointAxis()
        {
            String name = this.toString();
            if (name.endsWith("YAW"))
                return new Vector3D(0.0, 0.0, 1.0);
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
            {
                case SHOULDER_YAW:
                    return new Vector3D(0.0, 0.0, 1.5);
                case SHOULDER_PITCH:
                    return new Vector3D(0.0, 0.0, 0.0);
                case SHOULDER_ROLL:
                    return new Vector3D(0.0, 0.0, 0.0);
                case ELBOW_YAW:
                    return new Vector3D(0.0, 0.0, -0.5);
                case ELBOW_PITCH:
                    return new Vector3D(0.0, 0.0, 0.0);
                case WRIST_YAW:
                    return new Vector3D(0.0, 0.0, -0.5);
                case WRIST_ROLL:
                    return new Vector3D(0.0, 0.0, 0.0);
                case WRIST_PITCH:
                    return new Vector3D(0.0, 0.0, 0.0);
                default:
                    throw new RuntimeException("Should not get there.");
            }
        }

        public double getJointLowerLimit()
        {
            switch (this)
            {
                case SHOULDER_YAW:
                    return -1.22173;
                    //return 0.0;
                case SHOULDER_PITCH:
                    return -Math.PI;
                    //return 0.0;
                case SHOULDER_ROLL:
                    return -0.34907;
                    //return 0.0;
                case ELBOW_YAW:
                    return 0.0;  // Keep 0.0
                case ELBOW_PITCH:
                    return -2.35619;
                case WRIST_YAW:
                    return -2.61799;
                case WRIST_ROLL:
                    return -0.61087;
                case WRIST_PITCH:
                    return -2.61799;

                default:
                    throw new RuntimeException("Should not get there.");
            }
        }

        public double getJointUpperLimit()
        {
            switch (this)
            {
                case SHOULDER_YAW:
                    return 1.91986;
                    //return 0.0;
                case SHOULDER_PITCH:
                    return 1.22173;
                    //return 0.0;
                case SHOULDER_ROLL:
                    return 2.79253;
                    //return 0.0;
                case ELBOW_YAW:
                    return 1.0; // Keep 0.0
                case ELBOW_PITCH:
                    return 0.17453;
                case WRIST_YAW:
                    return 2.61799;
                case WRIST_ROLL:
                    return 1.83260;
                case WRIST_PITCH:
                    return 2.61799;
                default:
                    throw new RuntimeException("Should not get there.");
            }
        }

        public AlexArmJointParameters getParentJoint()
        {
            switch (this)
            {
                case SHOULDER_YAW:
                    return null;
                case SHOULDER_PITCH:
                    return SHOULDER_YAW;
                case SHOULDER_ROLL:
                    return SHOULDER_PITCH;
                case ELBOW_YAW:
                    return SHOULDER_ROLL;
                case ELBOW_PITCH:
                    return ELBOW_YAW;
                case WRIST_YAW:
                    return ELBOW_PITCH;
                case WRIST_ROLL:
                    return WRIST_YAW;
                case WRIST_PITCH:
                    return WRIST_ROLL;
                default:
                    throw new RuntimeException("Should not get there.");
            }
        }

        public AlexArmJointParameters getChildJoint()
        {
            switch (this)
            {
                case SHOULDER_YAW:
                    return SHOULDER_PITCH;
                case SHOULDER_PITCH:
                    return SHOULDER_ROLL;
                case SHOULDER_ROLL:
                    return ELBOW_YAW;
                case ELBOW_YAW:
                    return ELBOW_PITCH;
                case ELBOW_PITCH:
                    return WRIST_YAW;
                case WRIST_YAW:
                    return WRIST_ROLL;
                case WRIST_ROLL:
                    return WRIST_PITCH;
                case WRIST_PITCH:
                    return null;
                default:
                    throw new RuntimeException("Should not get there.");
            }
        }

        public AlexArmLinkParameters getAttachedLink()
        {
            switch (this)
            {
                case SHOULDER_YAW:
                    return AlexArmLinkParameters.UPPER_SHOULDER;
                case SHOULDER_PITCH:
                    return AlexArmLinkParameters.LOWER_SHOULDER;
                case SHOULDER_ROLL:
                    return AlexArmLinkParameters.UPPER_ARM;
                case ELBOW_YAW:
                    return AlexArmLinkParameters.ELBOW;
                case ELBOW_PITCH:
                    return AlexArmLinkParameters.LOWER_ARM;
                case WRIST_YAW:
                    return AlexArmLinkParameters.UPPER_WRIST;
                case WRIST_ROLL:
                    return AlexArmLinkParameters.LOWER_WRIST;
                case WRIST_PITCH:
                    return AlexArmLinkParameters.HAND;
                default:
                    throw new RuntimeException("Should not get there.");
            }
        }
    }

    public enum AlexArmLinkParameters
    {
        UPPER_SHOULDER, LOWER_SHOULDER, UPPER_ARM, ELBOW, LOWER_ARM, UPPER_WRIST, LOWER_WRIST, HAND;

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
                case UPPER_SHOULDER:
                    VisualDefinitionFactory upperShoulderGraphics = new VisualDefinitionFactory();
                    upperShoulderGraphics.addSphere(0.05, ColorDefinitions.White());
                    return upperShoulderGraphics.getVisualDefinitions();
                case LOWER_SHOULDER:
                    VisualDefinitionFactory lowerShoulderGraphics = new VisualDefinitionFactory();
                    lowerShoulderGraphics.addSphere(0.05, ColorDefinitions.White());
                    return lowerShoulderGraphics.getVisualDefinitions();
                case UPPER_ARM:
                    VisualDefinitionFactory upperArmGraphics = new VisualDefinitionFactory();
                    double zOffsetUpperArm = getChildJoint().getJointOffset().getZ() / 2.0;
                    upperArmGraphics.appendTranslation(0.0, 0.0, zOffsetUpperArm);
                    upperArmGraphics.addEllipsoid(0.05, 0.05, Math.abs(zOffsetUpperArm), ColorDefinitions.DarkBlue());
                    return upperArmGraphics.getVisualDefinitions();
                case ELBOW:
                    VisualDefinitionFactory elbowGraphics = new VisualDefinitionFactory();
                    elbowGraphics.addSphere(0.04, ColorDefinitions.White());
                    return elbowGraphics.getVisualDefinitions();
                case LOWER_ARM:
                    VisualDefinitionFactory lowerArmGraphics = new VisualDefinitionFactory();
                    double zOffsetLowerArm = getChildJoint().getJointOffset().getZ() / 2.0;
                    lowerArmGraphics.appendTranslation(0.0, 0.0, zOffsetLowerArm);
                    lowerArmGraphics.addEllipsoid(0.05, 0.05, Math.abs(zOffsetLowerArm), ColorDefinitions.DarkGreen());
                    return lowerArmGraphics.getVisualDefinitions();
                case UPPER_WRIST:
                    VisualDefinitionFactory upperWristGraphics = new VisualDefinitionFactory();
                    upperWristGraphics.addSphere(0.03, ColorDefinitions.White());
                    return upperWristGraphics.getVisualDefinitions();
                case LOWER_WRIST:
                    VisualDefinitionFactory lowerWristGraphics = new VisualDefinitionFactory();
                    lowerWristGraphics.addSphere(0.03, ColorDefinitions.White());
                    return lowerWristGraphics.getVisualDefinitions();
                case HAND:
                    VisualDefinitionFactory handGraphics = new VisualDefinitionFactory();
                    handGraphics.appendTransform(endEffectorTransformToWrist);
                    handGraphics.addEllipsoid(0.08, 0.03, 0.06, ColorDefinitions.Gray());
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
                    return AlexArmJointParameters.SHOULDER_PITCH;
                case LOWER_SHOULDER:
                    return AlexArmJointParameters.SHOULDER_ROLL;
                case UPPER_ARM:
                    return AlexArmJointParameters.ELBOW_YAW;
                case ELBOW:
                    return AlexArmJointParameters.ELBOW_PITCH;
                case LOWER_ARM:
                    return AlexArmJointParameters.WRIST_YAW;
                case UPPER_WRIST:
                    return AlexArmJointParameters.WRIST_ROLL;
                case LOWER_WRIST:
                    return AlexArmJointParameters.WRIST_PITCH;
                case HAND:
                    return null;
                default:
                    throw new RuntimeException("Should not get there.");
            }
        }
    }
}