package us.ihmc.exampleSimulations.fourBarLinkage;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.SingularValueDecomposition3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.robotics.robotDescription.InertiaTools;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.robot.LoopClosureDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;

/**
 * <pre>
 *     _
 *     |
 *     O shoulderJoint
 *     |
 *     | upperarm
 *     |
 * D O----O A
 *    \  /
 *     \/
 *     /\
 *    /  \
 * B O----O C
 *     |
 *     | forearm
 *     |
 *     EE
 * </pre>
 * 
 * <pre>
 *                 A     C
 *                 O     O
 *                 |\   /|
 *     shoulder    | \ / |
 * |----O----------|  X  |--------- end-effector
 *                 | / \ |
 *                 |/   \|
 *                 O     O
 *                 D     B
 * </pre>
 */
public class CrossFourBarLinkageRobotDefinition extends RobotDefinition
{
   public static final boolean HAS_SHOULDER_JOINT = true;
   public static final boolean HAS_WRIST_JOINT = true;

   private static final MaterialDefinition blackMetalMaterial = new MaterialDefinition(new ColorDefinition(0.45, 0.45, 0.45),
                                                                                       new ColorDefinition(0.95, 0.95, 0.95),
                                                                                       new ColorDefinition(0.4, 0.4, 0.4),
                                                                                       null,
                                                                                       2.0);
   private static final MaterialDefinition grayMaterial = new MaterialDefinition(ColorDefinitions.Gray());
   private static final MaterialDefinition aliceBlueMaterial = new MaterialDefinition(ColorDefinitions.AliceBlue());
   private static final MaterialDefinition blueVioletMaterial = new MaterialDefinition(ColorDefinitions.BlueViolet());
   private static final MaterialDefinition cornflowerBlueMaterial = new MaterialDefinition(ColorDefinitions.CornflowerBlue());


   private final String shoulderJointName = "shoulder";
   private final String jointAName = "fourBarA";
   private final String jointBName = "fourBarB";
   private final String jointCName = "fourBarC";
   private final String jointDName = "fourBarD";
   private final String wristJointName = "wrist";

   public CrossFourBarLinkageRobotDefinition()
   {
      super("CrossFourBarLinkageRobot");

      double lengthAB = 0.2 * Math.sqrt(2.0);
      double lengthBC = 0.2;
      double lengthCD = 0.2 * Math.sqrt(2.0);
      double lengthDA = 0.2;
      double upperarmLength = 0.5;
      double forearmLength = 0.5;
      UnitVector3D axisAB = new UnitVector3D(+0.1, 0.0, -0.1);
      UnitVector3D axisCD = new UnitVector3D(-0.1, 0.0, -0.1);

      Vector3D rootJointOffset = new Vector3D(0.0, 0.0, 0.0);
      Vector3D jointAOffset = new Vector3D(upperarmLength, 0.0, 0.5 * lengthDA);
      Vector3D jointBOffset = new Vector3D();
      jointBOffset.setAndScale(lengthAB, axisAB);
      Vector3D jointCOffsetFromD = new Vector3D();
      jointCOffsetFromD.setAndScale(-lengthCD, axisCD);
      Vector3D jointCOffsetFromB = new Vector3D(0.0, 0.0, lengthBC);
      Vector3D jointDOffset = new Vector3D(upperarmLength, 0.0, -0.5 * lengthDA);
      Vector3D wristJointOffset = new Vector3D(forearmLength, 0.0, 0.5 * lengthBC);

      RevoluteJointDefinition shoulderJoint = null;
      if (HAS_SHOULDER_JOINT)
      {
         shoulderJoint = new RevoluteJointDefinition(shoulderJointName, rootJointOffset, Axis3D.Y);
         shoulderJoint.setPositionLimits(-Math.PI, Math.PI);
         shoulderJoint.setGainsSoftLimitStop(100.0, 10.0);
      }

      RevoluteJointDefinition fourBarJointA = new RevoluteJointDefinition(jointAName, jointAOffset, Axis3D.Y);
      RevoluteJointDefinition fourBarJointB = new RevoluteJointDefinition(jointBName, jointBOffset, Axis3D.Y);
      RevoluteJointDefinition fourBarJointC = new RevoluteJointDefinition(jointCName, jointCOffsetFromD, Axis3D.Y);
      fourBarJointC.setLoopClosureDefinition(new LoopClosureDefinition());
      fourBarJointC.getLoopClosureDefinition().setKpSoftConstraint(1.0e6);
      fourBarJointC.getLoopClosureDefinition().setKdSoftConstraint(5000.0);
      fourBarJointC.getLoopClosureDefinition().setOffsetFromSuccessorParent(jointCOffsetFromB);
      RevoluteJointDefinition fourBarJointD = new RevoluteJointDefinition(jointDName, jointDOffset, Axis3D.Y);

      RevoluteJointDefinition wristJoint = null;

      if (HAS_WRIST_JOINT)
      {
         wristJoint = new RevoluteJointDefinition(wristJointName, wristJointOffset, Axis3D.Y);
         wristJoint.setPositionLimits(-Math.PI, Math.PI);
         wristJoint.setGainsSoftLimitStop(100.0, 10.0);
      }

      Vector3D offsetAB = new Vector3D();
      offsetAB.setAndScale(0.5 * lengthAB, axisAB);
      RigidBodyDefinition linkAB = newCylinderRigidBodyDefinition("fourBarAB", lengthAB, 0.01, 0.5, axisAB, offsetAB, blackMetalMaterial, false);

      Vector3D offsetCD = new Vector3D();
      offsetCD.setAndScale(-0.5 * lengthCD, axisCD);
      RigidBodyDefinition linkCD = newCylinderRigidBodyDefinition("fourBarCD", lengthCD, 0.01, 0.5, axisCD, offsetCD, blackMetalMaterial, false);

      Vector3D offsetDA = new Vector3D(upperarmLength, 0.0, 0.0);
      RigidBodyDefinition linkDA = newCylinderRigidBodyDefinition("fourBarDA", lengthDA, 0.015, 0.1, Axis3D.Z, offsetDA, grayMaterial, true);

      Vector3D offsetBC = new Vector3D(0.0, 0.0, 0.5 * lengthBC);
      RigidBodyDefinition linkBC = newCylinderRigidBodyDefinition("fourBarBC", lengthBC, 0.015, 0.1, Axis3D.Z, offsetBC, grayMaterial, true);

      RigidBodyDefinition upperarm = null;

      if (HAS_SHOULDER_JOINT)
      {
         Vector3D upperarmOffset = new Vector3D(0.5 * upperarmLength, 0.0, 0.0);
         upperarm = newCylinderRigidBodyDefinition("upperarm", upperarmLength, 0.025, 1.0, Axis3D.X, upperarmOffset, aliceBlueMaterial, true);
         upperarm = RobotDefinition.merge("upperarm", linkDA, upperarm);
      }
      Vector3D forearmOffset = new Vector3D(0.5 * forearmLength, 0.0, 0.5 * lengthBC);
      RigidBodyDefinition forearm = newCylinderRigidBodyDefinition("forearm", forearmLength, 0.025, 1.0, Axis3D.X, forearmOffset, blueVioletMaterial, true);
      forearm = RobotDefinition.merge("forearm", linkBC, forearm);

      RigidBodyDefinition hand = null;

      if (HAS_WRIST_JOINT)
      {
         double handLength = 0.2;
         Vector3D handOffset = new Vector3D(0.5 * handLength, 0.0, 0.0);
         hand = newCylinderRigidBodyDefinition("hand", handLength, 0.0125, 1.0, Axis3D.X, handOffset, cornflowerBlueMaterial, false);
      }

      RigidBodyDefinition rootBody = new RigidBodyDefinition("rootBody");

      if (HAS_SHOULDER_JOINT)
         shoulderJoint.setSuccessor(upperarm);
      fourBarJointA.setSuccessor(linkAB);
      fourBarJointB.setSuccessor(forearm);
      fourBarJointC.setLoopClosureSuccessor(forearm);
      fourBarJointD.setSuccessor(linkCD);
      if (HAS_WRIST_JOINT)
         wristJoint.setSuccessor(hand);

      if (HAS_SHOULDER_JOINT)
      {
         upperarm.addChildJoint(fourBarJointA);
         upperarm.addChildJoint(fourBarJointD);
         rootBody.addChildJoint(shoulderJoint);
      }
      else
      {
         rootBody.addChildJoint(fourBarJointA);
         rootBody.addChildJoint(fourBarJointD);
      }

      linkAB.addChildJoint(fourBarJointB);
      linkCD.addChildJoint(fourBarJointC);

      if (HAS_WRIST_JOINT)
      {
         forearm.addChildJoint(wristJoint);
      }

      setRootBodyDefinition(rootBody);
   }

   public String getShoulderJointName()
   {
      if (HAS_SHOULDER_JOINT)
         return shoulderJointName;
      else
         return null;
   }

   public String getJointAName()
   {
      return jointAName;
   }

   public String getJointBName()
   {
      return jointBName;
   }

   public String getJointCName()
   {
      return jointCName;
   }

   public String getJointDName()
   {
      return jointDName;
   }

   public String getWristJointName()
   {
      if (HAS_WRIST_JOINT)
         return wristJointName;
      else
         return null;
   }

   private static RigidBodyDefinition newCylinderRigidBodyDefinition(String name,
                                                                     double length,
                                                                     double radius,
                                                                     double mass,
                                                                     Vector3DReadOnly axis,
                                                                     Vector3DReadOnly comOffset,
                                                                     MaterialDefinition material,
                                                                     boolean showMassProperties)
   {
      RigidBodyDefinition rigidBodyDefinition = new RigidBodyDefinition(name);
      rigidBodyDefinition.setMass(mass);
      rigidBodyDefinition.setMomentOfInertia(MomentOfInertiaFactory.solidCylinder(mass, radius, length, axis));
      rigidBodyDefinition.setCenterOfMassOffset(comOffset);

      VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
      visualDefinitionFactory.appendTranslation(comOffset);
      visualDefinitionFactory.appendRotation(EuclidGeometryTools.axisAngleFromZUpToVector3D(axis));
      visualDefinitionFactory.appendTranslation(0.0, 0.0, -0.5 * length);
      visualDefinitionFactory.addGeometryDefinition(new Cylinder3DDefinition(length, radius, true, 64), material);

      if (showMassProperties)
      {
         SingularValueDecomposition3D svd = new SingularValueDecomposition3D();
         if (svd.decompose(rigidBodyDefinition.getMomentOfInertia()))
         {
            Vector3D radii = InertiaTools.getInertiaEllipsoidRadii(svd.getW(), mass);
            visualDefinitionFactory.identity();
            visualDefinitionFactory.appendTranslation(comOffset);
            visualDefinitionFactory.appendRotation(svd.getU());
            visualDefinitionFactory.addEllipsoid(radii.getX(),
                                                 radii.getY(),
                                                 radii.getZ(),
                                                 ColorDefinitions.LightGreen().derive(0, 1, 1, 0.5));
         }
      }
      rigidBodyDefinition.getVisualDefinitions().addAll(visualDefinitionFactory.getVisualDefinitions());

      return rigidBodyDefinition;
   }
}
