package us.ihmc.simulationconstructionset;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBodyInertia;


public class LinkSolidCylinder extends Link
{

   /**
    * A solid cylindrical link with automatically generated mass and moment of inertia properties
    */
   private static final long serialVersionUID = 6789282991137530985L;
   
   public static final double aluminumDensityKgPerCubicM = 2800.0;
   private static final double wallThicknessPercentOfRadius = 0.2; // [0.1]

   protected final ReferenceFrame world = ReferenceFrame.getWorldFrame();
   protected final ReferenceFrame cylinderReferenceFrame;

   private final Graphics3DObject cylinderGeometry;
   private final Graphics3DObject boneGeometry;
   private final Graphics3DObject jointAxisGeometry;

   public LinkSolidCylinder(String name, Vector3d cylinderZAxisInWorld, double length, double radius, AppearanceDefinition color)
   {
      this(name, cylinderZAxisInWorld, computeMassOfHollowCylinder(length, radius), length, radius, color);
   }
   
   public static double computeMassOfHollowCylinder(double length, double radius)
   {
      double innerRadius = radius - wallThicknessPercentOfRadius * radius;
      double materialVolume = Math.PI*length*((radius*radius) - (innerRadius * innerRadius));
      return materialVolume * aluminumDensityKgPerCubicM;
   }
   
   public LinkSolidCylinder(String name, Vector3d cylinderZAxisInWorld, double mass, double length, double radius, AppearanceDefinition color)
   {
      this(name, cylinderZAxisInWorld, mass, length, radius, attachParentJointToDistalEndOfCylinder(cylinderZAxisInWorld, length), color);
   }

   private static Vector3d attachParentJointToDistalEndOfCylinder(Vector3d cylinderZAxisInWorld, double length)
   {
      Vector3d parentJointOffsetFromCoM = new Vector3d(cylinderZAxisInWorld);
      parentJointOffsetFromCoM.normalize();
      parentJointOffsetFromCoM.scale(-length / 2.0);

      return parentJointOffsetFromCoM;
   }

   public LinkSolidCylinder(String name, Vector3d cylinderZAxisInWorld, double mass, double length, double radius, Vector3d parentJointOffsetFromCoM,
         AppearanceDefinition color)
   {
      super(name);

      cylinderGeometry = createCylinderGeometry(length, radius, color, cylinderZAxisInWorld, parentJointOffsetFromCoM);
      boneGeometry = createBoneGeometry(length, radius, color, cylinderZAxisInWorld, parentJointOffsetFromCoM);
      jointAxisGeometry = createJointAxisGeometry(length, 0.1 * radius, color, cylinderZAxisInWorld, parentJointOffsetFromCoM);

      FrameVector cylinderZAxisExpressedInWorld = new FrameVector(world, cylinderZAxisInWorld);
      this.cylinderReferenceFrame = ReferenceFrame.constructReferenceFrameFromPointAndZAxis(name, new FramePoint(world), cylinderZAxisExpressedInWorld);

      comOffset.set(parentJointOffsetFromCoM);

      Matrix3d moiInCylinderFrame = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(mass, radius, length, Axis.Z);

      Boolean computeMoiInWorldInternally = false;

      if (computeMoiInWorldInternally)
      {
         FrameVector linkXAxis = new FrameVector(cylinderReferenceFrame, 1.0, 0.0, 0.0);
         FrameVector linkYAxis = new FrameVector(cylinderReferenceFrame, 0.0, 1.0, 0.0);
         FrameVector linkZAxis = new FrameVector(cylinderReferenceFrame, 0.0, 0.0, 1.0);

         linkXAxis.changeFrame(world);
         linkYAxis.changeFrame(world);
         linkZAxis.changeFrame(world);

         double[] eigenvectors = new double[] { linkXAxis.getX(), linkYAxis.getX(), linkZAxis.getX(), linkXAxis.getY(), linkYAxis.getY(), linkZAxis.getY(),
               linkXAxis.getZ(), linkYAxis.getZ(), linkZAxis.getZ() };

         Matrix3d Q = new Matrix3d(eigenvectors);

         Matrix3d moiInWorldFrame = new Matrix3d();

         moiInCylinderFrame.mulTransposeRight(moiInCylinderFrame, Q);
         moiInWorldFrame.mul(Q, moiInCylinderFrame);

         this.setMass(mass);
         setMomentOfInertia(moiInWorldFrame);
         setComOffset(comOffset);
      }
      else
      {
         RigidBodyInertia inertia = new RigidBodyInertia(cylinderReferenceFrame, moiInCylinderFrame, mass);
         inertia.changeFrame(ReferenceFrame.getWorldFrame());

         this.setMass(mass);
         setMomentOfInertia(inertia.getMassMomentOfInertiaPartCopy());
         setComOffset(comOffset);
      }

      this.addCoordinateSystemToCOM(length / 10.0);
      this.addEllipsoidFromMassProperties2(color);
   }

   private Graphics3DObject createCylinderGeometry(double length, double radius, AppearanceDefinition color, Vector3d cylinderZAxisInWorld,
         Vector3d parentJointOffsetFromCoM)
   {
      Graphics3DObject ret = new Graphics3DObject();
      ret.addSphere(1.5*radius, YoAppearance.BlackMetalMaterial());

      ret.identity();
      ret.translate(parentJointOffsetFromCoM);

      AxisAngle4d cylinderRotationFromZup;
      if (parentJointOffsetFromCoM.length() > 0.0)
         cylinderRotationFromZup = GeometryTools.getRotationBasedOnNormal(parentJointOffsetFromCoM);
      else
         cylinderRotationFromZup = GeometryTools.getRotationBasedOnNormal(cylinderZAxisInWorld);
      ret.rotate(cylinderRotationFromZup);
      ret.translate(0.0, 0.0, -length / 2.0);
      ret.addCylinder(length, radius, color);

      return ret;
   }

   private Graphics3DObject createBoneGeometry(double length, double radius, AppearanceDefinition color, Vector3d cylinderZAxisInWorld,
         Vector3d parentJointOffsetFromCoM)
   {
      Graphics3DObject ret = new Graphics3DObject();

      ret.identity();
      ret.addSphere(1.5*radius, YoAppearance.BlackMetalMaterial());
      ret.translate(parentJointOffsetFromCoM);

      AxisAngle4d cylinderRotationFromZup;
      if (parentJointOffsetFromCoM.length() > 0.0)
         cylinderRotationFromZup = GeometryTools.getRotationBasedOnNormal(parentJointOffsetFromCoM);
      else
         cylinderRotationFromZup = GeometryTools.getRotationBasedOnNormal(cylinderZAxisInWorld);
      ret.rotate(cylinderRotationFromZup);
      ret.translate(0.0, 0.0, -0.5 * length);
      ret.addCone(1.0 * length, radius, color);
      ret.translate(0.0, 0.0, 0.5 * length);

      Vector3d rotationAxis = new Vector3d(cylinderRotationFromZup.x, cylinderRotationFromZup.y, cylinderRotationFromZup.z);
      AxisAngle4d another180DegreeRotation = new AxisAngle4d(rotationAxis, Math.PI);
      ret.rotate(another180DegreeRotation);
      ret.translate(0.0, 0.0, -0.5 * length);
      ret.addCone(1.0 * length, radius, color);
      ret.translate(0.0, 0.0, 0.5 * length);

      return ret;
   }
   
   private Graphics3DObject createJointAxisGeometry(double length, double radius, AppearanceDefinition color, Vector3d cylinderZAxisInWorld,
         Vector3d parentJointOffsetFromCoM)
   {
      Graphics3DObject ret = new Graphics3DObject();

      ret.identity();
      ret.translate(parentJointOffsetFromCoM);

      AxisAngle4d cylinderRotationFromZup;
      if (parentJointOffsetFromCoM.length() > 0.0)
         cylinderRotationFromZup = GeometryTools.getRotationBasedOnNormal(parentJointOffsetFromCoM);
      else
         cylinderRotationFromZup = GeometryTools.getRotationBasedOnNormal(cylinderZAxisInWorld);
      
      ret.rotate(cylinderRotationFromZup);
      ret.translate(0.0, 0.0, -length / 2.0);
      ret.addCylinder(length, radius, color);

      return ret;
   }

   public void addCylinderLinkGraphicsFromMassProperties()
   {
      getLinkGraphics().combine(cylinderGeometry, new Vector3d());
   }

   public void useCylinderLinkGraphicsFromMassProperties()
   {
      getLinkGraphics().getGraphics3DInstructions().clear();
      getLinkGraphics().combine(cylinderGeometry, new Vector3d());
   }
   
   public void addBoneLinkGraphicsFromMassProperties()
   {
      getLinkGraphics().combine(boneGeometry, new Vector3d());
   }

   public void useBoneLinkGraphicsFromMassProperties()
   {
      getLinkGraphics().getGraphics3DInstructions().clear();
      getLinkGraphics().combine(boneGeometry, new Vector3d());
   }

   public void addJointAxisGraphics()
   {
      getLinkGraphics().combine(jointAxisGeometry, new Vector3d());
   }

   public void useJointAxisGraphicsFromMassProperties()
   {
      getLinkGraphics().getGraphics3DInstructions().clear();
      getLinkGraphics().combine(jointAxisGeometry, new Vector3d());
   }
   
   private final Vector3d linkVectorCopy = new Vector3d();

   private void addCylinderLinkGraphics(Vector3d linkStartPointOffsetFromParentJoint, double linkThickness, Vector3d linkVectorInWorld,
         AppearanceDefinition color)
   {
      addCylinderLinkGraphics(linkStartPointOffsetFromParentJoint, linkThickness, linkVectorInWorld, color, 0.5 * linkThickness);
   }

   public void addCylinderLinkGraphics(Point3d cylinderStart, Point3d cylinderEnd, double linkThickness, AppearanceDefinition color)
   {
      addCylinderLinkGraphics(cylinderStart, cylinderEnd, linkThickness, color, 0.5 * linkThickness);
   }

   private void addCylinderLinkGraphics(Vector3d linkStartPointOffsetFromParentJoint, double linkThickness, Vector3d linkVectorInWorld,
         AppearanceDefinition color, double trimBothEndsByThisMuch)
   {
      linkVectorCopy.set(linkVectorInWorld);

      Graphics3DObject linkGraphics = getLinkGraphics();

      linkGraphics.identity();
      linkGraphics.translate(linkStartPointOffsetFromParentJoint);
      linkVectorCopy.sub(linkStartPointOffsetFromParentJoint);
      linkGraphics.rotate(GeometryTools.getRotationBasedOnNormal(linkVectorCopy));
      linkGraphics.translate(0.0, 0.0, trimBothEndsByThisMuch);
      linkGraphics.addCylinder(linkVectorCopy.length() - trimBothEndsByThisMuch, linkThickness, color);
   }

   private void addCylinderLinkGraphics(Vector3d linkStartPointOffsetFromParentJoint, double linkThickness, Joint jointWhereLinkEnds,
         AppearanceDefinition color, double trimBothEndsByThisMuch)
   {
      jointWhereLinkEnds.getOffset(linkVectorCopy);

      Graphics3DObject linkGraphics = getLinkGraphics();

      linkGraphics.identity();
      linkGraphics.translate(linkStartPointOffsetFromParentJoint);
      linkVectorCopy.sub(linkStartPointOffsetFromParentJoint);
      linkGraphics.rotate(GeometryTools.getRotationBasedOnNormal(linkVectorCopy));
      linkGraphics.translate(0.0, 0.0, trimBothEndsByThisMuch);
      linkGraphics.addCylinder(linkVectorCopy.length() - trimBothEndsByThisMuch, linkThickness, color);
   }

   public void addCylinderLinkGraphics(Point3d cylinderStart, Point3d cylinderEnd, double linkThickness, AppearanceDefinition color,
         double trimBothEndsByThisMuch)
   {
      Graphics3DObject linkGraphics = getLinkGraphics();

      linkGraphics.identity();

      Vector3d linkVector = new Vector3d();
      linkVector.sub(cylinderEnd, cylinderStart);

      linkGraphics.translate(cylinderStart);
      linkGraphics.rotate(GeometryTools.getRotationBasedOnNormal(linkVector));
      linkGraphics.translate(0.0, 0.0, trimBothEndsByThisMuch);
      linkGraphics.addCylinder(linkVector.length() - trimBothEndsByThisMuch, linkThickness, color);
   }
}
