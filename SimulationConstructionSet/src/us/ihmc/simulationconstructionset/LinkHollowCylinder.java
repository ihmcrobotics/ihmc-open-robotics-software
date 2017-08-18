package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBodyInertia;


public class LinkHollowCylinder extends Link
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

   public LinkHollowCylinder(String name, Vector3D cylinderZAxisInWorld, double length, double radius, AppearanceDefinition color)
   {
      this(name, cylinderZAxisInWorld, computeMassOfHollowCylinder(length, radius), length, radius, color);
   }
   
   public LinkHollowCylinder(String name, Vector3D cylinderZAxisInWorld, double length, double radius, Vector3D parentJointOffsetFromCoM,
	         AppearanceDefinition color)
   {
	   this(name, cylinderZAxisInWorld, computeMassOfHollowCylinder(length, radius), length, radius, parentJointOffsetFromCoM, color);
   }
   
   public static double computeMassOfHollowCylinder(double length, double radius)
   {
      double innerRadius = radius - wallThicknessPercentOfRadius * radius;
      double materialVolume = Math.PI*length*((radius*radius) - (innerRadius * innerRadius));
      return materialVolume * aluminumDensityKgPerCubicM;
   }
   
   public LinkHollowCylinder(String name, Vector3D cylinderZAxisInWorld, double mass, double length, double radius, AppearanceDefinition color)
   {
      this(name, cylinderZAxisInWorld, mass, length, radius, attachParentJointToDistalEndOfCylinder(cylinderZAxisInWorld, length), color);
   }

   private static Vector3D attachParentJointToDistalEndOfCylinder(Vector3D cylinderZAxisInWorld, double length)
   {
      Vector3D parentJointOffsetFromCoM = new Vector3D(cylinderZAxisInWorld);
      parentJointOffsetFromCoM.normalize();
      parentJointOffsetFromCoM.scale(-length / 2.0);

      return parentJointOffsetFromCoM;
   }

   public LinkHollowCylinder(String name, Vector3D cylinderZAxisInWorld, double mass, double length, double radius, Vector3D parentJointOffsetFromCoM,
         AppearanceDefinition color)
   {
      super(name);

      cylinderGeometry = createCylinderGeometryWithParentJointPoint(length, radius, color, cylinderZAxisInWorld, parentJointOffsetFromCoM);
      boneGeometry = createBoneGeometryWithParentJointPoint(length, radius, color, cylinderZAxisInWorld, parentJointOffsetFromCoM);

      FrameVector cylinderZAxisExpressedInWorld = new FrameVector(world, cylinderZAxisInWorld);
      this.cylinderReferenceFrame = ReferenceFrame.constructReferenceFrameFromPointAndZAxis(name, new FramePoint(world), cylinderZAxisExpressedInWorld);

      comOffset.set(parentJointOffsetFromCoM);

      Matrix3D moiInCylinderFrame = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(mass, radius, length, Axis.Z);

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

         Matrix3D Q = new Matrix3D(eigenvectors);

         Matrix3D moiInWorldFrame = new Matrix3D();

         moiInCylinderFrame.multiplyTransposeOther(Q);
         moiInWorldFrame.preMultiply(Q);

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
      this.addEllipsoidFromMassProperties(color);
   }

   private Graphics3DObject createCylinderGeometryWithParentJointPoint(double length, double radius, AppearanceDefinition color, Vector3D cylinderZAxisInWorld,
         Vector3D parentJointOffsetFromCoM)
   {
      Graphics3DObject ret = new Graphics3DObject();
      
      addGraphics3DJointSphere(ret, 1.5 * radius);
      addGraphics3DCylinder(ret, length, radius, color, cylinderZAxisInWorld, parentJointOffsetFromCoM);

      return ret;
   }
   
   private void addGraphics3DJointSphere(Graphics3DObject graphicsToAddTo, double radius)
   {
	   graphicsToAddTo.addSphere(1.5*radius, YoAppearance.BlackMetalMaterial());
   }
   
   private void addGraphics3DCylinder(Graphics3DObject graphicsToAddTo, double length, double radius, AppearanceDefinition color, Vector3D cylinderZAxisInWorld, Vector3D cylinderCenterInWorld)
	  {
	   	  graphicsToAddTo.identity();
	   	  graphicsToAddTo.translate(cylinderCenterInWorld);

		  AxisAngle cylinderRotationFromZup = EuclidGeometryTools.axisAngleFromZUpToVector3D(cylinderZAxisInWorld);
		  graphicsToAddTo.rotate(cylinderRotationFromZup);
		  graphicsToAddTo.translate(0.0, 0.0, -length / 2.0);
		  graphicsToAddTo.addCylinder(length, radius, color);
	  }

   private Graphics3DObject createBoneGeometryWithParentJointPoint(double length, double radius, AppearanceDefinition color, Vector3D cylinderZAxisInWorld,
         Vector3D parentJointOffsetFromCoM)
   {
      Graphics3DObject ret = new Graphics3DObject();

      addGraphics3DJointSphere(ret, 1.5 * radius);
      
      ret.translate(parentJointOffsetFromCoM);

	  AxisAngle cylinderRotationFromZup = EuclidGeometryTools.axisAngleFromZUpToVector3D(cylinderZAxisInWorld);
      ret.rotate(cylinderRotationFromZup);
      ret.translate(0.0, 0.0, -0.5 * length);
      ret.addCone(1.0 * length, radius, color);
      ret.translate(0.0, 0.0, 0.5 * length);

      Vector3D rotationAxis = new Vector3D(cylinderRotationFromZup.getX(), cylinderRotationFromZup.getY(), cylinderRotationFromZup.getZ());
      AxisAngle another180DegreeRotation = new AxisAngle(rotationAxis, Math.PI);
      ret.rotate(another180DegreeRotation);
      ret.translate(0.0, 0.0, -0.5 * length);
      ret.addCone(1.0 * length, radius, color);
      ret.translate(0.0, 0.0, 0.5 * length);

      return ret;
   }
   
   
   public void addParentJointAxisGeometry(double jointGraphicCylinderLength, AppearanceDefinition color)
	{
	   double jointGraphicCylinderRadius = jointGraphicCylinderLength / 50.0;
	      
	   Vector3D jointAxis = new Vector3D();
	   parentJoint.getJointAxis(jointAxis);
	   
	   	addGraphics3DCylinder(getLinkGraphics(), jointGraphicCylinderLength, jointGraphicCylinderRadius, color, jointAxis, new Vector3D());
	}
   
   public void addCylinderLinkGraphicsFromMassProperties()
   {
      getLinkGraphics().combine(cylinderGeometry, new Vector3D());
   }

   public void useCylinderLinkGraphicsFromMassProperties()
   {
      getLinkGraphics().getGraphics3DInstructions().clear();
      getLinkGraphics().combine(cylinderGeometry, new Vector3D());
   }
   
   public void addBoneLinkGraphicsFromMassProperties()
   {
      getLinkGraphics().combine(boneGeometry, new Vector3D());
   }

   public void useBoneLinkGraphicsFromMassProperties()
   {
      getLinkGraphics().getGraphics3DInstructions().clear();
      getLinkGraphics().combine(boneGeometry, new Vector3D());
   }
   
   private final Vector3D linkVectorCopy = new Vector3D();

   private void addCylinderLinkGraphics(Vector3D linkStartPointOffsetFromParentJoint, double linkThickness, Vector3D linkVectorInWorld,
         AppearanceDefinition color)
   {
      addCylinderLinkGraphics(linkStartPointOffsetFromParentJoint, linkThickness, linkVectorInWorld, color, 0.5 * linkThickness);
   }


   private void addCylinderLinkGraphics(Vector3D linkStartPointOffsetFromParentJoint, double linkThickness, Vector3D linkVectorInWorld,
         AppearanceDefinition color, double trimBothEndsByThisMuch)
   {
      linkVectorCopy.set(linkVectorInWorld);

      Graphics3DObject linkGraphics = getLinkGraphics();

      linkGraphics.identity();
      linkGraphics.translate(linkStartPointOffsetFromParentJoint);
      linkVectorCopy.sub(linkStartPointOffsetFromParentJoint);
      linkGraphics.rotate(EuclidGeometryTools.axisAngleFromZUpToVector3D(linkVectorCopy));
      linkGraphics.translate(0.0, 0.0, trimBothEndsByThisMuch);
      linkGraphics.addCylinder(linkVectorCopy.length() - trimBothEndsByThisMuch, linkThickness, color);
   }

   private void addCylinderLinkGraphics(Vector3D linkStartPointOffsetFromParentJoint, double linkThickness, Joint jointWhereLinkEnds,
         AppearanceDefinition color, double trimBothEndsByThisMuch)
   {
      jointWhereLinkEnds.getOffset(linkVectorCopy);

      Graphics3DObject linkGraphics = getLinkGraphics();

      linkGraphics.identity();
      linkGraphics.translate(linkStartPointOffsetFromParentJoint);
      linkVectorCopy.sub(linkStartPointOffsetFromParentJoint);
      linkGraphics.rotate(EuclidGeometryTools.axisAngleFromZUpToVector3D(linkVectorCopy));
      linkGraphics.translate(0.0, 0.0, trimBothEndsByThisMuch);
      linkGraphics.addCylinder(linkVectorCopy.length() - trimBothEndsByThisMuch, linkThickness, color);
   }

   public void addCylinderLinkGraphics(Point3D cylinderStart, Point3D cylinderEnd, double linkThickness, AppearanceDefinition color)
   {
	   addCylinderLinkGraphics(cylinderStart, cylinderEnd, linkThickness, color, 0.5 * linkThickness);
   }

   public void addCylinderLinkGraphics(Point3D cylinderStart, Point3D cylinderEnd, double linkThickness, AppearanceDefinition color,
         double trimBothEndsByThisMuch)
   {
      Graphics3DObject linkGraphics = getLinkGraphics();

      linkGraphics.identity();

      Vector3D linkVector = new Vector3D();
      linkVector.sub(cylinderEnd, cylinderStart);

      linkGraphics.translate(cylinderStart);
      linkGraphics.rotate(EuclidGeometryTools.axisAngleFromZUpToVector3D(linkVector));
      linkGraphics.translate(0.0, 0.0, trimBothEndsByThisMuch);
      linkGraphics.addCylinder(linkVector.length() - trimBothEndsByThisMuch, linkThickness, color);
   }
}
