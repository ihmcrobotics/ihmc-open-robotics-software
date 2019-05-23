package us.ihmc.simulationconstructionset;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
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
   public static final double aluminumShearModulusNperSqM = 25.5e9;
   private static final double wallThicknessPercentOfRadius = 0.2; // [0.1]

   
   private final Vector3D cylinderZAxisInWorld;
   protected final ReferenceFrame world = ReferenceFrame.getWorldFrame();
   protected final ReferenceFrame cylinderReferenceFrame;

   private final boolean showJointGraphics;
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
   
   public LinkHollowCylinder(String name, Vector3D cylinderZAxisInWorld, double length, double radius, Vector3D parentJointOffsetFromCoM,
		   AppearanceDefinition color, boolean showJointGraphics)
   {
	   this(name, cylinderZAxisInWorld, computeMassOfHollowCylinder(length, radius), length, radius, parentJointOffsetFromCoM, color, showJointGraphics);
   }
   
   public static double computeMassOfHollowCylinder(double length, double radius)
   {
      double innerRadius = radius - wallThicknessPercentOfRadius * radius;
      double materialVolume = Math.PI*length*((radius*radius) - (innerRadius * innerRadius));
      return materialVolume * aluminumDensityKgPerCubicM;
   }
   
   public static double computeRadius(double length, double mass)
   {
	   double materialVolume = mass / aluminumDensityKgPerCubicM;
	   double ret = Math.sqrt(materialVolume / (Math.PI * length * ( 2 * wallThicknessPercentOfRadius - wallThicknessPercentOfRadius*wallThicknessPercentOfRadius)));
	   return ret;
   }  
   
   public static double computeTheoreticalTorsionalStiffness(double length_m, double radius_m, double wallThickness)
   {
	   double J = 2.0 * Math.PI * radius_m * Math.pow(wallThickness, 3) / 3.0;
	   
	   double ret = aluminumShearModulusNperSqM * J / length_m;
	   
	   return ret;
   }
   
   public LinkHollowCylinder(String name, Vector3D cylinderZAxisInWorld, double mass, double length, double radius, AppearanceDefinition color)
   {
      this(name, cylinderZAxisInWorld, mass, length, radius, attachParentJointToDistalEndOfCylinder(cylinderZAxisInWorld, length), color);
   }

   public static Vector3D attachParentJointToDistalEndOfCylinder(Vector3D cylinderZAxisInWorld, double length)
   {
      Vector3D parentJointOffsetFromCoM = new Vector3D(cylinderZAxisInWorld);
      parentJointOffsetFromCoM.normalize();
      parentJointOffsetFromCoM.scale(-length / 2.0);

      return parentJointOffsetFromCoM;
   }

   public LinkHollowCylinder(String name, Vector3D cylinderZAxisInWorld, double mass, double length, double radius, Vector3D parentJointOffsetFromCoM,
	         AppearanceDefinition color)
   {
	   this(name, cylinderZAxisInWorld, mass, length, radius, parentJointOffsetFromCoM, color, true);
   }
   
   public LinkHollowCylinder(String name, Vector3D cylinderZAxisInWorld, double mass, double length, double radius, Vector3D parentJointOffsetFromCoM,
         AppearanceDefinition color, boolean showJointGraphics)
   {
      super(name);

      this.showJointGraphics = showJointGraphics;
      
      this.cylinderZAxisInWorld = new Vector3D(cylinderZAxisInWorld);
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
         PrintTools.info(this, "Setting Moment of Inertia.  Mass = " + String.format("%.1f", 1000.0 * mass) + " g");
         setMomentOfInertia(inertia.getMassMomentOfInertiaPartCopy());
         setComOffset(comOffset);
      }

      this.addCoordinateSystemToCOM(length / 10.0);
      PrintTools.info(this, "Adding graphics ellipsoid from mass properties.  Mass = "  + String.format("%.1f", 1000.0 * mass) + " g");
      this.addEllipsoidFromMassProperties(color);
   }

   private Graphics3DObject createCylinderGeometryWithParentJointPoint(double length, double radius, AppearanceDefinition color, Vector3D cylinderZAxisInWorld,
         Vector3D parentJointOffsetFromCoM)
   {
      Graphics3DObject ret = new Graphics3DObject();
      
      if (showJointGraphics)
      {
    	  addGraphics3DJointSphere(ret, 1.5 * radius);
      }
      addGraphics3DCylinder(ret, length, radius, color, cylinderZAxisInWorld, parentJointOffsetFromCoM);

      return ret;
   }
   
   private void addGraphics3DJointSphere(Graphics3DObject graphicsToAddTo, double radius)
   {
	   addGraphics3DJointSphere(graphicsToAddTo, radius, YoAppearance.BlackMetalMaterial());
   }
   
   private void addGraphics3DJointSphere(Graphics3DObject graphicsToAddTo, double radius, AppearanceDefinition color)
   {
	   graphicsToAddTo.addSphere(1.5*radius, color);
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

      if (showJointGraphics)
      {
    	  AppearanceDefinition jointColor = YoAppearance.BlackMetalMaterial();
    	  jointColor.setTransparency(color.getTransparency());
    	  
    	  addGraphics3DJointSphere(ret, 0.7 * radius, jointColor);
      }
      
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
   
   public void addModelGraphics(String fileName,  AppearanceDefinition color)
   {
		  AxisAngle cylinderRotationFromZup = EuclidGeometryTools.axisAngleFromZUpToVector3D(cylinderZAxisInWorld);
		  Graphics3DObject linkGraphics = getLinkGraphics();
		  linkGraphics.rotate(cylinderRotationFromZup);
		 linkGraphics.addModelFile(fileName, color);
   }
   
   public void addModelGraphics(String stlFileName, String jointName, String solidworksMassPropertiesTextFileName, AppearanceDefinition color)
   {	   
	   double mass = readScalarMassFromSolidworks(solidworksMassPropertiesTextFileName);
	   Point3D comInWorld = readComPointInWorldFromSolidworks(solidworksMassPropertiesTextFileName);
	   Matrix3D moiAtComExpressedInWorldMatrix = readMoiAboutComAlignedWithOutputCoordinatesFromSolidWorks(solidworksMassPropertiesTextFileName);	   
	   
	   String linkParentJointTransformFileName = jointName + "TransformToWorld.csv";
	   RigidBodyTransform jointTransformToWorld = getRigidBodyTransform(linkParentJointTransformFileName);


	   setMass(mass);

	   Point3D comInLinkParentJointFrame = new Point3D();
	   jointTransformToWorld.transform(comInWorld, comInLinkParentJointFrame);
	   setComOffset(new Vector3D(comInLinkParentJointFrame));
	   
	   jointTransformToWorld.inverseTransform(moiAtComExpressedInWorldMatrix);
//	   setMomentOfInertia(moiAtComExpressedInWorldMatrix);

	   
	   
	   Graphics3DObject linkGraphics = getLinkGraphics();
	   linkGraphics.identity();
	   linkGraphics.transform(jointTransformToWorld);
	   linkGraphics.translate(comInWorld);
	   linkGraphics.addCoordinateSystem(0.1);
	   linkGraphics.identity();
	   
	   AppearanceDefinition transparentBlack = YoAppearance.Black();
	   transparentBlack.setTransparency(0.6);
	   addEllipsoidFromMassProperties(transparentBlack);
	   
	   addModelGraphics(stlFileName, linkParentJointTransformFileName, color);
   }
   
   public List<String> convertTextFileIntoListOfStringsByRow(String textFileName)
   {
	   List<String> ret;
	   
	   try
	   {
		   ret = Files.readAllLines(Paths.get(textFileName));
	   } 
	   catch (IOException e)
	   {
		// TODO Auto-generated catch block
		e.printStackTrace();
		ret = null;
	   }
	   
	   return ret;
   }
      
   public double readScalarMassFromSolidworks(String fileName)
   {
	   List<String> list =  convertTextFileIntoListOfStringsByRow(fileName);
	   
	   String regex = "[^-0-9!\\.,]";
	   int rowNumberWhereMassIsSpecified = 4;
	   
	   double ret = Double.parseDouble(list.get(rowNumberWhereMassIsSpecified).replaceAll(regex, ""));
	   
	   return ret;
   }
   
   public Point3D readComPointInWorldFromSolidworks(String fileName)
   {
	   List<String> list = convertTextFileIntoListOfStringsByRow(fileName);
	   
	   String regex = "[^-0-9!\\.,]";
	   
	   double comX = Double.parseDouble(list.get(11).replaceAll(regex, ""));
	   double comY = Double.parseDouble(list.get(12).replaceAll(regex, ""));
	   double comZ = Double.parseDouble(list.get(13).replaceAll(regex, ""));
	   
	   Point3D ret = new Point3D(comX, comY, comZ);
	   
	   return ret;
   }
   
   public Matrix3D readMoiAboutComAlignedWithOutputCoordinatesFromSolidWorks(String fileName)
   {
	   List<String> list = convertTextFileIntoListOfStringsByRow(fileName);
	   
	   String regex = "[^-=0-9!\\.,]";

	   double[] moiAtCoMExpressedInWorldArray = new double[9];
	   int moiStartRow = 23;
	   int j = 0;
	   
	   for (int i = 0 ; i<3 ; i++)
	   {
		   int row = moiStartRow + i;
		   String[] rowStringArray = list.get(row).replaceAll(regex, "").split("=");
		   
		   for (String string : rowStringArray)
		   {
			   if (! string.isEmpty())
			   {
				   System.out.println("Unit vec string: " + string);
				   moiAtCoMExpressedInWorldArray[j] = Double.parseDouble(string);
				   j++;
			   }
		   }
	   }
	   
	   return new Matrix3D(moiAtCoMExpressedInWorldArray);
   }
   
   public Matrix3D readPrincipicalMoiFromSolidworks(String fileName)
   {
	   List<String> list = convertTextFileIntoListOfStringsByRow(fileName);
	   
	   String regex = "[^-=0-9!\\.,]";
	   
	   double[] moiPrincipleAxesArray = new double[9];
	   double[] moiEigenvalues = new double[3];
	   int moiStartRow = 17;
	   
	   int j = 0;
	   
	   for (int i = 0 ; i<3 ; i++)
	   {
		   int row = moiStartRow + i;
		   String[] rowStringArray = list.get(row).replaceAll(regex, "").split("=");
		   
		   String[] unitVecStringArray = rowStringArray[1].split(",");
		   String moiString = rowStringArray[2];
		   
		   for (String string : unitVecStringArray)
		   {
//			   System.out.println("Unit vec string: " + string);
			   moiPrincipleAxesArray[j] = Double.parseDouble(string);
			   j++;
		   }
		   moiEigenvalues[i] = Double.parseDouble(moiString);
//		   System.out.println("moiString: " + moiString);
	   }
	   
	   Matrix3D moiPrincipleAxesMatrix = new Matrix3D(moiPrincipleAxesArray);
	   Matrix3D eigenValueMatrix = new Matrix3D(moiEigenvalues[0], 0.0, 0.0, 0.0, moiEigenvalues[1], 0.0, 0.0, 0.0, moiEigenvalues[2]);
	   Matrix3D ret = new Matrix3D(moiPrincipleAxesMatrix);
	   ret.multiply(eigenValueMatrix);
	   
	   return ret;
   }
   
   public void addModelGraphics(String stlFileName, String linkParentJointTransformFileName, AppearanceDefinition color)
   {
	   RigidBodyTransform jointTransformToWorld = getRigidBodyTransform(linkParentJointTransformFileName);
	   
	   Graphics3DObject linkGraphics = getLinkGraphics();
	   linkGraphics.transform(jointTransformToWorld);
	   linkGraphics.addModelFile(stlFileName, color);
   }
   
   private RigidBodyTransform getRigidBodyTransform(String transformFileName)
   {
	   double [] transformArray = new double[16];
	   
	   ArrayList<Double> transformArrayToPack = new ArrayList<>();
	   
	   getDataColumnFromCSV(0, new File(transformFileName), transformArrayToPack);
	   
	   for (int i = 0; i < 15 ; i++)
	   {
		   transformArray[i] = transformArrayToPack.get(i);
	   } 

	   RigidBodyTransform ret = new RigidBodyTransform();
	   ret.set(transformArray);
	   ret.invert();
	   
	   return ret;
   }
	
	private void getDataColumnFromCSV(int column, File csvFile, ArrayList<Double> arrayListToPack)
	{
		Scanner csvScanner;
		try
		{
			csvScanner = new Scanner(csvFile);
			
			while( csvScanner.hasNext() )
			{
				String dataRow = csvScanner.next();
				String[] dataRowArray = dataRow.split(",");
				arrayListToPack.add( Double.parseDouble(dataRowArray[column]) );
			}
			csvScanner.close();
		} 
		catch (FileNotFoundException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}	
	}

}
