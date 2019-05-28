package us.ihmc.simulationconstructionset;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.InertiaTools;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.robotdefinition.LinkDefinitionFixedFrame;

/**
 * Describes physical properties of a rigid body. Can attach graphics to it.<p>
 *
 * Title:        Yobotics! Simulation Construction Set<p>
 * Description:  Package for Simulating Dynamic Robots and Mechanisms<p>
 * Copyright:    Copyright (c) Yobotics, Inc. 2000-2005 <p>
 * @author Jerry Pratt
 * @version 1.0
 */
public class Link implements java.io.Serializable
{
   private static final long serialVersionUID = 5447931499263283994L;

   private static final double minimumValidInertia = 2e-7;

   private final String name;

   protected Joint parentJoint;    // Needed for collisions.
   private double mass;

   public Matrix3D Inertia = new Matrix3D();
   public Vector3D principalMomentsOfInertia = new Vector3D();
   public RotationMatrix principalAxesRotation = new RotationMatrix();

   public Vector3D comOffset = new Vector3D();

   private Graphics3DObject linkGraphics;
   private ArrayList<CollisionMeshDescription> collisionMeshDescriptions;

   public Link(LinkDefinitionFixedFrame linkDefinition)
   {
      this(linkDefinition.getName());

      setMass(linkDefinition.getMass());
      setComOffset(linkDefinition.getComOffset());

      setMomentOfInertia(linkDefinition.getInertia());

      linkGraphics = new Graphics3DObject(linkDefinition.getGraphicsDefinition().getGraphics3DInstructions());

      // setLinkGraphics(linkGraphics);
   }

   /**
    * Basic constructor, creates a new link with the specified name.
    *
    * @param linkName Name for this link.
    */
   public Link(String linkName)
   {
      this.name = linkName;

      linkGraphics = new Graphics3DObject();
   }

   /**
    * Copy constructor. Doesn't copy the parentJoint reference.
    * @param other Link to copy
    */
   public Link(Link other)
   {
      this.name = new String(other.name);
      this.linkGraphics = new Graphics3DObject(other.linkGraphics.getGraphics3DInstructions());
      this.setComOffset(other.getComOffset());
      this.setMass(other.getMass());
      Matrix3D moi = new Matrix3D();
      other.getMomentOfInertia(moi);
      this.setMomentOfInertia(moi);
   }

   /**
    * Retrieves this link's name.
    *
    * @return Name of this link.
    */
   public String getName()
   {
      return name;
   }

   /**
    * Returns a string representation of this link.
    * <pre>The following is an example of its format:
    *
    *       {@literal Link: <Link Name>}
    *          {@literal Mass: <Link Mass>}
    *          {@literal COM Offset: <Center of Mass offset>}
    *          {@literal Moment of Inertia: }
    * {@literal m00, m01, m02}
    * {@literal m10, m11, m12}
    * {@literal m20, m21, m22}</pre>
    *
    * @return String representation of this link.
    */
   @Override
   public String toString()
   {
      StringBuffer retBuffer = new StringBuffer();

      retBuffer.append("      Link: " + name + "\n");
      retBuffer.append("         Mass: " + mass + "\n");
      retBuffer.append("         COM Offset: " + comOffset + "\n");
      retBuffer.append("         Moment of Inertia: \n" + this.Inertia);

      return retBuffer.toString();
   }

   /**
    * Sets the moment of inertia for this link.  This is stored as a matrix
    * with the values Ixx, Iyy and Izz representing the positions m00, m11 and
    * m22 respectively.<br /><br />
    *
    * {@literal Ixx  0   0}<br />
    * {@literal  0  Iyy  0 }<br />
    * {@literal  0   0  Izz}<br />
    *
    * Inertias are represented in units of kg*m^2
    *
    * @param Ixx double
    * @param Iyy double
    * @param Izz double
    */
   public void setMomentOfInertia(double Ixx, double Iyy, double Izz)
   {
      this.Inertia.setM00(Ixx);
      this.Inertia.setM01(0.0);
      this.Inertia.setM02(0.0);
      this.Inertia.setM10(0.0);
      this.Inertia.setM11(Iyy);
      this.Inertia.setM12(0.0);
      this.Inertia.setM20(0.0);
      this.Inertia.setM21(0.0);
      this.Inertia.setM22(Izz);

      computePrincipalMomentsOfInertia();
   }

   /**
    * Sets the moment of inertia for this link.  Inertias are represented in units of kg*m^2
    *
    * @param momentOfInertia Matrix3d representing the moment of inertia
    */
   public void setMomentOfInertia(Matrix3D momentOfInertia)
   {
      this.Inertia.set(momentOfInertia);
      computePrincipalMomentsOfInertia();
   }

   public void setMomentOfInertia(DenseMatrix64F momentOfInertia)
   {
      Inertia.setM00(momentOfInertia.get(0, 0));
      Inertia.setM01(momentOfInertia.get(0, 1));
      Inertia.setM02(momentOfInertia.get(0, 2));

      Inertia.setM10(momentOfInertia.get(1, 0));
      Inertia.setM11(momentOfInertia.get(1, 1));
      Inertia.setM12(momentOfInertia.get(1, 2));

      Inertia.setM20(momentOfInertia.get(2, 0));
      Inertia.setM21(momentOfInertia.get(2, 1));
      Inertia.setM22(momentOfInertia.get(2, 2));

      computePrincipalMomentsOfInertia();
   }

   /**
    * Retrieves the offset from this link's inboard joint to its center of mass.
    *
    * @return Vector3d representing the offset.
    */
   public Vector3D getComOffset()
   {
      return this.comOffset;
   }

   /**
    * Sets the offset from the inbound joint to this links center of mass. The inbound
    * joint is usually the parent to the link in question.  This defaults to 0,0,0 and must
    * be specified prior to simulation if accurate results are desired.
    *
    * @param xOffset Offset in the x direction.
    * @param yOffset Offset in the y direction.
    * @param zOffset Offset in the z direction.
    */
   public void setComOffset(double xOffset, double yOffset, double zOffset)
   {
      this.comOffset.set(xOffset, yOffset, zOffset);
   }


   /**
    * Sets the offset from the inbound joint ot this links center of mass.
    *
    * @param comOffset Vector3d representing the offset.
    */
   public void setComOffset(Vector3D comOffset)
   {
      this.comOffset.set(comOffset);
   }

   /**
    * Sets the parent joint of this link.  Non-static links are always associated with joints as they
    * determine the manner of the links response.
    *
    * @param joint Joint to assign as the parent of this link.
    */
   protected void setParentJoint(Joint joint)
   {
      this.parentJoint = joint;
      if (this.contactingExternalForcePoints != null)
      {
         throw new RuntimeException("Need to attach link to joint before enabling collisions!");
//         joint.addExternalForcePoint(ef_collision);
      }
   }

   public static Link combineLinks(String name, Link linkOne, Link linkTwo)
   {
      return combineLinks(name, linkOne, linkTwo, new Vector3D());
   }

   /**
    * Combines the specified link with this link.  This combines their
    * mass, calculates the resulting center of mass, and the new overall
    * inertia of the combined link.
    *
    * @param linkTwo Link to be combined with this link.
    */
   public static Link combineLinks(String name, Link linkOne, Link linkTwo, Vector3D linkOffset)
   {
      // Compute the new center of mass:

      // Center of mass is weighted averages of centers of mass:
      Vector3D newComOffset = new Vector3D();
      newComOffset.setAndScale(linkOne.mass, linkOne.comOffset);

      Vector3D linkToAddTotalOffset = new Vector3D(linkOffset);
      linkToAddTotalOffset.add(linkTwo.comOffset);

      Vector3D temp = new Vector3D();
      temp.setAndScale(linkTwo.mass, linkToAddTotalOffset);
      newComOffset.add(temp);
      newComOffset.scale(1.0 / (linkOne.mass + linkTwo.mass));

      // New mass is the sum of the old:
      double newMass = linkOne.mass + linkTwo.mass;

      // New inertia is sum of the old plus parallel axis theorem:
      Matrix3D newInertia = new Matrix3D();
      newInertia.add(linkOne.Inertia, linkTwo.Inertia);

      // Parallel Axis Theorem:
      double a, b, c;

      a = linkOne.comOffset.getX() - newComOffset.getX();
      b = linkOne.comOffset.getY() - newComOffset.getY();
      c = linkOne.comOffset.getZ() - newComOffset.getZ();
      Matrix3D inertiaOffset1 = new Matrix3D(b * b + c * c, -a * b, -a * c, -a * b, c * c + a * a, -b * c, -a * c, -b * c, a * a + b * b);

      a = linkToAddTotalOffset.getX() - newComOffset.getX();
      b = linkToAddTotalOffset.getY() - newComOffset.getY();
      c = linkToAddTotalOffset.getZ() - newComOffset.getZ();
      Matrix3D inertiaOffset2 = new Matrix3D(b * b + c * c, -a * b, -a * c, -a * b, c * c + a * a, -b * c, -a * c, -b * c, a * a + b * b);

      inertiaOffset1.scale(linkOne.mass);
      inertiaOffset2.scale(linkTwo.mass);

      newInertia.add(inertiaOffset1);
      newInertia.add(inertiaOffset2);

      // Done with Parallel Axis Theorm

      Link ret = new Link(name);
      ret.setMass(newMass);
      ret.setComOffset(newComOffset);
      ret.setMomentOfInertia(newInertia);

      Graphics3DObject newLinkGraphics = new Graphics3DObject(linkOne.linkGraphics);

      newLinkGraphics.combine(linkTwo.linkGraphics, linkOffset);
      ret.setLinkGraphics(newLinkGraphics);

      return ret;
   }

   /**
    * Sets the mass of this joint.  All masses are in kilograms.
    *
    * @param mass New joint mass.
    */
   public void setMass(double mass)
   {
      this.mass = mass;
   }

   /**
    * Retrieves the mass of this joint.  All masses are in kilograms.
    *
    * @return The mass of this joint.
    */
   public double getMass()
   {
      return this.mass;
   }

   public void getMomentOfInertia(Matrix3D momentOfInertiaToPack)
   {
      momentOfInertiaToPack.set(this.Inertia);
   }

   /**
    * Sets the mass and moment of inertia of this link. The moments of inertia are computed as
    * Ixx = mass * (radiusOfGyrationY * radiusOfGyrationY + radiusOfGyrationZ * radiusOfGyrationY), etc.
    * This is equivalent to the mass being concentrated on the surface of a thin ellipsoid with the given radii of gyration.
    *
    * @param mass Mass of the link.
    * @param radiusOfGyrationX Radius of gyration in the x direction.
    * @param radiusOfGyrationY Radius of gyration in the y direction.
    * @param radiusOfGyrationZ Radius of gyration in the z direction.
    */
   public void setMassAndRadiiOfGyration(double mass, double radiusOfGyrationX, double radiusOfGyrationY, double radiusOfGyrationZ)
   {
      this.mass = mass;

      double Ixx = mass * (radiusOfGyrationY * radiusOfGyrationY + radiusOfGyrationZ * radiusOfGyrationZ);
      double Iyy = mass * (radiusOfGyrationX * radiusOfGyrationX + radiusOfGyrationZ * radiusOfGyrationZ);
      double Izz = mass * (radiusOfGyrationX * radiusOfGyrationX + radiusOfGyrationY * radiusOfGyrationY);

      setMomentOfInertia(Ixx, Iyy, Izz);

   }

   /**
    * Sets the mass and moment of inertia of this link. The moments of inertia are computed as
    * Ixx = mass * (radiusOfGyrationY * radiusOfGyrationY + radiusOfGyrationZ * radiusOfGyrationY), etc.
    * This is equivalent to the mass being concentrated on the surface of a thin ellipsoid with the given radii of gyration.
    * 
    * @param mass Mass of the link.
    * @param radiiOfGyration Radii of gyration in the x, y, and z directions.
    */
   public void setMassAndRadiiOfGyration(double mass, Vector3D radiiOfGyration)
   {
      setMassAndRadiiOfGyration(mass, radiiOfGyration.getX(), radiiOfGyration.getY(), radiiOfGyration.getZ());
   }

   /**
    * Sets the graphical representation of this link to the provided LinkGraphics.
    * LinkGraphics store the graphical data for each link allowing different shapes and
    * features to be created and stored.
    *
    * @param linkGraphics LinkGraphics to be used for this link.
    */
   public void setLinkGraphics(Graphics3DObject linkGraphics)
   {
      this.linkGraphics = linkGraphics;
   }

   public void addCollisionMesh(CollisionMeshDescription collisionMeshDescription)
   {
      if (collisionMeshDescriptions == null)
      {
         collisionMeshDescriptions = new ArrayList<CollisionMeshDescription>();
      }
      
      this.collisionMeshDescriptions.add(collisionMeshDescription);
   }

   /**
    * Retrieves the LinkGraphics object representing this link.
    *
    * @return LinkGraphics representing the graphical properties of this link.
    */
   public Graphics3DObject getLinkGraphics()
   {
      return linkGraphics;
   }

   public ArrayList<CollisionMeshDescription> getCollisionMeshDescriptions()
   {
      return collisionMeshDescriptions;
   }

   //TODO: Get this stuff out of here. Put it in Joint maybe?
// ///////////// Collision Stuff Here /////////////

//   public ExternalForcePoint ef_collision;
   private ArrayList<ContactingExternalForcePoint> contactingExternalForcePoints;

   public ArrayList<ContactingExternalForcePoint> getContactingExternalForcePoints()
   {
      return contactingExternalForcePoints;
   }

   /**
    * Enables collisions for this link.
    * @param maxVelocity Maximum velocity of any point on the link. Used for improving collision detection performance.
    * @param polyTree PolyTree defining collision geometry.
    */
   public void enableCollisions(int maximumNumberOfPotentialContacts, CollisionHandler collisionHandler, YoVariableRegistry registry)
   {
      if (parentJoint == null)
      {
         throw new RuntimeException("Need to attach link to joint before enabling collisions!");
      }

      contactingExternalForcePoints = new ArrayList<>();

      for (int i=0; i<maximumNumberOfPotentialContacts; i++)
      {
         ContactingExternalForcePoint contactingExternalForcePoint = new ContactingExternalForcePoint(this.name + "ContactingPoint" + i, parentJoint, registry);
         contactingExternalForcePoints.add(contactingExternalForcePoint);
         parentJoint.addExternalForcePoint(contactingExternalForcePoint);
      }
      
      collisionHandler.addContactingExternalForcePoints(this, contactingExternalForcePoints);
   }

   // ////////// Graphics from Mass Properties Here ///////////////////////

   /**
    * Goes through the given robot and finds all Links and adds an ellipsoid based on mass properties
    * @param robot
    * @param appearance
    */
   public static void addEllipsoidFromMassPropertiesToAllLinks(RobotFromDescription robot, AppearanceDefinition appearance)
   {
      ArrayList<Joint> joints = new ArrayList<>(robot.getRootJoints());
      
      while(!joints.isEmpty())
      {
         int lastIndex = joints.size() - 1;
         Joint joint = joints.get(lastIndex);
         joint.getLink().addEllipsoidFromMassProperties(appearance);

         joints.remove(lastIndex);

         ArrayList<Joint> childrenJoints = joint.getChildrenJoints();
         if (childrenJoints != null)
         {
            joints.addAll(childrenJoints);
         }
      }
   }
   
   /**
    * Adds an ellipsoid representing the mass and inertia of the link at its center of mass.
    * This ellipsoid has a default matte black appearance.
    */
   public void addEllipsoidFromMassProperties()
   {
      addEllipsoidFromMassProperties(null);
   }

   /**
    * Adds a coordinate system representation at the center of mass of this link.  The axis of this system
    * have the given length.
    *
    * @param length length in meters of each arm/axis on the coordinate system.
    */
   public void addCoordinateSystemToCOM(double length)
   {
      linkGraphics.identity();

      Vector3D comOffset = new Vector3D();
      getComOffset(comOffset);

      linkGraphics.translate(comOffset.getX(), comOffset.getY(), comOffset.getZ());
      linkGraphics.addCoordinateSystem(length);

      linkGraphics.identity();
   }


   public void addEllipsoidFromMassProperties2(AppearanceDefinition appearance)
   {
	   computePrincipalMomentsOfInertia();

	   Vector3D inertiaEllipsoidRadii = InertiaTools.getInertiaEllipsoidRadii(principalMomentsOfInertia, mass);

	   ArrayList<Vector3D> inertiaEllipsoidAxes = new ArrayList<Vector3D>();

	   Vector3D e1 = new Vector3D(); principalAxesRotation.getColumn(0, e1); e1.normalize(); e1.scale(inertiaEllipsoidRadii.getX()); inertiaEllipsoidAxes.add(e1);
	   Vector3D e2 = new Vector3D(); principalAxesRotation.getColumn(1, e2); e2.normalize(); e2.scale(inertiaEllipsoidRadii.getY()); inertiaEllipsoidAxes.add(e2);
	   Vector3D e3 = new Vector3D(); principalAxesRotation.getColumn(2, e3); e3.normalize(); e3.scale(inertiaEllipsoidRadii.getZ()); inertiaEllipsoidAxes.add(e3);
	   Vector3D e4 = new Vector3D(e1); e4.negate(); inertiaEllipsoidAxes.add(e4);
	   Vector3D e5 = new Vector3D(e2); e5.negate(); inertiaEllipsoidAxes.add(e5);
	   Vector3D e6 = new Vector3D(e3); e6.negate(); inertiaEllipsoidAxes.add(e6);

	   double vertexSize = 0.01 * inertiaEllipsoidRadii.length();

	   for (Vector3D vector : inertiaEllipsoidAxes)
	   {
		   linkGraphics.identity();
		   linkGraphics.translate(comOffset.getX(), comOffset.getY(), comOffset.getZ());
		   linkGraphics.translate(vector);
		   linkGraphics.addCube(vertexSize, vertexSize, vertexSize, appearance);
	   }

	   ArrayList<Point3D> inertiaOctahedronVertices = new ArrayList<Point3D>();

	   Point3D p1 = new Point3D(e1); inertiaOctahedronVertices.add(p1);
	   Point3D p2 = new Point3D(e2); inertiaOctahedronVertices.add(p2);
	   Point3D p3 = new Point3D(e4); inertiaOctahedronVertices.add(p3);
	   Point3D p4 = new Point3D(e5); inertiaOctahedronVertices.add(p4);
	   Point3D p5 = new Point3D(e3); inertiaOctahedronVertices.add(p5);
	   Point3D p6 = new Point3D(e6); inertiaOctahedronVertices.add(p6);

	   ArrayList<Point3D> face1 = new ArrayList<Point3D>(); face1.add(p1); face1.add(p5); face1.add(p4);
	   ArrayList<Point3D> face2 = new ArrayList<Point3D>(); face2.add(p4); face2.add(p5); face2.add(p3);
	   ArrayList<Point3D> face3 = new ArrayList<Point3D>(); face3.add(p3); face3.add(p5); face3.add(p2);
	   ArrayList<Point3D> face4 = new ArrayList<Point3D>(); face4.add(p2); face4.add(p5); face4.add(p1);

	   ArrayList<Point3D> face5 = new ArrayList<Point3D>(); face5.add(p4); face5.add(p6); face5.add(p1);
	   ArrayList<Point3D> face6 = new ArrayList<Point3D>(); face6.add(p3); face6.add(p6); face6.add(p4);
	   ArrayList<Point3D> face7 = new ArrayList<Point3D>(); face7.add(p2); face7.add(p6); face7.add(p3);
	   ArrayList<Point3D> face8 = new ArrayList<Point3D>(); face8.add(p1); face8.add(p6); face8.add(p2);


	   linkGraphics.identity();
	   linkGraphics.translate(comOffset.getX(), comOffset.getY(), comOffset.getZ());
	   linkGraphics.addPolygon(face1, appearance);
	   linkGraphics.addPolygon(face2, appearance);
	   linkGraphics.addPolygon(face3, appearance);
	   linkGraphics.addPolygon(face4, appearance);
	   linkGraphics.addPolygon(face5, appearance);
	   linkGraphics.addPolygon(face6, appearance);
	   linkGraphics.addPolygon(face7, appearance);
	   linkGraphics.addPolygon(face8, appearance);

//	   linkGraphics.identity();
//	   linkGraphics.translate(comOffset.x, comOffset.y, comOffset.z);
//	   linkGraphics.rotate(principalAxesRotation);
//	   linkGraphics.addEllipsoid(inertiaEllipsoidRadii.x, inertiaEllipsoidRadii.y, inertiaEllipsoidRadii.z, appearance);
//	   linkGraphics.identity();

   }


   /**
    * Adds an ellipsoid representing the mass and inertia of the link at its center of mass
    * with the specified appearance.
    *
    * @param appearance Appearance to be used with the ellipsoid.  See {@link YoAppearance YoAppearance} for implementations.
    */
   public void addEllipsoidFromMassProperties(AppearanceDefinition appearance)
   {
      computePrincipalMomentsOfInertia();

      Vector3D inertiaEllipsoidRadii = InertiaTools.getInertiaEllipsoidRadii(principalMomentsOfInertia, mass);

      if (appearance == null)
         appearance = YoAppearance.Black();

      linkGraphics.identity();
      linkGraphics.translate(comOffset.getX(), comOffset.getY(), comOffset.getZ());
      linkGraphics.rotate(principalAxesRotation);
      linkGraphics.addEllipsoid(inertiaEllipsoidRadii.getX(), inertiaEllipsoidRadii.getY(), inertiaEllipsoidRadii.getZ(), appearance);
      linkGraphics.identity();
   }


   /**
    * Adds an box representing the mass and inertia of the link at its center of mass
    * with the specified appearance.
    *
    * Specifically, mimics the code from Gazebo to debug SDF loader
    *
    * See https://bitbucket.org/osrf/gazebo/src/0709b57a8a3a8abce3c67e992e5c6a5c24c8d84a/gazebo/rendering/COMVisual.cc?at=default
    *
    * @param appearance Appearance to be used with the ellipsoid.  See {@link YoAppearance YoAppearance} for implementations.
    */
   public void addBoxFromMassProperties(AppearanceDefinition appearance)
   {
      if (mass <= 0 || Inertia.getM00() <= 0 || Inertia.getM11() <= 0 || Inertia.getM22() <= 0 || Inertia.getM00() + Inertia.getM11() <= Inertia.getM22()
            || Inertia.getM11() + Inertia.getM22() <= Inertia.getM00() || Inertia.getM00() + Inertia.getM22() <= Inertia.getM11())
      {
         System.err.println(getName() + " has unrealistic inertia");
      }
      else
      {
         linkGraphics.identity();
         linkGraphics.translate(comOffset.getX(), comOffset.getY(), comOffset.getZ());
         double lx = Math.sqrt(6 * (Inertia.getM22() + Inertia.getM11() - Inertia.getM00()) / mass);
         double ly = Math.sqrt(6 * (Inertia.getM22() + Inertia.getM00() - Inertia.getM11()) / mass);
         double lz = Math.sqrt(6 * (Inertia.getM00() + Inertia.getM11() - Inertia.getM22()) / mass);
         linkGraphics.translate(0, 0, -lz/2.0);
         linkGraphics.addCube(lx, ly, lz, appearance);
         linkGraphics.identity();
      }
   }

   /**
    * Stores a vector3d representation of the offset from the links center of mass in
    * the provided variable.
    *
    * @param comOffsetRet Vector3d in which the offset will be stored.
    */
   public void getComOffset(Vector3D comOffsetRet)
   {
      comOffsetRet.set(this.comOffset);
   }

   public Joint getParentJoint()
   {
      return parentJoint;
   }

   public void computePrincipalMomentsOfInertia()
   {
      InertiaTools.computePrincipalMomentsOfInertia(Inertia, principalAxesRotation, principalMomentsOfInertia);
      if ((principalMomentsOfInertia.getX() < minimumValidInertia) || (principalMomentsOfInertia.getY() < minimumValidInertia) || (principalMomentsOfInertia.getZ() < minimumValidInertia))
      {
    	  String parentJointName = "  No parent joint.";
    	  if (parentJoint != null)
    	  {
    		  parentJointName = "  Link parent: " + parentJoint.getName() + " .";
    	  }
    	  System.err.println("Warning: Inertia may be too small for Link " + getName() + " for stable simulation. principalMomentsOfInertia = " + principalMomentsOfInertia + parentJointName);
      }
   }


}
