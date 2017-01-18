package us.ihmc.robotics.robotDescription;

import java.util.ArrayList;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.InertiaTools;

public class LinkDescription
{
   private String name;

   private double mass;
   private final Vector3d centerOfMassOffset = new Vector3d();
   private final DenseMatrix64F momentOfInertia = new DenseMatrix64F(3, 3);

   private final Vector3d principalMomentsOfInertia = new Vector3d();
   private final Matrix3d principalAxesRotation = new Matrix3d();

   private LinkGraphicsDescription linkGraphics;
   private CollisionMeshDescription collisionMesh;

   public LinkDescription(String name)
   {
      this.name = name;
   }

   public String getName()
   {
      return name;
   }

   public LinkGraphicsDescription getLinkGraphics()
   {
      return linkGraphics;
   }

   public void setLinkGraphics(LinkGraphicsDescription linkGraphics)
   {
      this.linkGraphics = linkGraphics;
   }

   public CollisionMeshDescription getCollisionMesh()
   {
      return collisionMesh;
   }

   public void setCollisionMesh(CollisionMeshDescription collisionMesh)
   {
      this.collisionMesh = collisionMesh;
   }

   public double getMass()
   {
      return mass;
   }

   public void setMass(double mass)
   {
      if (mass < 0.0)
         throw new RuntimeException("mass < 0.0");
      this.mass = mass;
   }

   public void getCenterOfMassOffset(Vector3d centerOfMassOffsetToPack)
   {
      centerOfMassOffsetToPack.set(centerOfMassOffset);
   }

   public Vector3d getCenterOfMassOffset()
   {
      return centerOfMassOffset;
   }

   public void setCenterOfMassOffset(Vector3d centerOfMassOffset)
   {
      this.centerOfMassOffset.set(centerOfMassOffset);
   }

   public void setCenterOfMassOffset(double xOffset, double yOffset, double zOffset)
   {
      this.centerOfMassOffset.set(xOffset, yOffset, zOffset);
   }

   public void setMomentOfInertia(DenseMatrix64F momentOfInertia)
   {
      this.momentOfInertia.set(momentOfInertia);
   }

   public void setMomentOfInertia(Matrix3d momentOfInertia)
   {
      for (int i=0; i<3; i++)
      {
         for (int j=0; j<3; j++)
         {
            this.momentOfInertia.set(i, j, momentOfInertia.getElement(i, j));
         }
      }
   }

   public Matrix3d getMomentOfInertiaCopy()
   {
      Matrix3d momentOfInertia = new Matrix3d();

      for (int i=0; i<3; i++)
      {
         for (int j=0; j<3; j++)
         {
            momentOfInertia.setElement(i, j, this.momentOfInertia.get(i, j));
         }
      }

      return momentOfInertia;
   }

   public void setMassAndRadiiOfGyration(double mass, double radiusOfGyrationX, double radiusOfGyrationY, double radiusOfGyrationZ)
   {
      this.mass = mass;

      double Ixx = mass * (radiusOfGyrationY * radiusOfGyrationY + radiusOfGyrationZ * radiusOfGyrationZ);
      double Iyy = mass * (radiusOfGyrationX * radiusOfGyrationX + radiusOfGyrationZ * radiusOfGyrationZ);
      double Izz = mass * (radiusOfGyrationX * radiusOfGyrationX + radiusOfGyrationY * radiusOfGyrationY);

      setMomentOfInertia(Ixx, Iyy, Izz);
   }

   public void getMomentOfInertia(DenseMatrix64F momentOfInertiaToPack)
   {
      momentOfInertiaToPack.set(momentOfInertia);
   }

   public DenseMatrix64F getMomentOfInertia()
   {
      return momentOfInertia;
   }

   public void setMomentOfInertia(double Ixx, double Iyy, double Izz)
   {
      this.momentOfInertia.zero();
      this.momentOfInertia.set(0, 0, Ixx);
      this.momentOfInertia.set(1, 1, Iyy);
      this.momentOfInertia.set(2, 2, Izz);

   }

   // ////////// Graphics from Mass Properties Here ///////////////////////

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

      Vector3d comOffset = new Vector3d();
      getCenterOfMassOffset(comOffset);

      linkGraphics.translate(comOffset.getX(), comOffset.getY(), comOffset.getZ());
      linkGraphics.addCoordinateSystem(length);

      linkGraphics.identity();
   }

   public void addEllipsoidFromMassProperties2(AppearanceDefinition appearance)
   {
      computePrincipalMomentsOfInertia();

      Vector3d inertiaEllipsoidRadii = InertiaTools.getInertiaEllipsoidRadii(principalMomentsOfInertia, mass);

      ArrayList<Vector3d> inertiaEllipsoidAxes = new ArrayList<Vector3d>();

      Vector3d e1 = new Vector3d();
      principalAxesRotation.getColumn(0, e1);
      e1.normalize();
      e1.scale(inertiaEllipsoidRadii.getX());
      inertiaEllipsoidAxes.add(e1);
      Vector3d e2 = new Vector3d();
      principalAxesRotation.getColumn(1, e2);
      e2.normalize();
      e2.scale(inertiaEllipsoidRadii.getY());
      inertiaEllipsoidAxes.add(e2);
      Vector3d e3 = new Vector3d();
      principalAxesRotation.getColumn(2, e3);
      e3.normalize();
      e3.scale(inertiaEllipsoidRadii.getZ());
      inertiaEllipsoidAxes.add(e3);
      Vector3d e4 = new Vector3d(e1);
      e4.negate();
      inertiaEllipsoidAxes.add(e4);
      Vector3d e5 = new Vector3d(e2);
      e5.negate();
      inertiaEllipsoidAxes.add(e5);
      Vector3d e6 = new Vector3d(e3);
      e6.negate();
      inertiaEllipsoidAxes.add(e6);

      double vertexSize = 0.01 * inertiaEllipsoidRadii.length();

      Vector3d comOffset = new Vector3d();
      getCenterOfMassOffset(comOffset);

      for (Vector3d vector : inertiaEllipsoidAxes)
      {
         linkGraphics.identity();
         linkGraphics.translate(comOffset.getX(), comOffset.getY(), comOffset.getZ());
         linkGraphics.translate(vector);
         linkGraphics.addCube(vertexSize, vertexSize, vertexSize, appearance);
      }

      ArrayList<Point3d> inertiaOctahedronVertices = new ArrayList<Point3d>();

      Point3d p1 = new Point3d(e1);
      inertiaOctahedronVertices.add(p1);
      Point3d p2 = new Point3d(e2);
      inertiaOctahedronVertices.add(p2);
      Point3d p3 = new Point3d(e4);
      inertiaOctahedronVertices.add(p3);
      Point3d p4 = new Point3d(e5);
      inertiaOctahedronVertices.add(p4);
      Point3d p5 = new Point3d(e3);
      inertiaOctahedronVertices.add(p5);
      Point3d p6 = new Point3d(e6);
      inertiaOctahedronVertices.add(p6);

      ArrayList<Point3d> face1 = new ArrayList<Point3d>();
      face1.add(p1);
      face1.add(p5);
      face1.add(p4);
      ArrayList<Point3d> face2 = new ArrayList<Point3d>();
      face2.add(p4);
      face2.add(p5);
      face2.add(p3);
      ArrayList<Point3d> face3 = new ArrayList<Point3d>();
      face3.add(p3);
      face3.add(p5);
      face3.add(p2);
      ArrayList<Point3d> face4 = new ArrayList<Point3d>();
      face4.add(p2);
      face4.add(p5);
      face4.add(p1);

      ArrayList<Point3d> face5 = new ArrayList<Point3d>();
      face5.add(p4);
      face5.add(p6);
      face5.add(p1);
      ArrayList<Point3d> face6 = new ArrayList<Point3d>();
      face6.add(p3);
      face6.add(p6);
      face6.add(p4);
      ArrayList<Point3d> face7 = new ArrayList<Point3d>();
      face7.add(p2);
      face7.add(p6);
      face7.add(p3);
      ArrayList<Point3d> face8 = new ArrayList<Point3d>();
      face8.add(p1);
      face8.add(p6);
      face8.add(p2);

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

      //    linkGraphics.identity();
      //    linkGraphics.translate(comOffset.x, comOffset.y, comOffset.z);
      //    linkGraphics.rotate(principalAxesRotation);
      //    linkGraphics.addEllipsoid(inertiaEllipsoidRadii.x, inertiaEllipsoidRadii.y, inertiaEllipsoidRadii.z, appearance);
      //    linkGraphics.identity();

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

      Vector3d inertiaEllipsoidRadii = InertiaTools.getInertiaEllipsoidRadii(principalMomentsOfInertia, mass);

      if (appearance == null)
         appearance = YoAppearance.Black();

      Vector3d comOffset = new Vector3d();
      getCenterOfMassOffset(comOffset);

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
      Vector3d comOffset = new Vector3d();
      getCenterOfMassOffset(comOffset);

      if (mass <= 0 || momentOfInertia.get(0, 0) <= 0 || momentOfInertia.get(1, 1) <= 0 || momentOfInertia.get(2, 2) <= 0 || momentOfInertia.get(0, 0) + momentOfInertia.get(1, 1) <= momentOfInertia.get(2, 2)
            || momentOfInertia.get(1, 1) + momentOfInertia.get(2, 2) <= momentOfInertia.get(0, 0) || momentOfInertia.get(0, 0) + momentOfInertia.get(2, 2) <= momentOfInertia.get(1, 1))
      {
         System.err.println(getName() + " has unrealistic inertia");
      }
      else
      {
         linkGraphics.identity();
         linkGraphics.translate(comOffset.getX(), comOffset.getY(), comOffset.getZ());
         double lx = Math.sqrt(6.0 * (momentOfInertia.get(2, 2) + momentOfInertia.get(1, 1) - momentOfInertia.get(0, 0)) / mass);
         double ly = Math.sqrt(6.0 * (momentOfInertia.get(2, 2) + momentOfInertia.get(0, 0) - momentOfInertia.get(1, 1)) / mass);
         double lz = Math.sqrt(6.0 * (momentOfInertia.get(0, 0) + momentOfInertia.get(1, 1) - momentOfInertia.get(2, 2)) / mass);
         linkGraphics.translate(0.0, 0.0, -lz / 2.0);
         linkGraphics.addCube(lx, ly, lz, appearance);
         linkGraphics.identity();
      }
   }

   public void computePrincipalMomentsOfInertia()
   {
      InertiaTools.computePrincipalMomentsOfInertia(momentOfInertia, principalAxesRotation, principalMomentsOfInertia);
   }

   public void scale(double factor, double massScalePower, boolean scaleInertia)
   {
      // Center of mass offset scales with the scaling factor
      centerOfMassOffset.scale(factor);
      
      // Mass scales with factor^massScalePower. massScalePower is 3 when considering constant density
      
      if(scaleInertia)
      {
         mass = Math.pow(factor, massScalePower) * mass;
         // The components of the inertia matrix are defined with int(r^2 dm). So they scale factor ^ (2 + massScalePower)
         double inertiaScale = Math.pow(factor, massScalePower + 2);
         CommonOps.scale(inertiaScale, momentOfInertia);
         computePrincipalMomentsOfInertia();
      }
      
      
      if(linkGraphics != null)
      {
         linkGraphics.preScale(factor);
      }
      if(collisionMesh != null)
      {
         collisionMesh.scale(factor);
      }
      
      
      
   }

}
