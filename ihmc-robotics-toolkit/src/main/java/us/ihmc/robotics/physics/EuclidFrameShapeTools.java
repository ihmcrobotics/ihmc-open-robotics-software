package us.ihmc.robotics.physics;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.shape.primitives.Ellipsoid3D;
import us.ihmc.euclid.shape.primitives.PointShape3D;
import us.ihmc.euclid.shape.primitives.Ramp3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidFrameShapeTools
{
   public static void boundingBox3D(ReferenceFrame frame, Shape3DReadOnly shape3D, BoundingBox3DBasics boundingBoxToPack)
   {
      if (shape3D instanceof Box3DReadOnly)
         boundingBox3D(frame, (Box3DReadOnly) shape3D, boundingBoxToPack);
      else if (shape3D instanceof Capsule3DReadOnly)
         boundingBox3D(frame, (Capsule3DReadOnly) shape3D, boundingBoxToPack);
      else if (shape3D instanceof Cylinder3DReadOnly)
         boundingBox3D(frame, (Cylinder3DReadOnly) shape3D, boundingBoxToPack);
      else if (shape3D instanceof Ellipsoid3DReadOnly)
         boundingBox3D(frame, (Ellipsoid3DReadOnly) shape3D, boundingBoxToPack);
      else if (shape3D instanceof PointShape3DReadOnly)
         boundingBox3D(frame, (PointShape3DReadOnly) shape3D, boundingBoxToPack);
      else if (shape3D instanceof Ramp3DReadOnly)
         boundingBox3D(frame, (Ramp3DReadOnly) shape3D, boundingBoxToPack);
      else if (shape3D instanceof Sphere3DReadOnly)
         boundingBox3D(frame, (Sphere3DReadOnly) shape3D, boundingBoxToPack);
      else if (shape3D instanceof SupportingVertexHolder)
         boundingBox3D(frame, (SupportingVertexHolder) shape3D, boundingBoxToPack);
      else
         throw new UnsupportedOperationException("Unsupported shape: " + shape3D);
   }

   public static void boundingBox3D(ReferenceFrame frame, Box3DReadOnly box3D, BoundingBox3DBasics boundingBoxToPack)
   {
      if (frame.isRootFrame())
      {
         box3D.getBoundingBox(boundingBoxToPack);
         return;
      }

      RigidBodyTransform transformToRoot = frame.getTransformToRoot();
      Vector3DBasics framePos = transformToRoot.getTranslation();
      RotationMatrixBasics frameRot = transformToRoot.getRotation();

      if (frameRot.isIdentity())
      {
         box3D.getBoundingBox(boundingBoxToPack);
         boundingBoxToPack.getMinPoint().add(framePos);
         boundingBoxToPack.getMaxPoint().add(framePos);
         return;
      }

      Vector3DReadOnly size = box3D.getSize();
      Point3DReadOnly position = box3D.getPosition();
      RotationMatrixReadOnly orientation = box3D.getOrientation();

      double m00, m01, m02;
      double m10, m11, m12;
      double m20, m21, m22;

      if (orientation.isIdentity())
      {
         m00 = frameRot.getM00();
         m01 = frameRot.getM01();
         m02 = frameRot.getM02();
         m10 = frameRot.getM10();
         m11 = frameRot.getM11();
         m12 = frameRot.getM12();
         m20 = frameRot.getM20();
         m21 = frameRot.getM21();
         m22 = frameRot.getM22();
      }
      else
      {
         m00 = frameRot.getM00() * orientation.getM00() + frameRot.getM01() * orientation.getM10() + frameRot.getM02() * orientation.getM20();
         m01 = frameRot.getM00() * orientation.getM01() + frameRot.getM01() * orientation.getM11() + frameRot.getM02() * orientation.getM21();
         m02 = frameRot.getM00() * orientation.getM02() + frameRot.getM01() * orientation.getM12() + frameRot.getM02() * orientation.getM22();
         m10 = frameRot.getM10() * orientation.getM00() + frameRot.getM11() * orientation.getM10() + frameRot.getM12() * orientation.getM20();
         m11 = frameRot.getM10() * orientation.getM01() + frameRot.getM11() * orientation.getM11() + frameRot.getM12() * orientation.getM21();
         m12 = frameRot.getM10() * orientation.getM02() + frameRot.getM11() * orientation.getM12() + frameRot.getM12() * orientation.getM22();
         m20 = frameRot.getM20() * orientation.getM00() + frameRot.getM21() * orientation.getM10() + frameRot.getM22() * orientation.getM20();
         m21 = frameRot.getM20() * orientation.getM01() + frameRot.getM21() * orientation.getM11() + frameRot.getM22() * orientation.getM21();
         m22 = frameRot.getM20() * orientation.getM02() + frameRot.getM21() * orientation.getM12() + frameRot.getM22() * orientation.getM22();
      }

      double positionX = frameRot.getM00() * position.getX() + frameRot.getM01() * position.getY() + frameRot.getM02() * position.getZ() + framePos.getX();
      double positionY = frameRot.getM10() * position.getX() + frameRot.getM11() * position.getY() + frameRot.getM12() * position.getZ() + framePos.getY();
      double positionZ = frameRot.getM20() * position.getX() + frameRot.getM21() * position.getY() + frameRot.getM22() * position.getZ() + framePos.getZ();

      double halfSizeX = 0.5 * size.getX();
      double halfSizeY = 0.5 * size.getY();
      double halfSizeZ = 0.5 * size.getZ();
      double xRange = Math.abs(m00) * halfSizeX + Math.abs(m01) * halfSizeY + Math.abs(m02) * halfSizeZ;
      double yRange = Math.abs(m10) * halfSizeX + Math.abs(m11) * halfSizeY + Math.abs(m12) * halfSizeZ;
      double zRange = Math.abs(m20) * halfSizeX + Math.abs(m21) * halfSizeY + Math.abs(m22) * halfSizeZ;

      double maxX = positionX + xRange;
      double maxY = positionY + yRange;
      double maxZ = positionZ + zRange;
      double minX = positionX - xRange;
      double minY = positionY - yRange;
      double minZ = positionZ - zRange;

      boundingBoxToPack.set(minX, minY, minZ, maxX, maxY, maxZ);
   }

   public static void boundingBox3D(ReferenceFrame frame, Capsule3DReadOnly capsule3D, BoundingBox3DBasics boundingBoxToPack)
   {
      if (frame.isRootFrame())
      {
         capsule3D.getBoundingBox(boundingBoxToPack);
         return;
      }

      RigidBodyTransform transformToRoot = frame.getTransformToRoot();
      Vector3DBasics framePos = transformToRoot.getTranslation();
      RotationMatrixBasics frameRot = transformToRoot.getRotation();

      if (frameRot.isIdentity())
      {
         capsule3D.getBoundingBox(boundingBoxToPack);
         boundingBoxToPack.getMinPoint().add(framePos);
         boundingBoxToPack.getMaxPoint().add(framePos);
         return;
      }

      double halfLength = capsule3D.getHalfLength();
      double radius = capsule3D.getRadius();
      Vector3DReadOnly axis = capsule3D.getAxis();
      Point3DReadOnly position = capsule3D.getPosition();

      double m00 = frameRot.getM00();
      double m01 = frameRot.getM01();
      double m02 = frameRot.getM02();
      double m10 = frameRot.getM10();
      double m11 = frameRot.getM11();
      double m12 = frameRot.getM12();
      double m20 = frameRot.getM20();
      double m21 = frameRot.getM21();
      double m22 = frameRot.getM22();

      double axisX = m00 * axis.getX() + m01 * axis.getY() + m02 * axis.getZ();
      double axisY = m10 * axis.getX() + m11 * axis.getY() + m12 * axis.getZ();
      double axisZ = m20 * axis.getX() + m21 * axis.getY() + m22 * axis.getZ();

      double positionX = m00 * position.getX() + m01 * position.getY() + m02 * position.getZ() + framePos.getX();
      double positionY = m10 * position.getX() + m11 * position.getY() + m12 * position.getZ() + framePos.getY();
      double positionZ = m20 * position.getX() + m21 * position.getY() + m22 * position.getZ() + framePos.getZ();

      double maxX = Math.abs(halfLength * axisX) + radius;
      double maxY = Math.abs(halfLength * axisY) + radius;
      double maxZ = Math.abs(halfLength * axisZ) + radius;

      double minX = -maxX + positionX;
      double minY = -maxY + positionY;
      double minZ = -maxZ + positionZ;
      maxX += positionX;
      maxY += positionY;
      maxZ += positionZ;

      boundingBoxToPack.set(minX, minY, minZ, maxX, maxY, maxZ);
   }

   public static void boundingBox3D(ReferenceFrame frame, Cylinder3DReadOnly cylinder3D, BoundingBox3DBasics boundingBoxToPack)
   {
      if (frame.isRootFrame())
      {
         cylinder3D.getBoundingBox(boundingBoxToPack);
         return;
      }

      RigidBodyTransform transformToRoot = frame.getTransformToRoot();
      Vector3DBasics framePos = transformToRoot.getTranslation();
      RotationMatrixBasics frameRot = transformToRoot.getRotation();

      if (frameRot.isIdentity())
      {
         cylinder3D.getBoundingBox(boundingBoxToPack);
         boundingBoxToPack.getMinPoint().add(framePos);
         boundingBoxToPack.getMaxPoint().add(framePos);
         return;
      }

      double halfLength = cylinder3D.getHalfLength();
      double radius = cylinder3D.getRadius();
      Vector3DReadOnly axis = cylinder3D.getAxis();
      Point3DReadOnly position = cylinder3D.getPosition();

      double m00 = frameRot.getM00();
      double m01 = frameRot.getM01();
      double m02 = frameRot.getM02();
      double m10 = frameRot.getM10();
      double m11 = frameRot.getM11();
      double m12 = frameRot.getM12();
      double m20 = frameRot.getM20();
      double m21 = frameRot.getM21();
      double m22 = frameRot.getM22();

      double axisX = m00 * axis.getX() + m01 * axis.getY() + m02 * axis.getZ();
      double axisY = m10 * axis.getX() + m11 * axis.getY() + m12 * axis.getZ();
      double axisZ = m20 * axis.getX() + m21 * axis.getY() + m22 * axis.getZ();

      double positionX = m00 * position.getX() + m01 * position.getY() + m02 * position.getZ() + framePos.getX();
      double positionY = m10 * position.getX() + m11 * position.getY() + m12 * position.getZ() + framePos.getY();
      double positionZ = m20 * position.getX() + m21 * position.getY() + m22 * position.getZ() + framePos.getZ();

      double invNormSquared = 1.0 / axis.lengthSquared();
      double capMinMaxX = Math.max(0.0, radius * Math.sqrt(1.0 - axisX * axisX * invNormSquared));
      double capMinMaxY = Math.max(0.0, radius * Math.sqrt(1.0 - axisY * axisY * invNormSquared));
      double capMinMaxZ = Math.max(0.0, radius * Math.sqrt(1.0 - axisZ * axisZ * invNormSquared));

      double maxX = Math.abs(halfLength * axisX) + capMinMaxX;
      double maxY = Math.abs(halfLength * axisY) + capMinMaxY;
      double maxZ = Math.abs(halfLength * axisZ) + capMinMaxZ;

      double minX = -maxX + positionX;
      double minY = -maxY + positionY;
      double minZ = -maxZ + positionZ;
      maxX += positionX;
      maxY += positionY;
      maxZ += positionZ;

      boundingBoxToPack.set(minX, minY, minZ, maxX, maxY, maxZ);
   }

   public static void boundingBox3D(ReferenceFrame frame, Ellipsoid3DReadOnly ellipsoid3D, BoundingBox3DBasics boundingBoxToPack)
   {
      if (frame.isRootFrame())
      {
         ellipsoid3D.getBoundingBox(boundingBoxToPack);
         return;
      }

      RigidBodyTransform transformToRoot = frame.getTransformToRoot();
      Vector3DBasics framePos = transformToRoot.getTranslation();
      RotationMatrixBasics frameRot = transformToRoot.getRotation();

      if (frameRot.isIdentity())
      {
         ellipsoid3D.getBoundingBox(boundingBoxToPack);
         boundingBoxToPack.getMinPoint().add(framePos);
         boundingBoxToPack.getMaxPoint().add(framePos);
         return;
      }

      Vector3DReadOnly radii = ellipsoid3D.getRadii();
      Point3DReadOnly position = ellipsoid3D.getPosition();
      RotationMatrixReadOnly orientation = ellipsoid3D.getOrientation();

      double m00, m01, m02;
      double m10, m11, m12;
      double m20, m21, m22;

      if (orientation.isIdentity())
      {
         m00 = frameRot.getM00();
         m01 = frameRot.getM01();
         m02 = frameRot.getM02();
         m10 = frameRot.getM10();
         m11 = frameRot.getM11();
         m12 = frameRot.getM12();
         m20 = frameRot.getM20();
         m21 = frameRot.getM21();
         m22 = frameRot.getM22();
      }
      else
      {
         m00 = frameRot.getM00() * orientation.getM00() + frameRot.getM01() * orientation.getM10() + frameRot.getM02() * orientation.getM20();
         m01 = frameRot.getM00() * orientation.getM01() + frameRot.getM01() * orientation.getM11() + frameRot.getM02() * orientation.getM21();
         m02 = frameRot.getM00() * orientation.getM02() + frameRot.getM01() * orientation.getM12() + frameRot.getM02() * orientation.getM22();
         m10 = frameRot.getM10() * orientation.getM00() + frameRot.getM11() * orientation.getM10() + frameRot.getM12() * orientation.getM20();
         m11 = frameRot.getM10() * orientation.getM01() + frameRot.getM11() * orientation.getM11() + frameRot.getM12() * orientation.getM21();
         m12 = frameRot.getM10() * orientation.getM02() + frameRot.getM11() * orientation.getM12() + frameRot.getM12() * orientation.getM22();
         m20 = frameRot.getM20() * orientation.getM00() + frameRot.getM21() * orientation.getM10() + frameRot.getM22() * orientation.getM20();
         m21 = frameRot.getM20() * orientation.getM01() + frameRot.getM21() * orientation.getM11() + frameRot.getM22() * orientation.getM21();
         m22 = frameRot.getM20() * orientation.getM02() + frameRot.getM21() * orientation.getM12() + frameRot.getM22() * orientation.getM22();
      }

      double rx = radii.getX() * radii.getX();
      double ry = radii.getY() * radii.getY();
      double rz = radii.getZ() * radii.getZ();

      double xRange = Math.sqrt(m00 * m00 * rx + m01 * m01 * ry + m02 * m02 * rz);
      double yRange = Math.sqrt(m10 * m10 * rx + m11 * m11 * ry + m12 * m12 * rz);
      double zRange = Math.sqrt(m20 * m20 * rx + m21 * m21 * ry + m22 * m22 * rz);

      double positionX = frameRot.getM00() * position.getX() + frameRot.getM01() * position.getY() + frameRot.getM02() * position.getZ() + framePos.getX();
      double positionY = frameRot.getM10() * position.getX() + frameRot.getM11() * position.getY() + frameRot.getM12() * position.getZ() + framePos.getY();
      double positionZ = frameRot.getM20() * position.getX() + frameRot.getM21() * position.getY() + frameRot.getM22() * position.getZ() + framePos.getZ();

      double maxX = positionX + xRange;
      double maxY = positionY + yRange;
      double maxZ = positionZ + zRange;
      double minX = positionX - xRange;
      double minY = positionY - yRange;
      double minZ = positionZ - zRange;

      boundingBoxToPack.set(minX, minY, minZ, maxX, maxY, maxZ);
   }

   public static void boundingBox3D(ReferenceFrame frame, PointShape3DReadOnly pointShape3D, BoundingBox3DBasics boundingBoxToPack)
   {
      if (frame.isRootFrame())
      {
         pointShape3D.getBoundingBox(boundingBoxToPack);
         return;
      }

      boundingBoxToPack.getMinPoint().set(pointShape3D);
      frame.transformFromThisToDesiredFrame(frame.getRootFrame(), boundingBoxToPack.getMinPoint());
      boundingBoxToPack.getMaxPoint().set(boundingBoxToPack.getMinPoint());
   }

   public static void boundingBox3D(ReferenceFrame frame, Ramp3DReadOnly ramp3D, BoundingBox3DBasics boundingBoxToPack)
   {
      if (frame.isRootFrame())
      {
         ramp3D.getBoundingBox(boundingBoxToPack);
         return;
      }

      RigidBodyTransform transformToRoot = frame.getTransformToRoot();
      Vector3DBasics framePos = transformToRoot.getTranslation();
      RotationMatrixBasics frameRot = transformToRoot.getRotation();

      if (frameRot.isIdentity())
      {
         ramp3D.getBoundingBox(boundingBoxToPack);
         boundingBoxToPack.getMinPoint().add(framePos);
         boundingBoxToPack.getMaxPoint().add(framePos);
         return;
      }

      Point3DReadOnly position = ramp3D.getPosition();
      RotationMatrixReadOnly orientation = ramp3D.getOrientation();

      double minX = Double.POSITIVE_INFINITY;
      double minY = Double.POSITIVE_INFINITY;
      double minZ = Double.POSITIVE_INFINITY;
      double maxX = Double.NEGATIVE_INFINITY;
      double maxY = Double.NEGATIVE_INFINITY;
      double maxZ = Double.NEGATIVE_INFINITY;
      double sizeX = ramp3D.getSizeX();
      double halfSizeY = 0.5 * ramp3D.getSizeY();
      double sizeZ = ramp3D.getSizeZ();

      double m00, m01, m02;
      double m10, m11, m12;
      double m20, m21, m22;

      if (orientation.isIdentity())
      {
         m00 = frameRot.getM00();
         m01 = frameRot.getM01();
         m02 = frameRot.getM02();
         m10 = frameRot.getM10();
         m11 = frameRot.getM11();
         m12 = frameRot.getM12();
         m20 = frameRot.getM20();
         m21 = frameRot.getM21();
         m22 = frameRot.getM22();
      }
      else
      {
         m00 = frameRot.getM00() * orientation.getM00() + frameRot.getM01() * orientation.getM10() + frameRot.getM02() * orientation.getM20();
         m01 = frameRot.getM00() * orientation.getM01() + frameRot.getM01() * orientation.getM11() + frameRot.getM02() * orientation.getM21();
         m02 = frameRot.getM00() * orientation.getM02() + frameRot.getM01() * orientation.getM12() + frameRot.getM02() * orientation.getM22();
         m10 = frameRot.getM10() * orientation.getM00() + frameRot.getM11() * orientation.getM10() + frameRot.getM12() * orientation.getM20();
         m11 = frameRot.getM10() * orientation.getM01() + frameRot.getM11() * orientation.getM11() + frameRot.getM12() * orientation.getM21();
         m12 = frameRot.getM10() * orientation.getM02() + frameRot.getM11() * orientation.getM12() + frameRot.getM12() * orientation.getM22();
         m20 = frameRot.getM20() * orientation.getM00() + frameRot.getM21() * orientation.getM10() + frameRot.getM22() * orientation.getM20();
         m21 = frameRot.getM20() * orientation.getM01() + frameRot.getM21() * orientation.getM11() + frameRot.getM22() * orientation.getM21();
         m22 = frameRot.getM20() * orientation.getM02() + frameRot.getM21() * orientation.getM12() + frameRot.getM22() * orientation.getM22();
      }

      for (int i = 0; i < 6; i++)
      {
         double xLocal = (i & 2) == 0 ? sizeX : 0.0;
         double yLocal = (i & 1) == 0 ? halfSizeY : -halfSizeY;
         double zLocal = (i & 4) == 0 ? 0.0 : sizeZ;

         double xRoot = m00 * xLocal + m01 * yLocal + m02 * zLocal;
         double yRoot = m10 * xLocal + m11 * yLocal + m12 * zLocal;
         double zRoot = m20 * xLocal + m21 * yLocal + m22 * zLocal;

         minX = Math.min(minX, xRoot);
         minY = Math.min(minY, yRoot);
         minZ = Math.min(minZ, zRoot);
         maxX = Math.max(maxX, xRoot);
         maxY = Math.max(maxY, yRoot);
         maxZ = Math.max(maxZ, zRoot);
      }

      double positionX = frameRot.getM00() * position.getX() + frameRot.getM01() * position.getY() + frameRot.getM02() * position.getZ() + framePos.getX();
      double positionY = frameRot.getM10() * position.getX() + frameRot.getM11() * position.getY() + frameRot.getM12() * position.getZ() + framePos.getY();
      double positionZ = frameRot.getM20() * position.getX() + frameRot.getM21() * position.getY() + frameRot.getM22() * position.getZ() + framePos.getZ();

      minX += positionX;
      minY += positionY;
      minZ += positionZ;
      maxX += positionX;
      maxY += positionY;
      maxZ += positionZ;

      boundingBoxToPack.set(minX, minY, minZ, maxX, maxY, maxZ);
   }

   public static void boundingBox3D(ReferenceFrame frame, Sphere3DReadOnly sphere3D, BoundingBox3DBasics boundingBoxToPack)
   {
      if (frame.isRootFrame())
      {
         sphere3D.getBoundingBox(boundingBoxToPack);
         return;
      }

      RigidBodyTransform transformToRoot = frame.getTransformToRoot();
      RotationMatrixBasics frameRot = transformToRoot.getRotation();
      Vector3DReadOnly framePos = transformToRoot.getTranslation();

      if (frameRot.isIdentity())
      {
         sphere3D.getBoundingBox(boundingBoxToPack);
         boundingBoxToPack.getMinPoint().add(framePos);
         boundingBoxToPack.getMaxPoint().add(framePos);
         return;
      }

      Point3DReadOnly position = sphere3D.getPosition();
      double radius = sphere3D.getRadius();

      double m00 = frameRot.getM00();
      double m01 = frameRot.getM01();
      double m02 = frameRot.getM02();
      double m10 = frameRot.getM10();
      double m11 = frameRot.getM11();
      double m12 = frameRot.getM12();
      double m20 = frameRot.getM20();
      double m21 = frameRot.getM21();
      double m22 = frameRot.getM22();

      double positionX = m00 * position.getX() + m01 * position.getY() + m02 * position.getZ() + framePos.getX();
      double positionY = m10 * position.getX() + m11 * position.getY() + m12 * position.getZ() + framePos.getY();
      double positionZ = m20 * position.getX() + m21 * position.getY() + m22 * position.getZ() + framePos.getZ();

      double minX = positionX - radius;
      double minY = positionY - radius;
      double minZ = positionZ - radius;
      double maxX = positionX + radius;
      double maxY = positionY + radius;
      double maxZ = positionZ + radius;
      boundingBoxToPack.set(minX, minY, minZ, maxX, maxY, maxZ);
   }

   public static void boundingBox3D(ReferenceFrame frame, ConvexPolytope3DReadOnly convexPolytope3D, BoundingBox3DBasics boundingBoxToPack)
   {
      if (frame.isRootFrame() || convexPolytope3D.isEmpty())
      {
         convexPolytope3D.getBoundingBox(boundingBoxToPack);
         return;
      }

      RigidBodyTransform transformToRoot = frame.getTransformToRoot();
      RotationMatrixBasics frameRot = transformToRoot.getRotation();
      Vector3DReadOnly framePos = transformToRoot.getTranslation();

      if (frameRot.isIdentity())
      {
         convexPolytope3D.getBoundingBox(boundingBoxToPack);
         boundingBoxToPack.getMinPoint().add(framePos);
         boundingBoxToPack.getMaxPoint().add(framePos);
         return;
      }

      boundingBox3D(frame, (SupportingVertexHolder) convexPolytope3D, boundingBoxToPack);
   }

   public static void boundingBox3D(ReferenceFrame frame, SupportingVertexHolder supportingVertexHolder, BoundingBox3DBasics boundingBoxToPack)
   {
      double minX = Double.POSITIVE_INFINITY;
      double minY = Double.POSITIVE_INFINITY;
      double minZ = Double.POSITIVE_INFINITY;
      double maxX = Double.NEGATIVE_INFINITY;
      double maxY = Double.NEGATIVE_INFINITY;
      double maxZ = Double.NEGATIVE_INFINITY;

      RigidBodyTransform transformToRoot = frame.getTransformToRoot();
      RotationMatrixBasics frameRot = transformToRoot.getRotation();
      Vector3DReadOnly framePos = transformToRoot.getTranslation();

      Point3DReadOnly supportingVertex;
      Vector3D supportDirection = new Vector3D();

      if (frameRot.isIdentity())
      {
         supportDirection.set(Axis3D.X);
         supportingVertex = supportingVertexHolder.getSupportingVertex(supportDirection);
         maxX = supportingVertex.getX();
         supportDirection.negate();
         supportingVertex = supportingVertexHolder.getSupportingVertex(supportDirection);
         minX = supportingVertex.getX();

         supportDirection.set(Axis3D.Y);
         supportingVertex = supportingVertexHolder.getSupportingVertex(supportDirection);
         maxY = supportingVertex.getY();
         supportDirection.negate();
         supportingVertex = supportingVertexHolder.getSupportingVertex(supportDirection);
         minY = supportingVertex.getY();

         supportDirection.set(Axis3D.Z);
         supportingVertex = supportingVertexHolder.getSupportingVertex(supportDirection);
         maxZ = supportingVertex.getZ();
         supportDirection.negate();
         supportingVertex = supportingVertexHolder.getSupportingVertex(supportDirection);
         minZ = supportingVertex.getZ();
      }
      else
      {
         frameRot.getRow(0, supportDirection);
         supportingVertex = supportingVertexHolder.getSupportingVertex(supportDirection);
         maxX = frameRot.getM00() * supportingVertex.getX() + frameRot.getM01() * supportingVertex.getY() + frameRot.getM02() * supportingVertex.getZ();
         supportDirection.negate();
         supportingVertex = supportingVertexHolder.getSupportingVertex(supportDirection);
         minX = frameRot.getM00() * supportingVertex.getX() + frameRot.getM01() * supportingVertex.getY() + frameRot.getM02() * supportingVertex.getZ();

         frameRot.getRow(1, supportDirection);
         supportingVertex = supportingVertexHolder.getSupportingVertex(supportDirection);
         maxY = frameRot.getM10() * supportingVertex.getX() + frameRot.getM11() * supportingVertex.getY() + frameRot.getM12() * supportingVertex.getZ();
         supportDirection.negate();
         supportingVertex = supportingVertexHolder.getSupportingVertex(supportDirection);
         minY = frameRot.getM10() * supportingVertex.getX() + frameRot.getM11() * supportingVertex.getY() + frameRot.getM12() * supportingVertex.getZ();

         frameRot.getRow(2, supportDirection);
         supportingVertex = supportingVertexHolder.getSupportingVertex(supportDirection);
         maxZ = frameRot.getM20() * supportingVertex.getX() + frameRot.getM21() * supportingVertex.getY() + frameRot.getM22() * supportingVertex.getZ();
         supportDirection.negate();
         supportingVertex = supportingVertexHolder.getSupportingVertex(supportDirection);
         minZ = frameRot.getM20() * supportingVertex.getX() + frameRot.getM21() * supportingVertex.getY() + frameRot.getM22() * supportingVertex.getZ();
      }

      maxX += framePos.getX();
      maxY += framePos.getY();
      maxZ += framePos.getZ();
      minX += framePos.getX();
      minY += framePos.getY();
      minZ += framePos.getZ();
      boundingBoxToPack.set(minX, minY, minZ, maxX, maxY, maxZ);
   }

   public static Box3D changeFrame(Box3DReadOnly original, ReferenceFrame from, ReferenceFrame to)
   {
      Box3D transformed = new Box3D(original);
      from.transformFromThisToDesiredFrame(to, transformed);
      return transformed;
   }

   public static Capsule3D changeFrame(Capsule3DReadOnly original, ReferenceFrame from, ReferenceFrame to)
   {
      Capsule3D transformed = new Capsule3D(original);
      from.transformFromThisToDesiredFrame(to, transformed);
      return transformed;
   }

   public static Cylinder3D changeFrame(Cylinder3DReadOnly original, ReferenceFrame from, ReferenceFrame to)
   {
      Cylinder3D transformed = new Cylinder3D(original);
      from.transformFromThisToDesiredFrame(to, transformed);
      return transformed;
   }

   public static Ellipsoid3D changeFrame(Ellipsoid3DReadOnly original, ReferenceFrame from, ReferenceFrame to)
   {
      Ellipsoid3D transformed = new Ellipsoid3D(original);
      from.transformFromThisToDesiredFrame(to, transformed);
      return transformed;
   }

   public static PointShape3D changeFrame(PointShape3DReadOnly original, ReferenceFrame from, ReferenceFrame to)
   {
      PointShape3D transformed = new PointShape3D(original);
      from.transformFromThisToDesiredFrame(to, transformed);
      return transformed;
   }

   public static Ramp3D changeFrame(Ramp3DReadOnly original, ReferenceFrame from, ReferenceFrame to)
   {
      Ramp3D transformed = new Ramp3D(original);
      from.transformFromThisToDesiredFrame(to, transformed);
      return transformed;
   }

   public static Sphere3D changeFrame(Sphere3DReadOnly original, ReferenceFrame from, ReferenceFrame to)
   {
      Sphere3D transformed = new Sphere3D(original);
      from.transformFromThisToDesiredFrame(to, transformed);
      return transformed;
   }

   static interface FrameChanger<A extends Shape3DReadOnly>
   {
      A changeFrame(A original, ReferenceFrame from, ReferenceFrame to);
   }
}
