package us.ihmc.robotics.geometry.shapes;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.triangleIsoscelesHeight;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.GeometryTools;

public class STPShape3DTools
{
   /**
    * <pre>
    * r = h
    *      r^2 - g^2 - 0.25 * l<sub>max</sub>
    * R = ------------------------
    *           2 * (r - g)
    * </pre>
    * 
    * where:
    * <ul>
    * <li><tt>R</tt> is {@link #largeRadius}
    * <li><tt>r</tt> is {@link #smallRadius}
    * <li><tt>h</tt> is {@link #minimumMargin}
    * <li><tt>g</tt> is {@link #maximumMargin}
    * <li><tt>l<sub>max</max></tt> is the maximum edge length that needs to be covered by the large
    * bounding sphere.
    * </ul>
    */
   public static double computeLargeRadiusFromMargins(double minimumMargin, double maximumMargin, double maximumEdgeLengthSquared)
   {
      double safeMaximumMargin = maximumMargin;

      if (EuclidCoreTools.square(maximumMargin - minimumMargin) > 0.25 * maximumEdgeLengthSquared)
      {
         safeMaximumMargin = 0.99 * (0.5 * EuclidCoreTools.squareRoot(maximumEdgeLengthSquared) + minimumMargin);
         LogTools.error("Unachievable margins, modified maximumMargin from: {}, down to: {}.", maximumMargin, safeMaximumMargin);
      }

      double smallRadius = minimumMargin;
      double smallRadiusSquared = EuclidCoreTools.square(smallRadius);
      double maximumMarginSquared = EuclidCoreTools.square(safeMaximumMargin);
      double largeRadius = smallRadiusSquared - maximumMarginSquared - 0.25 * maximumEdgeLengthSquared;
      largeRadius /= 2.0 * (smallRadius - safeMaximumMargin);
      return largeRadius;
   }

   public static double computeBox3DMaximumEdgeLengthSquared(Vector3DReadOnly box3DSize)
   {
      return EuclidCoreTools.max(EuclidCoreTools.normSquared(box3DSize.getX(), box3DSize.getY()),
                                 EuclidCoreTools.normSquared(box3DSize.getX(), box3DSize.getZ()),
                                 EuclidCoreTools.normSquared(box3DSize.getY(), box3DSize.getZ()));
   }
   
   public static class STPBox3DSupportingVertexCalculator
   {
      private final Vector3D halfSize = new Vector3D();
      private final UnitVector3D supportUnitDirection = new UnitVector3D();
      private final Point3D faceSphereCenter = new Point3D();
      private final Point3D closestVertexCenter = new Point3D();

      private final Point3D faceCenter = new Point3D();
      private final Point3D edgeCenter = new Point3D();

      public boolean getSupportingVertex(Box3DReadOnly box3D, double smallRadius, double largeRadius, Vector3DReadOnly supportDirection,
                                         Point3DBasics supportingVertexToPack)
      {
         if (box3D.getPose().hasRotation())
         {
            box3D.getPose().inverseTransform(supportDirection, supportUnitDirection);
         }
         else
            supportUnitDirection.set(supportDirection);
         supportDirection = supportUnitDirection;

         halfSize.setAndScale(0.5, box3D.getSize());

         double faceXPlusDot = supportDirection.getX();
         double faceYPlusDot = supportDirection.getY();
         double faceZPlusDot = supportDirection.getZ();

         double faceXAbsDot = Math.abs(faceXPlusDot);
         double faceYAbsDot = Math.abs(faceYPlusDot);
         double faceZAbsDot = Math.abs(faceZPlusDot);

         boolean isFaceXMaxCloser = faceXPlusDot > 0.0;
         boolean isFaceYMaxCloser = faceYPlusDot > 0.0;
         boolean isFaceZMaxCloser = faceZPlusDot > 0.0;

         Axis3D firstClosestFace;
         boolean isMaxFace;

         if (faceXAbsDot > faceYAbsDot)
         {
            if (faceXAbsDot > faceZAbsDot)
            { // Closest is one of the 2 x-faces
               firstClosestFace = Axis3D.X;
               isMaxFace = isFaceXMaxCloser;
            }
            else
            { // Closest is one of the 2 z-faces
               firstClosestFace = Axis3D.Z;
               isMaxFace = isFaceZMaxCloser;
            }
         }
         else if (faceYAbsDot > faceZAbsDot)
         { // Closest is one of the 2 y-faces
            firstClosestFace = Axis3D.Y;
            isMaxFace = isFaceYMaxCloser;
         }
         else
         { // Closest is one of the 2 z-faces
            firstClosestFace = Axis3D.Z;
            isMaxFace = isFaceZMaxCloser;
         }

         double faceSphereOffset = getFaceSphereOffset(firstClosestFace, halfSize, smallRadius, largeRadius);
         faceCenter.setToZero();
         faceSphereCenter.setToZero();
         faceCenter.setElement(firstClosestFace, halfSize.getElement(firstClosestFace));
         faceSphereCenter.setElement(firstClosestFace, halfSize.getElement(firstClosestFace) - faceSphereOffset);

         if (!isMaxFace)
         {
            faceCenter.negate();
            faceSphereCenter.negate();
         }

         EuclidShapeTools.supportingVertexSphere3D(supportDirection, faceSphereCenter, largeRadius, supportingVertexToPack);

         if (!isFaceSphereSupportingVertexValid(supportingVertexToPack, firstClosestFace, faceSphereOffset, halfSize))
         {
            closestVertexCenter.set(isFaceXMaxCloser ? halfSize.getX() : -halfSize.getX(),
                                    isFaceYMaxCloser ? halfSize.getY() : -halfSize.getY(),
                                    isFaceZMaxCloser ? halfSize.getZ() : -halfSize.getZ());

            EuclidShapeTools.supportingVertexSphere3D(supportDirection, closestVertexCenter, smallRadius, supportingVertexToPack);

            /*
             * Scale factor deduced from Thales similar triangle theorem and given that the edge's torus, which
             * tube radius is largeRadius, meets the vertex smallRadius.
             */
            double limitScaleFactor = largeRadius / (largeRadius - smallRadius);

            for (Axis3D axis : Axis3D.values)
            { // We look at whether the supportingVertex actually belongs to the torus of a neighboring edge.
               if (Math.abs(supportingVertexToPack.getElement(axis)) < halfSize.getElement(axis) * limitScaleFactor)
               {
                  edgeCenter.setElement(axis, 0.0);
                  edgeCenter.setElement(axis.next(), closestVertexCenter.getElement(axis.next()));
                  edgeCenter.setElement(axis.previous(), closestVertexCenter.getElement(axis.previous()));

                  double torusRadius = triangleIsoscelesHeight(largeRadius - smallRadius, box3D.getSize().getElement(axis));
                  EuclidShapeTools.innerSupportingVertexTorus3D(supportDirection, edgeCenter, axis, torusRadius, largeRadius, supportingVertexToPack);
                  break;
               }
            }
         }

         box3D.transformToWorld(supportingVertexToPack);

         return true;
      }

      private static boolean isFaceSphereSupportingVertexValid(Point3DReadOnly query, Axis3D face, double faceSphereOffset, Vector3DReadOnly box3DHalfSize)
      {
         /*
          * Testing that the query is inside the pyramid which apex is located at the face's large sphere
          * center and legs are extended to infinity while going through each face's vertices. The comparison
          * is done by testing that building one at a time each side plane of the pyramid and testing that
          * the face's center and the query are on the same side of the plane. In addition, the tests are
          * performed in the positive octant of the box only, this way, the number of tests is reduced to 2
          * instead of 4: only the 2 positive sides of the pyramid have to be tested for.
          */
         double queryAbsX = Math.abs(query.getX());
         double queryAbsY = Math.abs(query.getY());
         double queryAbsZ = Math.abs(query.getZ());

         double faceCenterX = face == Axis3D.X ? box3DHalfSize.getX() : 0.0;
         double faceCenterY = face == Axis3D.Y ? box3DHalfSize.getY() : 0.0;
         double faceCenterZ = face == Axis3D.Z ? box3DHalfSize.getZ() : 0.0;

         double faceSphereCenterX = face == Axis3D.X ? box3DHalfSize.getX() - faceSphereOffset : 0.0;
         double faceSphereCenterY = face == Axis3D.Y ? box3DHalfSize.getY() - faceSphereOffset : 0.0;
         double faceSphereCenterZ = face == Axis3D.Z ? box3DHalfSize.getZ() - faceSphereOffset : 0.0;

         // Testing against first side
         double edgeCenterX = faceCenterX + (face.next() == Axis3D.X ? box3DHalfSize.getX() : 0.0);
         double edgeCenterY = faceCenterY + (face.next() == Axis3D.Y ? box3DHalfSize.getY() : 0.0);
         double edgeCenterZ = faceCenterZ + (face.next() == Axis3D.Z ? box3DHalfSize.getZ() : 0.0);

         double sphereCenterToEdgeX = faceSphereCenterX - edgeCenterX;
         double sphereCenterToEdgeY = faceSphereCenterY - edgeCenterY;
         double sphereCenterToEdgeZ = faceSphereCenterZ - edgeCenterZ;

         double edgeAxisX = face.previous().getX();
         double edgeAxisY = face.previous().getY();
         double edgeAxisZ = face.previous().getZ();

         if (!GeometryTools.arePoint3DsSameSideOfPlane3D(faceCenterX,
                                                         faceCenterY,
                                                         faceCenterZ,
                                                         queryAbsX,
                                                         queryAbsY,
                                                         queryAbsZ,
                                                         edgeCenterX,
                                                         edgeCenterY,
                                                         edgeCenterZ,
                                                         edgeAxisX,
                                                         edgeAxisY,
                                                         edgeAxisZ,
                                                         sphereCenterToEdgeX,
                                                         sphereCenterToEdgeY,
                                                         sphereCenterToEdgeZ))
            return false;

         // Testing against second side
         edgeCenterX = faceCenterX + (face.previous() == Axis3D.X ? box3DHalfSize.getX() : 0.0);
         edgeCenterY = faceCenterY + (face.previous() == Axis3D.Y ? box3DHalfSize.getY() : 0.0);
         edgeCenterZ = faceCenterZ + (face.previous() == Axis3D.Z ? box3DHalfSize.getZ() : 0.0);

         sphereCenterToEdgeX = faceSphereCenterX - edgeCenterX;
         sphereCenterToEdgeY = faceSphereCenterY - edgeCenterY;
         sphereCenterToEdgeZ = faceSphereCenterZ - edgeCenterZ;

         edgeAxisX = face.next().getX();
         edgeAxisY = face.next().getY();
         edgeAxisZ = face.next().getZ();

         if (!GeometryTools.arePoint3DsSameSideOfPlane3D(faceCenterX,
                                                         faceCenterY,
                                                         faceCenterZ,
                                                         queryAbsX,
                                                         queryAbsY,
                                                         queryAbsZ,
                                                         edgeCenterX,
                                                         edgeCenterY,
                                                         edgeCenterZ,
                                                         edgeAxisX,
                                                         edgeAxisY,
                                                         edgeAxisZ,
                                                         sphereCenterToEdgeX,
                                                         sphereCenterToEdgeY,
                                                         sphereCenterToEdgeZ))
            return false;

         return true;
      }

      private static double getFaceSphereOffset(Axis3D face, Vector3DReadOnly box3DHalfSize, double smallRadius, double largeRadius)
      {
         double a = box3DHalfSize.getElement(face.next());
         double b = box3DHalfSize.getElement(face.previous());
         return Math.sqrt(EuclidCoreTools.square(largeRadius - smallRadius) - (a * a + b * b));
      }
   }

   public static class STPCapsule3DSupportingVertexCalculator
   {
      private final Vector3D orthogonalToAxis = new Vector3D();
      private final Point3D sideSphereCenter = new Point3D();
      private final Point3D edgeSphereCenter = new Point3D();

      public boolean getSupportingVertex(Capsule3DReadOnly capsule3D, double smallRadius, double largeRadius, Vector3DReadOnly supportDirection,
                                         Point3DBasics supportingVertexToPack)
      {
         double capsuleHalfLength = capsule3D.getHalfLength();
         UnitVector3DReadOnly capsuleAxis = capsule3D.getAxis();
         Point3DReadOnly capsulePosition = capsule3D.getPosition();

         orthogonalToAxis.set(supportDirection);

         double dot = supportDirection.dot(capsuleAxis);
         double sign = dot > 0.0 ? 1.0 : -1.0;
         orthogonalToAxis.setAndScale(dot, capsuleAxis);
         orthogonalToAxis.sub(supportDirection, orthogonalToAxis);
         edgeSphereCenter.setAndScale(sign * capsuleHalfLength, capsuleAxis);

         double distanceSquaredFromAxis = orthogonalToAxis.lengthSquared();

         if (distanceSquaredFromAxis < EuclidShapeTools.MIN_DISTANCE_EPSILON)
         {
            sideSphereCenter.setToNaN();
            edgeSphereCenter.setToNaN();
         }
         else
         {
            orthogonalToAxis.scale(1.0 / EuclidCoreTools.squareRoot(distanceSquaredFromAxis));

            double sideSphereRadius = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, capsule3D.getLength());
            sideSphereCenter.setAndScale(-sideSphereRadius, orthogonalToAxis);
         }

         if (!getSideSupportingVertex(capsule3D, supportDirection, supportingVertexToPack, smallRadius, largeRadius))
         {
            EuclidShapeTools.supportingVertexSphere3D(supportDirection, edgeSphereCenter, smallRadius, supportingVertexToPack);
         }

         supportingVertexToPack.add(capsulePosition);

         return true;
      }

      private boolean getSideSupportingVertex(Capsule3DReadOnly capsule3D, Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack,
                                              double smallRadius, double largeRadius)
      {
         if (sideSphereCenter.containsNaN())
            return false;
         EuclidShapeTools.supportingVertexSphere3D(supportDirection, sideSphereCenter, largeRadius, supportingVertexToPack);
         double dotMax = capsule3D.getHalfLength() * largeRadius / (largeRadius - smallRadius);
         double dot = TupleTools.dot(supportingVertexToPack, capsule3D.getAxis());
         return Math.abs(dot) <= Math.abs(dotMax);
      }
   }
}
