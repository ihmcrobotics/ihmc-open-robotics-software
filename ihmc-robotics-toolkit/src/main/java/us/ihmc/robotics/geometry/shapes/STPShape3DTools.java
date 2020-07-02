package us.ihmc.robotics.geometry.shapes;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.triangleIsoscelesHeight;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BiConsumer;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
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

   public static double computeCylinder3DMaximumEdgeLengthSquared(double length, double radius)
   {
      double maximumEdgeLengthSquared = Math.max(length, 2.0 * radius);
      return maximumEdgeLengthSquared * maximumEdgeLengthSquared;
   }

   public static double computeConvexPolytope3DMaximumEdgeLengthSquared(ConvexPolytope3DReadOnly convexPolytope3D)
   {
      double maximumEdgeLengthSquared = 0.0;

      for (int faceIndex = 0; faceIndex < convexPolytope3D.getNumberOfFaces(); faceIndex++)
      {
         Face3DReadOnly face = convexPolytope3D.getFace(faceIndex);
         Vertex3DReadOnly firstVertex = face.getVertex(0);

         for (int vertexIndex = 1; vertexIndex < face.getNumberOfEdges(); vertexIndex++)
         {
            maximumEdgeLengthSquared = Math.max(firstVertex.distanceSquared(face.getVertex(vertexIndex)), maximumEdgeLengthSquared);
         }
      }
      return maximumEdgeLengthSquared;
   }

   public static double computeRamp3DMaximumEdgeLengthSquared(Vector3DReadOnly ramp3DSize)
   {
      double rampLengthSquared = EuclidCoreTools.normSquared(ramp3DSize.getX(), ramp3DSize.getZ());
      return EuclidCoreTools.max(EuclidCoreTools.normSquared(ramp3DSize.getX(), ramp3DSize.getY()),
                                 EuclidCoreTools.normSquared(ramp3DSize.getY(), ramp3DSize.getZ()),
                                 rampLengthSquared + EuclidCoreTools.square(ramp3DSize.getY()));
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

         if (!getSideSupportingVertex(capsule3D, smallRadius, largeRadius, supportDirection, supportingVertexToPack))
         {
            EuclidShapeTools.supportingVertexSphere3D(supportDirection, edgeSphereCenter, smallRadius, supportingVertexToPack);
         }

         supportingVertexToPack.add(capsulePosition);

         return true;
      }

      private boolean getSideSupportingVertex(Capsule3DReadOnly capsule3D, double smallRadius, double largeRadius, Vector3DReadOnly supportDirection,
                                              Point3DBasics supportingVertexToPack)
      {
         if (sideSphereCenter.containsNaN())
            return false;
         EuclidShapeTools.supportingVertexSphere3D(supportDirection, sideSphereCenter, largeRadius, supportingVertexToPack);
         double dotMax = capsule3D.getHalfLength() * largeRadius / (largeRadius - smallRadius);
         double dot = TupleTools.dot(supportingVertexToPack, capsule3D.getAxis());
         return Math.abs(dot) <= Math.abs(dotMax);
      }
   }

   public static class STPCylinder3DSupportingVertexCalculator
   {
      private final Vector3D orthogonalToAxis = new Vector3D();
      private final Point3D sideSphereCenter = new Point3D();
      private final Point3D edgeSphereCenter = new Point3D();
      private final Point3D capSphereCenter = new Point3D();

      public boolean getSupportingVertex(Cylinder3DReadOnly cylinder3D, double smallRadius, double largeRadius, Vector3DReadOnly supportDirection,
                                         Point3DBasics supportingVertexToPack)
      {
         double cylinderRadius = cylinder3D.getRadius();
         double cylinderHalfLength = cylinder3D.getHalfLength();
         UnitVector3DReadOnly cylinderAxis = cylinder3D.getAxis();
         Point3DReadOnly cylinderPosition = cylinder3D.getPosition();

         orthogonalToAxis.set(supportDirection);

         double dot = supportDirection.dot(cylinderAxis);
         double sign = dot > 0.0 ? 1.0 : -1.0;
         orthogonalToAxis.setAndScale(dot, cylinderAxis);
         orthogonalToAxis.sub(supportDirection, orthogonalToAxis);

         double distanceSquaredFromAxis = orthogonalToAxis.lengthSquared();

         if (distanceSquaredFromAxis < EuclidShapeTools.MIN_DISTANCE_EPSILON)
         {
            sideSphereCenter.setToNaN();
            edgeSphereCenter.setToNaN();
         }
         else
         {
            orthogonalToAxis.scale(1.0 / EuclidCoreTools.squareRoot(distanceSquaredFromAxis));

            double sideSphereRadius = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, cylinder3D.getLength());
            sideSphereCenter.setAndScale(cylinderRadius - sideSphereRadius, orthogonalToAxis);

            edgeSphereCenter.setAndScale(cylinderRadius, orthogonalToAxis);
            edgeSphereCenter.scaleAdd(sign * cylinderHalfLength, cylinderAxis, edgeSphereCenter);
         }

         double capSphereRadius = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, 2.0 * cylinderRadius);
         capSphereCenter.setAndScale(sign * (cylinderHalfLength - capSphereRadius), cylinderAxis);

         if (!getSideSupportingVertex(cylinder3D, largeRadius, supportDirection, supportingVertexToPack))
         {
            if (!getCapSupportingVertex(cylinder3D, largeRadius, supportDirection, supportingVertexToPack))
            {
               EuclidShapeTools.supportingVertexSphere3D(supportDirection, edgeSphereCenter, smallRadius, supportingVertexToPack);
            }
         }

         supportingVertexToPack.add(cylinderPosition);

         return true;
      }

      private boolean getSideSupportingVertex(Cylinder3DReadOnly cylinder3D, double largeRadius, Vector3DReadOnly supportDirection,
                                              Point3DBasics supportingVertexToPack)
      {
         if (sideSphereCenter.containsNaN())
            return false;
         EuclidShapeTools.supportingVertexSphere3D(supportDirection, sideSphereCenter, largeRadius, supportingVertexToPack);
         double dot = TupleTools.dot(supportingVertexToPack, cylinder3D.getAxis());
         return dot <= cylinder3D.getHalfLength() && dot >= -cylinder3D.getHalfLength();
      }

      private final Point3D validationPoint = new Point3D();

      private boolean getCapSupportingVertex(Cylinder3DReadOnly cylinder3D, double largeRadius, Vector3DReadOnly supportDirection,
                                             Point3DBasics supportingVertexToPack)
      {
         EuclidShapeTools.supportingVertexSphere3D(supportDirection, capSphereCenter, largeRadius, supportingVertexToPack);
         double dot = TupleTools.dot(supportingVertexToPack, cylinder3D.getAxis());
         validationPoint.setAndScale(dot, cylinder3D.getAxis());
         validationPoint.sub(supportingVertexToPack, validationPoint);
         return validationPoint.distanceFromOriginSquared() <= cylinder3D.getRadius() * cylinder3D.getRadius();
      }
   }

   public static class STPConvexPolytope3DSupportingVertexCalculator
   {
      private final Point3D faceSphereCenter = new Point3D();
      private final Point3D edgeTorusCenter = new Point3D();
      private final Vector3D edgeTorusAxis = new Vector3D();

      public boolean getSupportingVertex(ConvexPolytope3DReadOnly convexPolytope3D, double smallRadius, double largeRadius, Vector3DReadOnly supportDirection,
                                         Point3DBasics supportingVertexToPack)
      {
         Vertex3DReadOnly bestVertex = convexPolytope3D.getSupportingVertex(supportDirection);

         if (bestVertex == null)
            return false;

         List<STPFace3D> bestSTPFaces = findBestFace(supportDirection, bestVertex);

         if (getFaceSupportingVertex(supportDirection, bestSTPFaces, bestVertex, smallRadius, largeRadius, supportingVertexToPack))
            return true;

         if (getBestEdgeSupportingVertex(supportDirection, bestSTPFaces, bestVertex, smallRadius, largeRadius, supportingVertexToPack))
            return true;

         EuclidShapeTools.supportingVertexSphere3D(supportDirection, bestVertex, smallRadius, supportingVertexToPack);
         return true;
      }

      private List<STPFace3D> findBestFace(Vector3DReadOnly supportDirection, Vertex3DReadOnly bestVertex)
      {
         Face3DReadOnly bestFace = bestVertex.getAssociatedEdge(0).getFace();
         double bestFaceDot = bestFace.getNormal().dot(supportDirection);

         for (int i = 1; i < bestVertex.getNumberOfAssociatedEdges(); i++)
         {
            HalfEdge3DReadOnly candidateEdge = bestVertex.getAssociatedEdge(i);
            Face3DReadOnly candidateFace = candidateEdge.getFace();
            double candidateDot = candidateFace.getNormal().dot(supportDirection);

            if (candidateDot > bestFaceDot)
            {
               bestFaceDot = candidateDot;
               bestFace = candidateFace;
            }
         }

         return STPFace3D.newSTPFace3Ds(bestFace);
      }

      private boolean getFaceSupportingVertex(Vector3DReadOnly supportDirection, List<STPFace3D> bestFaces, Vertex3DReadOnly bestVertex, double smallRadius,
                                              double largeRadius, Point3DBasics supportingVertexToPack)
      {
         for (STPFace3D stpFace : bestFaces)
         {
            if (!stpFace.contains(bestVertex))
               continue;

            if (getTriangleSupportingVertex(supportDirection, stpFace, smallRadius, largeRadius, supportingVertexToPack))
               return true;
         }
         return false;
      }

      private boolean getTriangleSupportingVertex(Vector3DReadOnly supportDirection, STPFace3D face, double smallRadius, double largeRadius,
                                                  Point3DBasics supportingVertexToPack)
      {
         EuclidGeometryTools.sphere3DPositionFromThreePoints(face.v0, face.v1, face.v2, largeRadius - smallRadius, faceSphereCenter);
         EuclidShapeTools.supportingVertexSphere3D(supportDirection, faceSphereCenter, largeRadius, supportingVertexToPack);
         return face.isPointDirectlyAboveOrBelow(supportingVertexToPack);
      }

      private boolean getBestEdgeSupportingVertex(Vector3DReadOnly supportDirection, List<STPFace3D> bestFaces, Vertex3DReadOnly bestVertex, double smallRadius,
                                                  double largeRadius, Point3DBasics supportingVertexToPack)
      {
         STPHalfEdge3D bestEdge = null;
         double bestEdgeDot = Double.NEGATIVE_INFINITY;

         for (STPFace3D face : bestFaces)
         {
            if (!face.contains(bestVertex))
               continue;

            for (STPHalfEdge3D edge : face.getEdges())
            {
               if (!edge.contains(bestVertex))
                  continue;

               double candidateDot = TupleTools.dot(edge.midpoint(), supportDirection);

               if (candidateDot > bestEdgeDot)
               {
                  bestEdgeDot = candidateDot;
                  bestEdge = edge;
               }
            }
         }

         return getEdgeSupportingVertex(supportDirection, bestEdge, smallRadius, largeRadius, supportingVertexToPack);
      }

      private boolean getEdgeSupportingVertex(Vector3DReadOnly supportDirection, STPHalfEdge3D edge, double smallRadius, double largeRadius,
                                              Point3DBasics supportingVertexToPack)
      {
         edgeTorusCenter.add(edge.getFirstEndpoint(), edge.getSecondEndpoint());
         edgeTorusCenter.scale(0.5);
         edgeTorusAxis.sub(edge.getSecondEndpoint(), edge.getFirstEndpoint());

         double radius = largeRadius - smallRadius;
         double torusRadius = Math.sqrt(radius * radius - 0.25 * edgeTorusAxis.lengthSquared());
         double torusTubeRadius = largeRadius;

         EuclidShapeTools.innerSupportingVertexTorus3D(supportDirection, edgeTorusCenter, edgeTorusAxis, torusRadius, torusTubeRadius, supportingVertexToPack);

         if (!edge.isBetweenEndpoints(supportingVertexToPack))
            return false;
         if (edge.face.isPointDirectlyAboveOrBelow(supportingVertexToPack))
            return false;
         if (edge.twin != null && edge.twin.face.isPointDirectlyAboveOrBelow(supportingVertexToPack))
            return false;
         return true;
      }

      private static class STPFace3D
      {
         private final Face3DReadOnly owner;
         private final Vertex3DReadOnly v0, v1, v2;
         private final STPHalfEdge3D e0, e1, e2;
         private final List<STPHalfEdge3D> edges;

         public static List<STPFace3D> newSTPFace3Ds(Face3DReadOnly owner)
         {
            List<STPFace3D> faces = new ArrayList<>();

            STPFace3D previousFace = new STPFace3D(owner, owner.getVertex(0), owner.getVertex(1), owner.getVertex(2));
            faces.add(previousFace);

            for (int i = 1; i < owner.getNumberOfEdges() - 2; i++)
            {
               Vertex3DReadOnly v0 = owner.getVertex(0);
               Vertex3DReadOnly v1 = owner.getVertex(i + 1);
               Vertex3DReadOnly v2 = owner.getVertex(i + 2);
               STPFace3D currentFace = new STPFace3D(owner, v0, v1, v2);

               currentFace.e0.twin = previousFace.e2;
               previousFace.e2.twin = currentFace.e0;

               faces.add(currentFace);
            }

            return faces;
         }

         public STPFace3D(Face3DReadOnly face3D, Vertex3DReadOnly v0, Vertex3DReadOnly v1, Vertex3DReadOnly v2)
         {
            owner = face3D;
            this.v0 = v0;
            this.v1 = v1;
            this.v2 = v2;
            e0 = new STPHalfEdge3D(v0, v1);
            e1 = new STPHalfEdge3D(v1, v2);
            e2 = new STPHalfEdge3D(v2, v0);
            e0.face = this;
            e1.face = this;
            e2.face = this;
            edges = Arrays.asList(e0, e1, e2);
         }

         public boolean contains(Vertex3DReadOnly query)
         {
            return v0 == query || v1 == query || v2 == query;
         }

         /**
          * Tests whether the query is located directly above or below this face, such its projection would
          * be located inside this face.
          *
          * @param query the coordinates of the query. Not modified.
          * @return {@code true} if the query is located either directly above or below this face,
          *         {@code false} otherwise.
          */
         public boolean isPointDirectlyAboveOrBelow(Point3DReadOnly query)
         {
            if (canObserverSeeEdge(query, e0))
               return false;
            if (canObserverSeeEdge(query, e1))
               return false;
            if (canObserverSeeEdge(query, e2))
               return false;
            return true;
         }

         public boolean canObserverSeeEdge(Point3DReadOnly observer, LineSegment3DReadOnly edge)
         {
            return EuclidPolytopeTools.isPoint3DOnLeftSideOfLine3D(observer, edge.getFirstEndpoint(), edge.getSecondEndpoint(), owner.getNormal());
         }

         public List<STPHalfEdge3D> getEdges()
         {
            return edges;
         }
      }

      private static class STPHalfEdge3D implements LineSegment3DReadOnly
      {
         private Vertex3DReadOnly origin, destination;
         private STPFace3D face;
         private STPHalfEdge3D twin;

         public STPHalfEdge3D(Vertex3DReadOnly origin, Vertex3DReadOnly destination)
         {
            this.origin = origin;
            this.destination = destination;
         }

         public boolean contains(Vertex3DReadOnly query)
         {
            return origin == query || destination == query;
         }

         @Override
         public Vertex3DReadOnly getFirstEndpoint()
         {
            return origin;
         }

         @Override
         public Vertex3DReadOnly getSecondEndpoint()
         {
            return destination;
         }
      }
   }

   public static class STPRamp3DSupportingVertexCalculator
   {
      private final UnitVector3D supportDirectionLocal = new UnitVector3D();
      private final Point3D faceSphereCenter = new Point3D();
      private final Point3D closestVertexCenter = new Point3D();

      private final Vector3D limitPlaneNormal = new Vector3D();
      private final Vector3D toFaceCentroid = new Vector3D();

      private final Point3D closestFaceEdgeCenter0 = new Point3D();
      private final Point3D closestFaceEdgeCenter1 = new Point3D();
      private final Point3D closestFaceEdgeCenter2 = new Point3D();
      private final Point3D[] closestFaceEdgeCenters = {closestFaceEdgeCenter0, closestFaceEdgeCenter1, closestFaceEdgeCenter2};
      private final Vector3D closestFaceEdgeAxis0 = new Vector3D();
      private final Vector3D closestFaceEdgeAxis1 = new Vector3D();
      private final Vector3D closestFaceEdgeAxis2 = new Vector3D();
      private final Vector3D[] closestFaceEdgeAxes = {closestFaceEdgeAxis0, closestFaceEdgeAxis1, closestFaceEdgeAxis2};
      private final double[] closestFaceEdgeLengths = {0, 0, 0};

      private enum Face
      {
         RAMP((shape3D, centroid) -> centroid.set(0.5 * shape3D.getSizeX(), 0.0, 0.5 * shape3D.getSizeZ())),
         X_MAX((shape3D, centroid) -> centroid.set(shape3D.getSizeX(), 0.0, 0.5 * shape3D.getSizeZ())),
         Y_MIN((shape3D, centroid) -> centroid.set(shape3D.getCentroid().getX(), -0.5 * shape3D.getSizeY(), shape3D.getCentroid().getZ())),
         Y_MAX((shape3D, centroid) -> centroid.set(shape3D.getCentroid().getX(), 0.5 * shape3D.getSizeY(), shape3D.getCentroid().getZ())),
         Z_MIN((shape3D, centroid) -> centroid.set(0.5 * shape3D.getSizeX(), 0.0, 0.0));

         private final BiConsumer<Ramp3DReadOnly, Tuple3DBasics> faceCentroidCalculator;

         private Face(BiConsumer<Ramp3DReadOnly, Tuple3DBasics> faceCentroidCalculator)
         {
            this.faceCentroidCalculator = faceCentroidCalculator;
         }

         public void getFaceCentroid(Ramp3DReadOnly shape3D, Tuple3DBasics faceCentroid)
         {
            faceCentroidCalculator.accept(shape3D, faceCentroid);
         }
      }

      public boolean getSupportingVertex(Ramp3DReadOnly ramp3D, double smallRadius, double largeRadius, Vector3DReadOnly supportDirection,
                                         Point3DBasics supportingVertexToPack)
      {
         if (ramp3D.getPose().hasRotation())
            ramp3D.getPose().inverseTransform(supportDirection, supportDirectionLocal);
         else
            supportDirectionLocal.set(supportDirection);
         supportDirection = supportDirectionLocal;

         Vector3DReadOnly rampSurfaceNormal = ramp3D.getRampSurfaceNormal();
         Vector3DReadOnly size = ramp3D.getSize();

         double rampDot = supportDirection.dot(rampSurfaceNormal);
         double faceXPlusDot = supportDirection.getX();
         double faceYPlusDot = supportDirection.getY();
         double faceZPlusDot = supportDirection.getZ();
         double faceYAbsDot = Math.abs(faceYPlusDot);
         boolean isFaceYMaxCloser = faceYPlusDot > 0.0;

         Face firstClosestFace;

         faceSphereCenter.setToZero();

         if (EuclidShapeTools.isFirstValueMaximum(faceXPlusDot, faceYAbsDot, -faceZPlusDot, rampDot))
         {
            firstClosestFace = Face.X_MAX;
            double sphereOffset = sphereOffset(ramp3D, smallRadius, largeRadius, firstClosestFace);
            faceSphereCenter.setX(size.getX() - sphereOffset);
            faceSphereCenter.setZ(0.5 * size.getZ());

            closestFaceEdgeLengths[0] = size.getZ();
            closestFaceEdgeLengths[1] = size.getY();
            closestFaceEdgeLengths[2] = size.getY();
            closestFaceEdgeAxis0.set(Axis3D.Z);
            closestFaceEdgeAxis1.set(Axis3D.Y);
            closestFaceEdgeAxis2.set(Axis3D.Y);
            closestFaceEdgeCenter0.set(size.getX(), isFaceYMaxCloser ? 0.5 * size.getY() : -0.5 * size.getY(), 0.5 * size.getZ());
            closestFaceEdgeCenter1.set(size.getX(), 0.0, 0.0);
            closestFaceEdgeCenter2.set(size.getX(), 0.0, size.getZ());
         }
         else if (EuclidShapeTools.isFirstValueMaximum(faceYAbsDot, -faceZPlusDot, rampDot))
         {
            firstClosestFace = isFaceYMaxCloser ? Face.Y_MAX : Face.Y_MIN;
            double sphereOffset = sphereOffset(ramp3D, smallRadius, largeRadius, firstClosestFace);
            faceSphereCenter.setX(0.5 * size.getX());
            faceSphereCenter.setY(isFaceYMaxCloser ? 0.5 * size.getY() - sphereOffset : -0.5 * size.getY() + sphereOffset);
            faceSphereCenter.setZ(0.5 * size.getZ());

            closestFaceEdgeLengths[0] = size.getZ();
            closestFaceEdgeLengths[1] = size.getX();
            closestFaceEdgeLengths[2] = ramp3D.getRampLength();
            closestFaceEdgeAxis0.set(Axis3D.Z);
            closestFaceEdgeAxis1.set(Axis3D.X);
            closestFaceEdgeAxis2.set(rampSurfaceNormal.getZ(), 0.0, -rampSurfaceNormal.getX());
            closestFaceEdgeCenter0.set(size.getX(), isFaceYMaxCloser ? 0.5 * size.getY() : -0.5 * size.getY(), 0.5 * size.getZ());
            closestFaceEdgeCenter1.set(0.5 * size.getX(), isFaceYMaxCloser ? 0.5 * size.getY() : -0.5 * size.getY(), 0.0);
            closestFaceEdgeCenter2.set(0.5 * size.getX(), isFaceYMaxCloser ? 0.5 * size.getY() : -0.5 * size.getY(), 0.5 * size.getZ());
         }
         else if (-faceZPlusDot > rampDot)
         {
            firstClosestFace = Face.Z_MIN;
            double sphereOffset = sphereOffset(ramp3D, smallRadius, largeRadius, firstClosestFace);
            faceSphereCenter.setX(0.5 * size.getX());
            faceSphereCenter.setZ(sphereOffset);

            closestFaceEdgeLengths[0] = size.getY();
            closestFaceEdgeLengths[1] = size.getX();
            closestFaceEdgeLengths[2] = size.getY();
            closestFaceEdgeAxis0.set(Axis3D.Y);
            closestFaceEdgeAxis1.set(Axis3D.X);
            closestFaceEdgeAxis2.set(Axis3D.Y);
            closestFaceEdgeCenter0.set(size.getX(), 0.0, 0.0);
            closestFaceEdgeCenter1.set(0.5 * size.getX(), isFaceYMaxCloser ? 0.5 * size.getY() : -0.5 * size.getY(), 0.0);
            closestFaceEdgeCenter2.set(0.0, 0.0, 0.0);
         }
         else
         {
            firstClosestFace = Face.RAMP;
            double sphereOffset = sphereOffset(ramp3D, smallRadius, largeRadius, firstClosestFace);
            faceSphereCenter.setX(0.5 * size.getX());
            faceSphereCenter.setZ(0.5 * size.getZ());
            faceSphereCenter.scaleAdd(-sphereOffset, rampSurfaceNormal, faceSphereCenter);

            closestFaceEdgeLengths[0] = size.getY();
            closestFaceEdgeLengths[1] = ramp3D.getRampLength();
            closestFaceEdgeLengths[2] = size.getY();
            closestFaceEdgeAxis0.set(Axis3D.Y);
            closestFaceEdgeAxis1.set(rampSurfaceNormal.getZ(), 0.0, -rampSurfaceNormal.getX());
            closestFaceEdgeAxis2.set(Axis3D.Y);
            closestFaceEdgeCenter0.set(size.getX(), 0.0, size.getZ());
            closestFaceEdgeCenter1.set(0.5 * size.getX(), isFaceYMaxCloser ? 0.5 * size.getY() : -0.5 * size.getY(), 0.5 * size.getZ());
            closestFaceEdgeCenter2.set(0.0, 0.0, 0.0);
         }

         double limitScaleFactor = 0.5 * largeRadius / (largeRadius - smallRadius);
         firstClosestFace.getFaceCentroid(ramp3D, toFaceCentroid);
         toFaceCentroid.sub(faceSphereCenter);

         int isOutsideCounter = 0;
         boolean isWithinFace = true;
         boolean hasComputedSupportingVertex = false;

         for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
         {
            /*
             * For each edge we construct a plane which the edge and the faceSphereCenter belong to, then we
             * test for whether the face centroid and support direction are on the same side of the plane. If
             * not the supportDirection cannot be handled by the face. If the support direction violates 2+
             * edges and the support direction is not handled by either edge, then the support direction can
             * only be handled by a ramp's vertex.
             */
            Point3D edgeCenter = closestFaceEdgeCenters[edgeIndex];
            Vector3D edgeAxis = closestFaceEdgeAxes[edgeIndex];
            double edgeLength = closestFaceEdgeLengths[edgeIndex];

            limitPlaneNormal.sub(edgeCenter, faceSphereCenter);
            limitPlaneNormal.cross(edgeAxis);

            if (toFaceCentroid.dot(limitPlaneNormal) * supportDirection.dot(limitPlaneNormal) < 0.0)
            {
               /*
                * The face cannot handle the supportVertex, then maybe the edge is. Here we simply evaluate the
                * supportingVertex and test that it is within the limits.
                */
               isOutsideCounter++;
               isWithinFace = false;

               double torusRadius = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, edgeLength);
               EuclidShapeTools.innerSupportingVertexTorus3D(supportDirection, edgeCenter, edgeAxis, torusRadius, largeRadius, supportingVertexToPack);

               if (Math.abs(EuclidGeometryTools.percentageAlongLine3D(supportingVertexToPack, edgeCenter, edgeAxis)) <= edgeLength * limitScaleFactor)
               {
                  hasComputedSupportingVertex = true;
                  break;
               }

               if (isOutsideCounter >= 2)
               {
                  EuclidShapeTools.supportingVectexRamp3D(supportDirection, size, closestVertexCenter);
                  EuclidShapeTools.supportingVertexSphere3D(supportDirection, closestVertexCenter, smallRadius, supportingVertexToPack);
                  hasComputedSupportingVertex = true;
                  break;
               }
            }
         }

         if (!hasComputedSupportingVertex)
         {
            if (isWithinFace)
            {
               EuclidShapeTools.supportingVertexSphere3D(supportDirection, faceSphereCenter, largeRadius, supportingVertexToPack);
            }
            else
            {
               EuclidShapeTools.supportingVectexRamp3D(supportDirection, size, closestVertexCenter);
               EuclidShapeTools.supportingVertexSphere3D(supportDirection, closestVertexCenter, smallRadius, supportingVertexToPack);
            }
         }

         ramp3D.transformToWorld(supportingVertexToPack);

         return true;
      }

      private double sphereOffset(Ramp3DReadOnly ramp3D, double smallRadius, double largeRadius, Face face)
      {
         double a, b;
         double diagonalSquared;

         switch (face)
         {
            case RAMP:
               a = ramp3D.getRampLength();
               b = ramp3D.getSizeY();
               diagonalSquared = a * a + b * b;
               break;
            case X_MAX:
               a = ramp3D.getSizeY();
               b = ramp3D.getSizeZ();
               diagonalSquared = a * a + b * b;
               break;
            case Y_MAX:
            case Y_MIN:
               diagonalSquared = ramp3D.getRampLength() * ramp3D.getRampLength();
               break;
            case Z_MIN:
               a = ramp3D.getSizeX();
               b = ramp3D.getSizeY();
               diagonalSquared = a * a + b * b;
               break;
            default:
               throw new IllegalStateException();
         }
         double radius = largeRadius - smallRadius;
         return Math.sqrt(radius * radius - 0.25 * diagonalSquared);
      }
   }

   public static FrameBoundingBox3DReadOnly newLinkedFrameBoundingBox3DReadOnly(ReferenceFrameHolder referenceFrameHolder, Point3DReadOnly minPoint,
                                                                                Point3DReadOnly maxPoint)
   {
      FramePoint3DReadOnly linkedMinPoint = EuclidFrameFactories.newLinkedFramePoint3DReadOnly(referenceFrameHolder, minPoint);
      FramePoint3DReadOnly linkedMaxPoint = EuclidFrameFactories.newLinkedFramePoint3DReadOnly(referenceFrameHolder, maxPoint);

      return new FrameBoundingBox3DReadOnly()
      {
         @Override
         public FramePoint3DReadOnly getMinPoint()
         {
            return linkedMinPoint;
         }

         @Override
         public FramePoint3DReadOnly getMaxPoint()
         {
            return linkedMaxPoint;
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof FrameBoundingBox3DReadOnly)
               return equals((FrameBoundingBox3DReadOnly) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameBoundingBox3DString(this);
         }

         @Override
         public int hashCode()
         {
            return EuclidHashCodeTools.toIntHashCode(minPoint, maxPoint);
         }
      };
   }
}
