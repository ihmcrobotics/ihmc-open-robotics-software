package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;

public class GeometryTools
{
   public static double angleFromXForwardToVector2D(Vector2DReadOnly vector)
   {
      return EuclidGeometryTools.angleFromFirstToSecondVector2D(1.0, 0.0, vector.getX(), vector.getY());
   }

   private static final ThreadLocal<Point2D> tempIntersection = new ThreadLocal<Point2D>()
   {
      @Override
      public Point2D initialValue()
      {
         return new Point2D();
      }
   };

   /**
    * Clip each component of the given tuple to the axis-aligned bounding box. Each of the bounding box
    * minimum coordinates is defined as follows: {@code minX = x1 < x2 ? x1 : x2}. Each of the bounding
    * box maximum coordinates is defined as follows: {@code maxX = x1 > x2 ? x1 : x2}.
    * 
    * @param tupleToClip the 3D tuple to clip to the bounding box. Modified.
    * @param x1          minimum/maximum x coordinate of the bounding box.
    * @param x2          minimum/maximum x coordinate of the bounding box.
    * @param y1          minimum/maximum y coordinate of the bounding box.
    * @param y2          minimum/maximum y coordinate of the bounding box.
    * @param z1          minimum/maximum z coordinate of the bounding box.
    * @param z2          minimum/maximum z coordinate of the bounding box.
    */
   // FIXME this is rather unsafe, the user should know the difference between the minimum and maximum coordinates of the bounding box.
   public static void clipToBoundingBox(Tuple3DBasics tupleToClip, double x1, double x2, double y1, double y2, double z1, double z2)
   {
      tupleToClip.setX(x1 < x2 ? MathTools.clamp(tupleToClip.getX(), x1, x2) : MathTools.clamp(tupleToClip.getX(), x2, x1));
      tupleToClip.setY(y1 < y2 ? MathTools.clamp(tupleToClip.getY(), y1, y2) : MathTools.clamp(tupleToClip.getY(), y2, y1));
      tupleToClip.setZ(z1 < z2 ? MathTools.clamp(tupleToClip.getZ(), z1, z2) : MathTools.clamp(tupleToClip.getZ(), z2, z1));
   }


   /**
    * Attempts to normalize the given 3D vector. If the vector's length falls below
    * {@value Epsilons#ONE_TRILLIONTH}, the vector is set to (0, 0, 1).
    * 
    * @param vector the 3D vector to normalize. Modified.
    */
   public static void normalizeSafelyZUp(Vector3DBasics vector)
   {
      double distance = vector.norm();

      if (distance > Epsilons.ONE_TRILLIONTH)
      {
         vector.scale(1.0 / distance);
      }
      else
      {
         vector.set(0.0, 0.0, 1.0);
      }
   }

   /**
    * Change the frame and then project the result onto the XY-plane for each point in the given list
    * {@code points}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param referenceFrame the new reference frame the result will be expressed in.. Not modified.
    * @param points         the list of points to transform. Not modified.
    * @return the result of the transformation.
    */
   public static List<FramePoint2D> changeFrameAndProjectToXYPlane(ReferenceFrame referenceFrame, List<FramePoint3D> points)
   {
      List<FramePoint2D> ret = new ArrayList<>(points.size());

      for (int i = 0; i < points.size(); i++)
      {
         FramePoint3D framePoint = new FramePoint3D(points.get(i));
         framePoint.changeFrame(referenceFrame);

         ret.add(new FramePoint2D(framePoint));
      }

      return ret;
   }

   /**
    * Creates a new reference frame such that it is centered at the given {@code point} and with its
    * z-axis aligned with the given {@code zAxis} vector.
    * <p>
    * Note that the parent frame is set to the reference frame the given {@code point} and
    * {@code zAxis} are expressed in.
    * </p>
    *
    * @param frameName the name of the new frame.
    * @param point     location of the reference frame's origin. Not modified.
    * @param zAxis     orientation the reference frame's z-axis. Not modified.
    * @return the new reference frame.
    * @throws ReferenceFrameMismatchException if {@code point} and {@code zAxis} are not expressed in
    *                                         the same reference frame.
    */
   public static ReferenceFrame constructReferenceFrameFromPointAndZAxis(String frameName, FramePoint3D point, FrameVector3D zAxis)
   {
      return constructReferenceFrameFromPointAndAxis(frameName, point, Axis3D.Z, zAxis);
   }

   /**
    * Creates a new reference frame such that it is centered at the given {@code point} and with one of
    * its axes aligned with the given {@code alignAxisWithThis} vector.
    * <p>
    * Note that the parent frame is set to the reference frame the given {@code point} and
    * {@code alignAxisWithThis} are expressed in.
    * </p>
    *
    * @param frameName         the name of the new frame.
    * @param point             location of the reference frame's origin. Not modified.
    * @param axisToAlign       defines which axis of the new reference frame is to be aligned with the
    *                          given {@code alignAxisWithThis} vector.
    * @param alignAxisWithThis the vector to which the reference frame chosen axis should be aligned
    *                          with. Not modified.
    * @return the new reference frame.
    * @throws ReferenceFrameMismatchException if {@code point} and {@code alignAxisWithThis} are not
    *                                         expressed in the same reference frame.
    */
   public static ReferenceFrame constructReferenceFrameFromPointAndAxis(String frameName, FramePoint3DReadOnly point, Axis3D axisToAlign,
                                                                        FrameVector3DReadOnly alignAxisWithThis)
   {
      ReferenceFrame parentFrame = point.getReferenceFrame();
      alignAxisWithThis.checkReferenceFrameMatch(point.getReferenceFrame());

      RigidBodyTransform transformToDesired = new RigidBodyTransform();
      transformToDesired.getTranslation().set(point);
      EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(axisToAlign, alignAxisWithThis, transformToDesired.getRotation());

      return ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToDesired);
   }

   public static void pitchAboutPoint(FramePoint3DReadOnly pointToTransform, FramePoint3DReadOnly pointToPitchAbout, double pitch, FramePoint3D resultToPack)
   {
      pointToTransform.checkReferenceFrameMatch(pointToPitchAbout);
      double tempX = pointToTransform.getX() - pointToPitchAbout.getX();
      double tempY = pointToTransform.getY() - pointToPitchAbout.getY();
      double tempZ = pointToTransform.getZ() - pointToPitchAbout.getZ();

      double cosAngle = Math.cos(pitch);
      double sinAngle = Math.sin(pitch);

      double x = cosAngle * tempX + sinAngle * tempZ;
      tempZ = -sinAngle * tempX + cosAngle * tempZ;
      tempX = x;

      resultToPack.setIncludingFrame(pointToPitchAbout);
      resultToPack.add(tempX, tempY, tempZ);
   }

   /**
    * yawAboutPoint
    *
    * @param pointToYawAbout FramePoint
    * @param yaw             double
    * @return CartesianPositionFootstep
    */
   public static void yawAboutPoint(FramePoint3DReadOnly pointToTransform, FramePoint3DReadOnly pointToYawAbout, double yaw, FramePoint3D resultToPack)
   {
      pointToTransform.checkReferenceFrameMatch(pointToYawAbout);
      double tempX = pointToTransform.getX() - pointToYawAbout.getX();
      double tempY = pointToTransform.getY() - pointToYawAbout.getY();
      double tempZ = pointToTransform.getZ() - pointToYawAbout.getZ();

      double cosAngle = Math.cos(yaw);
      double sinAngle = Math.sin(yaw);

      double x = cosAngle * tempX + -sinAngle * tempY;
      tempY = sinAngle * tempX + cosAngle * tempY;
      tempX = x;

      resultToPack.setIncludingFrame(pointToYawAbout);
      resultToPack.add(tempX, tempY, tempZ);
   }

   /**
    * yawAboutPoint
    *
    * @param pointToYawAbout FramePoint2d
    * @param yaw             double
    * @return CartesianPositionFootstep
    */
   public static void yawAboutPoint(FramePoint2DReadOnly pointToTransform, FramePoint2DReadOnly pointToYawAbout, double yaw, FramePoint2D resultToPack)
   {
      pointToTransform.checkReferenceFrameMatch(pointToYawAbout);
      double tempX = pointToTransform.getX() - pointToYawAbout.getX();
      double tempY = pointToTransform.getY() - pointToYawAbout.getY();

      double cosAngle = Math.cos(yaw);
      double sinAngle = Math.sin(yaw);

      double x = cosAngle * tempX + -sinAngle * tempY;
      tempY = sinAngle * tempX + cosAngle * tempY;
      tempX = x;

      resultToPack.setIncludingFrame(pointToYawAbout);
      resultToPack.add(tempX, tempY);
   }

   public static void rotatePoseAboutAxis(ReferenceFrame rotationAxisFrame, Axis3D rotationAxis, double angle, FramePose3D framePoseToPack)
   {
      GeometryTools.rotatePoseAboutAxis(rotationAxisFrame, rotationAxis, angle, false, false, framePoseToPack);
   }

   public static void rotatePoseAboutAxis(ReferenceFrame rotationAxisFrame, Axis3D rotationAxis, double angle, boolean lockPosition, boolean lockOrientation,
                                          FramePose3D framePoseToPack)
   {
      ReferenceFrame initialFrame = framePoseToPack.getReferenceFrame();

      framePoseToPack.changeFrame(rotationAxisFrame);

      AxisAngle axisAngle = new AxisAngle(0.0, 0.0, 0.0, angle);
      axisAngle.setElement(rotationAxis.ordinal(), 1.0);

      if (!lockPosition)
      {
         Point3D newPosition = new Point3D(framePoseToPack.getPosition());
         axisAngle.transform(newPosition);
         framePoseToPack.getPosition().set(newPosition);
      }

      if (!lockOrientation)
      {
         Quaternion newOrientation = new Quaternion(framePoseToPack.getOrientation());
         axisAngle.transform(newOrientation);
         framePoseToPack.getOrientation().set(newOrientation);
      }

      framePoseToPack.changeFrame(initialFrame);
   }

   public static void rotatePoseAboutAxis(FrameVector3D rotatationAxis, FramePoint3D rotationAxisOrigin, double angle, FramePose3D framePoseToPack)
   {
      ReferenceFrame frameWhoseZAxisIsRotationAxis = constructReferenceFrameFromPointAndZAxis("rotationAxisFrame", rotationAxisOrigin, rotatationAxis);

      rotatePoseAboutAxis(frameWhoseZAxisIsRotationAxis, Axis3D.Z, angle, framePoseToPack);
   }

   /**
    * Creates a 3D Euclid Box (a Shape) out of a 3D Bounding Box. Allocates a new Box3D.
    *
    * @param boundingBox
    * @return box
    */
   public static Box3D convertBoundingBox3DToBox3D(BoundingBox3DReadOnly boundingBox)
   {
      Point3DReadOnly minPoint = boundingBox.getMinPoint();
      Point3DReadOnly maxPoint = boundingBox.getMaxPoint();

      Point3D boxCenter = new Point3D();
      boxCenter.interpolate(minPoint, maxPoint, 0.5);
      Vector3D size = new Vector3D();
      size.sub(maxPoint, minPoint);

      return new Box3D(boxCenter, new Quaternion(), size.getX(), size.getY(), size.getZ());
   }

   /**
    * Returns a boolean value stating whether the two given points, i.e. {@code firstQuery} and
    * {@code secondQuery}, are on the same side of a plane or separated by the plane.
    * <p>
    * The plane is defined by two points that belongs to it and a tangent, i.e. vector orthogonal to
    * the plane's normal.
    * </p>
    * 
    * @param firstQueryX         the x-coordinate of the first point to test. Not modified.
    * @param firstQueryY         the y-coordinate of the first point to test. Not modified.
    * @param firstQueryZ         the z-coordinate of the first point to test. Not modified.
    * @param secondQueryX        the x-coordinate of the second point to test. Not modified.
    * @param secondQueryY        the y-coordinate of the second point to test. Not modified.
    * @param secondQueryZ        the z-coordinate of the second point to test. Not modified.
    * @param pointOnPlaneX       the x-coordinate of a point that belongs to the plane. Not modified.
    * @param pointOnPlaneY       the y-coordinate of a point that belongs to the plane. Not modified.
    * @param pointOnPlaneZ       the z-coordinate of a point that belongs to the plane. Not modified.
    * @param planeFirstTangentX  the x-component of a first vector that is orthogonal to the plane's
    *                            normal. Not modified.
    * @param planeFirstTangentY  the y-component of a first vector that is orthogonal to the plane's
    *                            normal. Not modified.
    * @param planeFirstTangentZ  the z-component of a first vector that is orthogonal to the plane's
    *                            normal. Not modified.
    * @param planeSecondTangentX the x-component of a second vector that is orthogonal to the plane's
    *                            normal. Not modified.
    * @param planeSecondTangentY the y-component of a second vector that is orthogonal to the plane's
    *                            normal. Not modified.
    * @param planeSecondTangentZ the z-component of a second vector that is orthogonal to the plane's
    *                            normal. Not modified.
    * @return {@code true} if the two queries are on the same side of the plane,
    *         {@code false otherwise}.
    */
   public static boolean arePoint3DsSameSideOfPlane3D(double firstQueryX, double firstQueryY, double firstQueryZ, double secondQueryX, double secondQueryY,
                                                      double secondQueryZ, double pointOnPlaneX, double pointOnPlaneY, double pointOnPlaneZ,
                                                      double planeFirstTangentX, double planeFirstTangentY, double planeFirstTangentZ,
                                                      double planeSecondTangentX, double planeSecondTangentY, double planeSecondTangentZ)
   {

      double planeNormalX = planeFirstTangentY * planeSecondTangentZ - planeFirstTangentZ * planeSecondTangentY;
      double planeNormalY = planeFirstTangentZ * planeSecondTangentX - planeFirstTangentX * planeSecondTangentZ;
      double planeNormalZ = planeFirstTangentX * planeSecondTangentY - planeFirstTangentY * planeSecondTangentX;

      boolean isFirstQueryAbovePlane = EuclidGeometryTools.isPoint3DAboveOrBelowPlane3D(firstQueryX,
                                                                                        firstQueryY,
                                                                                        firstQueryZ,
                                                                                        pointOnPlaneX,
                                                                                        pointOnPlaneY,
                                                                                        pointOnPlaneZ,
                                                                                        planeNormalX,
                                                                                        planeNormalY,
                                                                                        planeNormalZ,
                                                                                        true);
      boolean isSecondQueryAbovePlane = EuclidGeometryTools.isPoint3DAboveOrBelowPlane3D(secondQueryX,
                                                                                         secondQueryY,
                                                                                         secondQueryZ,
                                                                                         pointOnPlaneX,
                                                                                         pointOnPlaneY,
                                                                                         pointOnPlaneZ,
                                                                                         planeNormalX,
                                                                                         planeNormalY,
                                                                                         planeNormalZ,
                                                                                         true);
      return isFirstQueryAbovePlane == isSecondQueryAbovePlane;
   }

   /**
    * Computes the intersection between two infinitely long 2D lines each defined by two 2D points.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting at
    * {@code firstPointOnLine1}.
    * </ul>
    * </p>
    * <p>
    * WARNING: the 3D arguments are projected onto the XY-plane to perform the actual computation in
    * 2D.
    * </p>
    *
    * @param firstPointOnLine1  the x and y coordinates are used to define a first 2D point on the
    *                           first line. Not modified.
    * @param secondPointOnLine1 the x and y coordinates are used to define a second 2D point on the
    *                           first line. Not modified.
    * @param firstPointOnLine2  the x and y coordinates are used to define a first 2D point on the
    *                           second line. Not modified.
    * @param secondPointOnLine2 the x and y coordinates are used to define a second 2D point on the
    *                           second line. Not modified.
    * @param intersectionToPack the result is stored in the x and y components of this 3D point.
    *                           Modified.
    * @return {@code true} if the two lines intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame, except for {@code intersectionToPack}.
    */
   // FIXME This method is too confusing and error prone.
   public static boolean getIntersectionBetweenTwoLines2d(FramePoint3DReadOnly firstPointOnLine1, FramePoint3DReadOnly secondPointOnLine1,
                                                          FramePoint3DReadOnly firstPointOnLine2, FramePoint3DReadOnly secondPointOnLine2,
                                                          FramePoint3D intersectionToPack)
   {
      firstPointOnLine1.checkReferenceFrameMatch(secondPointOnLine1);
      firstPointOnLine2.checkReferenceFrameMatch(secondPointOnLine2);
      firstPointOnLine1.checkReferenceFrameMatch(firstPointOnLine2);
      intersectionToPack.changeFrame(firstPointOnLine1.getReferenceFrame());

      double pointOnLine1x = firstPointOnLine1.getX();
      double pointOnLine1y = firstPointOnLine1.getY();
      double lineDirection1x = secondPointOnLine1.getX() - firstPointOnLine1.getX();
      double lineDirection1y = secondPointOnLine1.getY() - firstPointOnLine1.getY();
      double pointOnLine2x = firstPointOnLine2.getX();
      double pointOnLine2y = firstPointOnLine2.getY();
      double lineDirection2x = secondPointOnLine2.getX() - firstPointOnLine2.getX();
      double lineDirection2y = secondPointOnLine2.getY() - firstPointOnLine2.getY();

      boolean success = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(pointOnLine1x,
                                                                          pointOnLine1y,
                                                                          lineDirection1x,
                                                                          lineDirection1y,
                                                                          pointOnLine2x,
                                                                          pointOnLine2y,
                                                                          lineDirection2x,
                                                                          lineDirection2y,
                                                                          tempIntersection.get());

      if (!success)
         intersectionToPack.setToNaN();
      else
         intersectionToPack.set(tempIntersection.get().getX(), tempIntersection.get().getY(), intersectionToPack.getZ());
      return success;
   }

   /**
    * Computes the intersection between two infinitely long 2D lines each defined by a 2D point and a
    * 2D direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting at
    * {@code pointOnLine1}.
    * </ul>
    * </p>
    * <p>
    * WARNING: the 3D arguments are projected onto the XY-plane to perform the actual computation in
    * 2D.
    * </p>
    *
    * @param pointOnLine1       the x and y coordinates are used to define a 2D point on the first
    *                           line. Not modified.
    * @param lineDirection1     the x and y components are used to define the 2D direction of the first
    *                           line. Not modified.
    * @param pointOnLine2       the x and y coordinates are used to define a 2D point on the second
    *                           line. Not modified.
    * @param lineDirection2     the x and y components are used to define the 2D direction of the
    *                           second line. Not modified.
    * @param intersectionToPack the result is stored in the x and y components of this 3D point.
    *                           Modified.
    * @return {@code true} if the two lines intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame, except for {@code intersectionToPack}.
    */
   // FIXME This method is too confusing and error prone.
   public static boolean getIntersectionBetweenTwoLines2d(FramePoint3D pointOnLine1, FrameVector3D lineDirection1, FramePoint3D pointOnLine2,
                                                          FrameVector3D lineDirection2, FramePoint3D intersectionToPack)
   {
      pointOnLine1.checkReferenceFrameMatch(lineDirection1);
      pointOnLine2.checkReferenceFrameMatch(lineDirection2);
      pointOnLine1.checkReferenceFrameMatch(pointOnLine2);
      intersectionToPack.changeFrame(pointOnLine1.getReferenceFrame());

      double pointOnLine1x = pointOnLine1.getX();
      double pointOnLine1y = pointOnLine1.getY();
      double lineDirection1x = lineDirection1.getX();
      double lineDirection1y = lineDirection1.getY();
      double pointOnLine2x = pointOnLine2.getX();
      double pointOnLine2y = pointOnLine2.getY();
      double lineDirection2x = lineDirection2.getX();
      double lineDirection2y = lineDirection2.getY();

      boolean success = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(pointOnLine1x,
                                                                          pointOnLine1y,
                                                                          lineDirection1x,
                                                                          lineDirection1y,
                                                                          pointOnLine2x,
                                                                          pointOnLine2y,
                                                                          lineDirection2x,
                                                                          lineDirection2y,
                                                                          tempIntersection.get());

      if (!success)
         intersectionToPack.setToNaN();
      else
         intersectionToPack.set(tempIntersection.get().getX(), tempIntersection.get().getY(), intersectionToPack.getZ());
      return success;
   }

   /**
    * Computes the 2D vector perpendicular to the given 2D {@code vector} such that:
    * <ul>
    * <li>{@code vector2d.dot(perpendicularVector2d) == 0.0}.
    * <li>{@code vector2d.angle(perpendicularVector2d) == Math.PI / 2.0}.
    * </ul>
    * <p>
    * WARNING: the 3D arguments are projected onto the XY-plane to perform the actual computation in
    * 2D.
    * </p>
    *
    * @param vector                    the vector to compute in the xy-plane the perpendicular of. Not
    *                                  modified.
    * @param perpendicularVectorToPack a vector in which the x and y components of the 2D perpendicular
    *                                  vector are stored. Modified.
    */
   // FIXME this is just bad.
   public static void getPerpendicularVector2d(FrameVector3D vector, FrameVector3D perpendicularVectorToPack)
   {
      perpendicularVectorToPack.set(-vector.getY(), vector.getX(), perpendicularVectorToPack.getZ());
   }
}
