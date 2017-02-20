package us.ihmc.robotics.geometry.interfaces;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

/**
 * Interface to serve a pose, allowing to transform objects to
 * and from local pose frame.
 *
 * @author Duncan Calvert
 *
 */
public interface PoseTransform
{
   /**
    * Transforms the given {@code transformable} to world frame.
    *
    * @param transformable the transformable to transform. Modified.
    */
   void tranformToWorld(Transformable transformable);
   
   /**
    * Transforms the given {@code pointToTransform} to world frame.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates then translates a point.
    * <li>{@link QuaternionBasedTransform} rotates then translates a point.
    * <li>{@link AffineTransform} scales, rotates, then translates a point.
    * </ul>
    * </p>
    *
    * @param pointToTransform the point to transform. Modified.
    */
   default void transformToWorld(Point3DBasics pointToTransform)
   {
      transformToWorld(pointToTransform, pointToTransform);
   }

   /**
    * Transforms the given {@code pointOriginal} to world frame and stores the result in
    * {@code pointTransformed}.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates then translates a point.
    * <li>{@link QuaternionBasedTransform} rotates then translates a point.
    * <li>{@link AffineTransform} scales, rotates, then translates a point.
    * </ul>
    * </p>
    *
    * @param pointOriginal the point to transform. Not modified.
    * @param pointTransformed the point in which the result is stored. Modified.
    */
   void transformToWorld(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed);

   /**
    * Transforms the given {@code vectorToTransform} to world frame.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates a vector.
    * <li>{@link QuaternionBasedTransform} rotates a vector.
    * <li>{@link AffineTransform} scales then rotates a vector.
    * </ul>
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    */
   default void transformToWorld(Vector3DBasics vectorToTransform)
   {
      transformToWorld(vectorToTransform, vectorToTransform);
   }

   /**
    * Transforms the given {@code vectorOriginal} to world frame and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates a vector.
    * <li>{@link QuaternionBasedTransform} rotates a vector.
    * <li>{@link AffineTransform} scales then rotates a vector.
    * </ul>
    * </p>
    *
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   void transformToWorld(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed);

   /**
    * Transforms the given {@code quaternionToTransform} to world frame.
    * <p>
    * {@link RigidBodyTransform}, {@link QuaternionBasedTransform}, and {@link AffineTransform}
    * prepend their rotation part the given quaternion. No scale or translation is applied to the
    * quaternion such that the output of this method is still a unit-quaternion.
    * </p>
    *
    * @param quaternionToTransform the quaternion to transform. Modified.
    */
   default void transformToWorld(QuaternionBasics quaternionToTransform)
   {
      transformToWorld(quaternionToTransform, quaternionToTransform);
   }

   /**
    * Transforms the given {@code quaternionOriginal} to world frame and stores the result in
    * {@code quaternionTransformed}.
    * <p>
    * {@link RigidBodyTransform}, {@link QuaternionBasedTransform}, and {@link AffineTransform}
    * prepend their rotation part the given quaternion. No scale or translation is applied to the
    * quaternion such that the output of this method is still a unit-quaternion.
    * </p>
    *
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    */
   void transformToWorld(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed);

   /**
    * Transforms the vector part (x, y, z) of the given {@code vector4DToTransform} as a 3D vector
    * and translates it by {@code s} times the translation part of the transform. The scalar part
    * (s) remains unchanged.
    * <p>
    * Note that for {@code s = 0}, a 4D vector behaves as a 3D vector, and for {@code s = 1} it
    * behaves as a 3D point.
    * </p>
    * <p>
    * <li>{@link RigidBodyTransform} rotates then translates a vector.
    * <li>{@link QuaternionBasedTransform} rotates then translates a vector.
    * <li>{@link AffineTransform} scales, rotates, then translates a vector.
    * </p>
    *
    * @param vectorToTransform the 4D vector to transform. Modified.
    */
   default void transformToWorld(Vector4DBasics vectorToTransform)
   {
      transformToWorld(vectorToTransform, vectorToTransform);
   }

   /**
    * Transforms the vector part (x, y, z) of the given {@code vector4DOriginal} as a 3D vector and
    * translates it by {@code s} times the translation part of the transform. The scalar part (s)
    * remains unchanged.
    * <p>
    * Note that for {@code s = 0}, a 4D vector behaves as a 3D vector, and for {@code s = 1} it
    * behaves as a 3D point.
    * </p>
    * <p>
    * <li>{@link RigidBodyTransform} rotates then translates a vector.
    * <li>{@link QuaternionBasedTransform} rotates then translates a vector.
    * <li>{@link AffineTransform} scales, rotates, then translates a vector.
    * </p>
    *
    * @param vectorOriginal the 4D vector to transform. Not modified.
    * @param vectorTransformed the 4D vector in which the result is stored. Modified.
    */
   void transformToWorld(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed);

   /**
    * Transforms the given {@code point2DToTransform} to world frame.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates then translates a point.
    * <li>{@link QuaternionBasedTransform} rotates then translates a point.
    * <li>{@link AffineTransform} scales, rotates, then translates a point.
    * </ul>
    * </p>
    *
    * @param pointToTransform the point to transform. Modified.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   default void transformToWorld(Point2DBasics pointToTransform)
   {
      transformToWorld(pointToTransform, true);
   }

   /**
    * Transforms the given {@code point2DOriginal} to world frame and stores the result in
    * {@code point2DTransformed}.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates then translates a point.
    * <li>{@link QuaternionBasedTransform} rotates then translates a point.
    * <li>{@link AffineTransform} scales, rotates, then translates a point.
    * </ul>
    * </p>
    *
    * @param pointOriginal the point to transform. Not modified.
    * @param pointTransformed the point in which the result is stored. Modified.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   default void transformToWorld(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed)
   {
      transformToWorld(pointOriginal, pointTransformed, true);
   }

   /**
    * Transforms the given {@code point2DToTransform} to world frame.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates then translates a point.
    * <li>{@link QuaternionBasedTransform} rotates then translates a point.
    * <li>{@link AffineTransform} scales, rotates, then translates a point.
    * </ul>
    * </p>
    *
    * @param pointToTransform the point to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of
    *           this transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the rotation
    *            part of this transform is not a transformation in the XY plane.
    */
   default void transformToWorld(Point2DBasics pointToTransform, boolean checkIfTransformInXYPlane)
   {
      transformToWorld(pointToTransform, pointToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given {@code point2DOriginal} to world frame and stores the result in
    * {@code point2DTransformed}.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates then translates a point.
    * <li>{@link QuaternionBasedTransform} rotates then translates a point.
    * <li>{@link AffineTransform} scales, rotates, then translates a point.
    * </ul>
    * </p>
    *
    * @param pointOriginal the point to transform. Not modified.
    * @param pointTransformed the point in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of
    *           this transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the rotation
    *            part of this transform is not a transformation in the XY plane.
    */
   void transformToWorld(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane);

   /**
    * Transforms the given {@code vector2DToTransform} to world frame.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates a vector.
    * <li>{@link QuaternionBasedTransform} rotates a vector.
    * <li>{@link AffineTransform} scales then rotates a vector.
    * </ul>
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   default void transformToWorld(Vector2DBasics vectorToTransform)
   {
      transformToWorld(vectorToTransform, true);
   }

   /**
    * Transforms the given {@code vector2DOriginal} to world frame and stores the result in
    * {@code vector2DTransformed}.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates a vector.
    * <li>{@link QuaternionBasedTransform} rotates a vector.
    * <li>{@link AffineTransform} scales then rotates a vector.
    * </ul>
    * </p>
    *
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   default void transformToWorld(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed)
   {
      transformToWorld(vectorOriginal, vectorTransformed, true);
   }

   /**
    * Transforms the given {@code vector2DToTransform} to world frame.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates then translates a point.
    * <li>{@link QuaternionBasedTransform} rotates then translates a point.
    * <li>{@link AffineTransform} scales, rotates, then translates a point.
    * </ul>
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of
    *           this transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   default void transformToWorld(Vector2DBasics vectorToTransform, boolean checkIfTransformInXYPlane)
   {
      transformToWorld(vectorToTransform, vectorToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given {@code vector2DOriginal} to world frame and stores the result in
    * {@code vector2DTransformed}.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates then translates a point.
    * <li>{@link QuaternionBasedTransform} rotates then translates a point.
    * <li>{@link AffineTransform} scales, rotates, then translates a point.
    * </ul>
    * </p>
    *
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of
    *           this transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   void transformToWorld(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane);

   /**
    * Transforms the given {@code matrixToTransform} to world frame.
    * <p>
    * WARNING: <b> This is different from concatenating orientations.</b>
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates a matrix.
    * <li>{@link QuaternionBasedTransform} rotates a matrix.
    * <li>{@link AffineTransform} scales then rotates a matrix.
    * </ul>
    * </p>
    *
    * @param matrixToTransform the matrix to transform. Modified.
    */
   default void transformToWorld(Matrix3D matrixToTransform)
   {
      transformToWorld(matrixToTransform, matrixToTransform);
   }

   /**
    * Transforms the given {@code matrixOriginal} to world frame and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * WARNING: <b> This is different from concatenating orientations.</b>
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates a matrix.
    * <li>{@link QuaternionBasedTransform} rotates a matrix.
    * <li>{@link AffineTransform} scales then rotates a matrix.
    * </ul>
    * </p>
    *
    * @param matrixOriginal the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result in stored. Modified.
    */
   void transformToWorld(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed);

   /**
    * Transforms the given {@code rotationMatrix} to world frame.
    * <p>
    * {@link RigidBodyTransform}, {@link QuaternionBasedTransform}, and {@link AffineTransform}
    * prepend their rotation part the given rotation matrix. No scale or translation is applied to
    * the rotation matrix such that the output of this method is still a proper rotation matrix.
    * </p>
    *
    * @param matrixToTransform the rotation matrix to transform. Modified.
    */
   default void transformToWorld(RotationMatrix matrixToTransform)
   {
      transformToWorld(matrixToTransform, matrixToTransform);
   }

   /**
    * Transforms the given {@code matrixOriginal} to world frame and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * {@link RigidBodyTransform}, {@link QuaternionBasedTransform}, and {@link AffineTransform}
    * prepend their rotation part the given rotation matrix. No scale or translation is applied to
    * the rotation matrix such that the output of this method is still a proper rotation matrix.
    * </p>
    *
    * @param matrixOriginal the rotation matrix to transform. Not modified.
    * @param matrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   void transformToWorld(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed);

   /**
    * Transforms the given {@code rigidBodyTransformToTransform} to world frame.
    * <p>
    * The given transform is only rotated and translated, no scaling is applied.
    * </p>
    * 
    * @param rigidBodyTransformToTransform the rigid-body transform to transform. Modified.
    */
   default void transformToWorld(RigidBodyTransform rigidBodyTransformToTransform)
   {
      transformToWorld(rigidBodyTransformToTransform, rigidBodyTransformToTransform);
   }

   /**
    * Transforms the given {@code original} to world frame and stores the result in
    * {@code transformed}.
    * <p>
    * The given transform is only rotated and translated, no scaling is applied.
    * </p>
    * 
    * @param original the rigid-body transform to transform. Not modified.
    * @param transformed the rigid-body transform in which the result is stored. Modified.
    */
   void transformToWorld(RigidBodyTransform original, RigidBodyTransform transformed);

   /**
    * Transforms the given {@code quaternionBasedTransformToTransform} to world frame.
    * <p>
    * The given transform is only rotated and translated, no scaling is applied.
    * </p>
    * 
    * @param quaternionBasedTransformToTransform the quaternion based transform to transform.
    *           Modified.
    */
   default void transformToWorld(QuaternionBasedTransform quaternionBasedTransformToTransform)
   {
      transformToWorld(quaternionBasedTransformToTransform, quaternionBasedTransformToTransform);
   }

   /**
    * Transforms the given {@code original} to world frame and stores the result in
    * {@code transformed}.
    * <p>
    * The given transform is only rotated and translated, no scaling is applied.
    * </p>
    * 
    * @param original the quaternion based transform to transform. Not modified.
    * @param transformed the quaternion based transform in which the result is stored. Modified.
    */
   void transformToWorld(QuaternionBasedTransform original, QuaternionBasedTransform transformed);

   
   /**
    * Transforms the given {@code transformable} to world frame.
    *
    * @param transformable the transformable to transform. Modified.
    */
   void tranformToLocal(Transformable transformable);
   
   /**
    * Performs the transform to local frame on the given point {@code pointToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(Point3DBasics)} with the inverse of this
    * transform.
    * </p>
    *
    * @param pointToTransform the point to transform. Modified.
    */
   default void transformToLocal(Point3DBasics pointToTransform)
   {
      transformToLocal(pointToTransform, pointToTransform);
   }

   /**
    * Performs the transform to local frame on the given point {@code pointOriginal} and stores the
    * result in {@code pointTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(Point3DReadOnly, Point3DBasics)} with the
    * inverse of this transform.
    * </p>
    *
    * @param pointOriginal the point to transform. Not modified.
    * @param pointTransformed the point in which the result is stored. Modified.
    */
   void transformToLocal(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed);

   /**
    * Performs the transform to local frame on the given vector {@code vectorToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(Vector3DBasics)} with the inverse of this
    * transform.
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    */
   default void transformToLocal(Vector3DBasics vectorToTransform)
   {
      transformToLocal(vectorToTransform, vectorToTransform);
   }

   /**
    * Performs the transform to local frame on the given vector {@code vectorOriginal} and stores
    * the result in {@code vectorTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(Vector3DReadOnly, Vector3DBasics)} with the
    * inverse of this transform.
    * </p>
    *
    * @param vectorOriginal the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   void transformToLocal(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed);

   /**
    * Performs the transform to local frame on the given quaternion {@code quaternionToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(QuaternionBasics)} with the inverse of this
    * transform.
    * </p>
    *
    * @param quaternionToTransform the quaternion to transform. Modified.
    */
   default void transformToLocal(QuaternionBasics quaternionToTransform)
   {
      transformToLocal(quaternionToTransform, quaternionToTransform);
   }

   /**
    * Performs the transform to local frame on the given quaternion {@code quaternionOriginal} and
    * stores the result in {@code quaternionTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(QuaternionReadOnly, QuaternionBasics)} with
    * the inverse of this transform.
    * </p>
    *
    * @param quaternionOriginal the quaternion to transform. Not modified.
    * @param quaternionTransformed the quaternion in which the result is stored. Modified.
    */
   void transformToLocal(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed);

   /**
    * Performs the transform to local frame on the given vector {@code vectorToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(Vector4DBasics)} with the inverse of this
    * transform.
    * </p>
    *
    * @param vectorToTransform the 4D vector to transform. Modified.
    */
   default void transformToLocal(Vector4DBasics vectorToTransform)
   {
      transformToLocal(vectorToTransform, vectorToTransform);
   }

   /**
    * Performs the transform to local frame on the given vector {@code vectorOriginal} and stores
    * the result in {@code vectorTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(Vector4DReadOnly, Vector4DBasics)} with the
    * inverse of this transform.
    * </p>
    *
    * @param vectorOriginal the 4D vector to transform. Not modified.
    * @param vectorTransformed the 4D vector in which the result is stored. Modified.
    */
   void transformToLocal(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed);

   /**
    * Performs the transform to local frame on the given point {@code pointToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(Point2DBasics)} with the inverse of this
    * transform.
    * </p>
    *
    * @param point2DToTransform the point to transform. Modified.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   default void transformToLocal(Point2DBasics pointToTransform)
   {
      transformToLocal(pointToTransform, true);
   }

   /**
    * Performs the transform to local frame on the given point {@code pointOriginal} and stores the
    * result in {@code pointTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(Point2DReadOnly, Point2DBasics)} with the
    * inverse of this transform.
    * </p>
    *
    * @param point2DOriginal the point to transform. Not modified.
    * @param point2DTransformed the point in which the result is stored. Modified.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   default void transformToLocal(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed)
   {
      transformToLocal(pointOriginal, pointTransformed, true);
   }

   /**
    * Performs the transform to local frame on the given point {@code pointToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(Point2DBasics, boolean)} with the inverse of
    * this transform.
    * </p>
    *
    * @param point2DToTransform the point to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of
    *           this transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the rotation
    *            part of this transform is not a transformation in the XY plane.
    */
   default void transformToLocal(Point2DBasics pointToTransform, boolean checkIfTransformInXYPlane)
   {
      transformToLocal(pointToTransform, pointToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Performs the transform to local frame on the given point {@code pointOriginal} and stores the
    * result in {@code pointTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(Point2DReadOnly, Point2DBasics, boolean)} with
    * the inverse of this transform.
    * </p>
    *
    * @param point2DOriginal the point to transform. Not modified.
    * @param point2DTransformed the point in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of
    *           this transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the rotation
    *            part of this transform is not a transformation in the XY plane.
    */
   void transformToLocal(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane);

   /**
    * Performs the transform to local frame on the given vector {@code vectorToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(Vector2DBasics)} with the inverse of this
    * transform.
    * </p>
    *
    * @param vector2DToTransform the vector to transform. Modified.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   default void transformToLocal(Vector2DBasics vectorToTransform)
   {
      transformToLocal(vectorToTransform, true);
   }

   /**
    * Performs the transform to local frame on the given vector {@code vectorOriginal} and stores
    * the result in {@code vectorTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(Vector2DReadOnly, Vector2DBasics)} with the
    * inverse of this transform.
    * </p>
    *
    * @param vector2DOriginal the vector to transform. Not modified.
    * @param vector2DTransformed the vector in which the result is stored. Modified.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   default void transformToLocal(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed)
   {
      transformToLocal(vectorOriginal, vectorTransformed, true);
   }

   /**
    * Performs the transform to local frame on the given vector {@code vectorToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(Vector2DBasics, boolean)} with the inverse of
    * this transform.
    * </p>
    *
    * @param vector2DToTransform the vector to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of
    *           this transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   default void transformToLocal(Vector2DBasics vectorToTransform, boolean checkIfTransformInXYPlane)
   {
      transformToLocal(vectorToTransform, vectorToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Performs the transform to local frame on the given vector {@code vectorOriginal} and stores
    * the result in {@code vectorTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(Vector2DReadOnly, Vector2DBasics, boolean)}
    * with the inverse of this transform.
    * </p>
    *
    * @param vector2DOriginal the vector to transform. Not modified.
    * @param vector2DTransformed the vector in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of
    *           this transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if the rotation part of this transform is not a transformation
    *            in the XY plane.
    */
   void transformToLocal(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane);

   /**
    * Performs the transform to local frame on the given matrix {@code matrixToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(Matrix3D)} with the inverse of this transform.
    * </p>
    *
    * @param matrixToTransform the matrix to transform. Modified.
    */
   default void transformToLocal(Matrix3D matrixToTransform)
   {
      transformToLocal(matrixToTransform, matrixToTransform);
   }

   /**
    * Performs the transform to local frame on the given matrix {@code matrixOriginal} and stores
    * the result in {@code matrixTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(Matrix3DReadOnly, Matrix3D)} with the inverse
    * of this transform.
    * </p>
    *
    * @param matrixOriginal the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result in stored. Modified.
    */
   void transformToLocal(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed);

   /**
    * Performs the transform to local frame on the given rotation matrix {@code matrixToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(RotationMatrix)} with the inverse of this
    * transform.
    * </p>
    *
    * @param matrixToTransform the rotation matrix to transform. Modified.
    */
   default void transformToLocal(RotationMatrix matrixToTransform)
   {
      transformToLocal(matrixToTransform, matrixToTransform);
   }

   /**
    * Performs the transform to local frame on the given matrix {@code matrixOriginal} and stores
    * the result in {@code matrixTransformed}.
    * <p>
    * This is equivalent to calling {@link #transform(RotationMatrixReadOnly, RotationMatrix)} with
    * the inverse of this transform.
    * </p>
    *
    * @param matrixOriginal the rotation matrix to transform. Not modified.
    * @param matrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   void transformToLocal(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed);

   /**
    * Performs the transform to local frame on the given {@code rigidBodyTransformToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(RigidBodyTransform)} with the inverse of this
    * transform.
    * </p>
    * <p>
    * The given transform is only rotated and translated, no scaling is applied.
    * </p>
    * 
    * @param rigidBodyTransformToTransform the rigid-body transform to transform. Modified.
    */
   default void transformToLocal(RigidBodyTransform rigidBodyTransformToTransform)
   {
      transformToLocal(rigidBodyTransformToTransform, rigidBodyTransformToTransform);
   }

   /**
    * Performs the transform to local frame on the given {@code original} and stores the result in
    * {@code transformed}.
    * <p>
    * This is equivalent to calling {@link #transform(RigidBodyTransform, RigidBodyTransform)} with
    * the inverse of this transform.
    * </p>
    * <p>
    * The given transform is only rotated and translated, no scaling is applied.
    * </p>
    * 
    * @param original the rigid-body transform to transform. Not modified.
    * @param transformed the rigid-body transform in which the result is stored. Modified.
    */
   void transformToLocal(RigidBodyTransform original, RigidBodyTransform transformed);

   /**
    * Performs the transform to local frame on the given
    * {@code quaternionBasedTransformToTransform}.
    * <p>
    * This is equivalent to calling {@link #transform(QuaternionBasedTransform)} with the inverse of
    * this transform.
    * </p>
    * <p>
    * The given transform is only rotated and translated, no scaling is applied.
    * </p>
    * 
    * @param quaternionBasedTransformToTransform the quaternion based transform to transform.
    *           Modified.
    */
   default void transformToLocal(QuaternionBasedTransform quaternionBasedTransformToTransform)
   {
      transformToLocal(quaternionBasedTransformToTransform, quaternionBasedTransformToTransform);
   }

   /**
    * Performs the transform to local frame on the given {@code original} stores the result in
    * {@code transformed}.
    * <p>
    * This is equivalent to calling
    * {@link #transform(QuaternionBasedTransform, QuaternionBasedTransform)} with the inverse of
    * this transform.
    * </p>
    * <p>
    * The given transform is only rotated and translated, no scaling is applied.
    * </p>
    * 
    * @param original the quaternion based transform to transform. Not modified.
    * @param transformed the quaternion based transform in which the result is stored. Modified.
    */
   void transformToLocal(QuaternionBasedTransform original, QuaternionBasedTransform transformed);
}
