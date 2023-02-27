package us.ihmc.robotics.math.trajectories.interfaces;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface Polynomial3DBasics extends Polynomial3DReadOnly, Transformable
{
   @Override
   default PolynomialBasics getAxis(Axis3D axis)
   {
      return getAxis(axis.ordinal());
   }

   @Override
   PolynomialBasics getAxis(int ordinal);

   @Override
   Tuple3DBasics[] getCoefficients();

   @Override
   default Tuple3DBasics getCoefficients(int i)
   {
      for (int axis = 0; axis < 3; axis++)
         getAxis(axis).setIsConstraintMatrixUpToDate(false);
      return getCoefficients()[i];
   }

   default void shiftTrajectory(Tuple3DReadOnly offset)
   {
      shiftTrajectory(offset.getX(), offset.getY(), offset.getZ());
   }

   default void shiftTrajectory(double offsetX, double offsetY, double offsetZ)
   {
      getCoefficients(0).add(offsetX, offsetY, offsetZ);
      for (int i = 0; i < 3; i++)
         getAxis(i).setIsConstraintMatrixUpToDate(false);
   }

   default void reset()
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).reset();
   }

   default void initialize()
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).initialize();
   }

   default void commitCoefficientsToMemory()
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).initialize();
   }

   default void set(Polynomial3DReadOnly other)
   {
      set(other.getAxis(Axis3D.X), other.getAxis(Axis3D.Y), other.getAxis(Axis3D.Z));
   }

   default void set(PolynomialReadOnly xPolynomial, PolynomialReadOnly yPolynomial, PolynomialReadOnly zPolynomial)
   {
      getAxis(Axis3D.X).set(xPolynomial);
      getAxis(Axis3D.Y).set(yPolynomial);
      getAxis(Axis3D.Z).set(zPolynomial);
   }


   @Override
   default void applyTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfCoefficients(); i++)
         getCoefficients(i).applyTransform(transform);
   }

   @Override
   default void applyInverseTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfCoefficients(); i++)
         getCoefficients(i).applyInverseTransform(transform);
   }

   default void setConstant(Point3DReadOnly z)
   {
      setConstant(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, z);
   }

   default void setZero()
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setZero();
   }

   default void setConstant(double t0, double tFinal, Tuple3DReadOnly z)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setConstant(t0, tFinal, z.getElement(index));
   }

   default void setCubic(double t0, double tFinal, Point3DReadOnly z0, Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setCubic(t0, tFinal, z0.getElement(index), zFinal.getElement(index));
   }

   default void setCubic(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setCubic(t0, tFinal, z0.getElement(index), zd0.getElement(index), zFinal.getElement(index), zdFinal.getElement(index));
   }

   default void setCubicDirectly(double duration, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setCubicDirectly(duration, z0.getElement(index), zd0.getElement(index), zFinal.getElement(index), zdFinal.getElement(index));
   }

   default void setCubicInitialPositionThreeFinalConditions(double t0, double tFinal, Point3DReadOnly z0, Point3DReadOnly zFinal, Vector3DReadOnly zdFinal,
                                                           Vector3DReadOnly zddFinal)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setCubicInitialPositionThreeFinalConditions(t0, tFinal, z0.getElement(index), zFinal.getElement(index), zdFinal.getElement(index),
                                                                          zddFinal.getElement(index));
   }

   default void setCubicThreeInitialConditionsFinalPosition(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0,
                                                           Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setCubicThreeInitialConditionsFinalPosition(t0, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index),
                                                                          zFinal.getElement(index));
   }

   default void setCubicUsingFinalAccelerationButNotFinalPosition(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdFinal,
                                                                 Vector3DReadOnly zddFinal)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setCubicUsingFinalAccelerationButNotFinalPosition(t0, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                                zdFinal.getElement(index), zddFinal.getElement(index));
   }

   default void setCubicUsingIntermediatePoints(double t0, double tIntermediate1, double tIntermediate2, double tFinal, Point3DReadOnly z0,
                                               Point3DReadOnly zIntermediate1, Point3DReadOnly zIntermediate2, Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setCubicUsingIntermediatePoints(t0, tIntermediate1, tIntermediate2, tFinal, z0.getElement(index),
                                                              zIntermediate1.getElement(index), zIntermediate2.getElement(index), zFinal.getElement(index));
   }

   default void setCubicUsingIntermediatePoint(double t0, double tIntermediate1, double tFinal, Point3DReadOnly z0, Point3DReadOnly zIntermediate1,
                                              Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setCubicUsingIntermediatePoint(t0, tIntermediate1, tFinal, z0.getElement(index), zIntermediate1.getElement(index),
                                                             zFinal.getElement(index));
   }

   default void setCubicWithIntermediatePositionAndFinalVelocityConstraint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0,
                                                                          Point3DReadOnly zIntermediate, Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setCubicWithIntermediatePositionAndFinalVelocityConstraint(t0, tIntermediate, tFinal, z0.getElement(index),
                                                                                         zIntermediate.getElement(index), zFinal.getElement(index),
                                                                                         zdFinal.getElement(index));
   }

   default void setCubicWithIntermediatePositionAndInitialVelocityConstraint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0,
                                                                            Vector3DReadOnly zd0, Point3DReadOnly zIntermediate, Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setCubicWithIntermediatePositionAndInitialVelocityConstraint(t0, tIntermediate, tFinal, z0.getElement(index),
                                                                                           zd0.getElement(index), zIntermediate.getElement(index),
                                                                                           zFinal.getElement(index));
   }

   default void setCubicBezier(double t0, double tFinal, Point3DReadOnly z0, Point3DReadOnly zR1, Point3DReadOnly zR2, Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setCubicBezier(t0, tFinal, z0.getElement(index), zR1.getElement(index), zR2.getElement(index), zFinal.getElement(index));
   }

   default void setInitialPositionVelocityZeroFinalHighOrderDerivatives(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                                                                       Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setInitialPositionVelocityZeroFinalHighOrderDerivatives(t0, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                                      zFinal.getElement(index), zdFinal.getElement(index));
   }

   default void setLinear(double t0, Point3DReadOnly z0, Vector3DReadOnly zDot)
   {
      for (Axis3D axis : Axis3D.values)
      {
         int index = axis.ordinal();
         getAxis(index).setLinear(t0, z0.getElement(index), zDot.getElement(index));
      }
   }

   default void setLinear(double t0, double tFinal, Point3DReadOnly z0, Point3DReadOnly zf)
   {
      for (Axis3D axis : Axis3D.values)
      {
         int index = axis.ordinal();
         getAxis(index).setLinear(t0, tFinal, z0.getElement(index), zf.getElement(index));
      }
   }

   default void setNonic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                        Point3DReadOnly zIntermediate0, Vector3DReadOnly zdIntermediate0, Point3DReadOnly zIntermediate1, Vector3DReadOnly zdIntermediate1,
                        Point3DReadOnly zf, Vector3DReadOnly zdf)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setNonic(t0, tIntermediate0, tIntermediate1, tFinal, z0.getElement(index), zd0.getElement(index),
                                       zIntermediate0.getElement(index), zdIntermediate0.getElement(index), zIntermediate1.getElement(index),
                                       zdIntermediate1.getElement(index), zf.getElement(index), zdf.getElement(index));

   }

   default void setQuadratic(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setQuadratic(t0, tFinal, z0.getElement(index), zd0.getElement(index), zFinal.getElement(index));
   }

   default void setQuadraticUsingInitialAcceleration(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setQuadraticUsingInitialAcceleration(t0, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index));
   }

   default void setQuadraticUsingIntermediatePoint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Point3DReadOnly zIntermediate,
                                                  Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setQuadraticUsingIntermediatePoint(t0, tIntermediate, tFinal, z0.getElement(index), zIntermediate.getElement(index),
                                                                 zFinal.getElement(index));
   }

   default void setQuadraticWithFinalVelocityConstraint(double t0, double tFinal, Point3DReadOnly z0, Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setQuadraticWithFinalVelocityConstraint(t0, tFinal, z0.getElement(index), zFinal.getElement(index), zdFinal.getElement(index));
   }

   default void setQuartic(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0, Point3DReadOnly zFinal,
                          Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setQuartic(t0, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index), zFinal.getElement(index),
                                         zdFinal.getElement(index));

   }

   default void setQuarticUsingFinalAcceleration(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zFinal,
                                                Vector3DReadOnly zdFinal, Vector3DReadOnly zddFinal)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setQuarticUsingFinalAcceleration(t0, tFinal, z0.getElement(index), zd0.getElement(index), zFinal.getElement(index),
                                                               zdFinal.getElement(index), zddFinal.getElement(index));
   }

   default void setQuarticUsingIntermediateVelocity(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                                                   Vector3DReadOnly zdIntermediate, Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setQuarticUsingIntermediateVelocity(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                  zdIntermediate.getElement(index), zFinal.getElement(index), zdFinal.getElement(index));

   }

   default void setQuarticUsingMidPoint(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zMid, Point3DReadOnly zFinal,
                                       Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setQuarticUsingMidPoint(t0, tFinal, z0.getElement(index), zd0.getElement(index), zMid.getElement(index), zFinal.getElement(index),
                                                      zdFinal.getElement(index));

   }

   default void setQuarticUsingOneIntermediateVelocity(double t0, double tIntermediate0, double tIntermediate1, double tFinal, Point3DReadOnly z0,
                                                      Point3DReadOnly zIntermediate0, Point3DReadOnly zIntermediate1, Point3DReadOnly zFinal,
                                                      Vector3DReadOnly zdIntermediate1)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setQuarticUsingOneIntermediateVelocity(t0, tIntermediate0, tIntermediate1, tFinal, z0.getElement(index),
                                                                     zIntermediate0.getElement(index), zIntermediate1.getElement(index),
                                                                     zFinal.getElement(index), zdIntermediate1.getElement(index));

   }

   default void setQuarticUsingWayPoint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zIntermediate,
                                       Point3DReadOnly zf, Vector3DReadOnly zdf)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setQuarticUsingWayPoint(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index), zIntermediate.getElement(index),
                                                      zf.getElement(index), zdf.getElement(index));
   }

   default void setQuintic(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0, Point3DReadOnly zf, Vector3DReadOnly zdf,
                          Vector3DReadOnly zddf)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setQuintic(t0, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index), zf.getElement(index),
                                         zdf.getElement(index), zddf.getElement(index));
   }

   default void setQuinticTwoWaypoints(double t0, double tIntermediate0, double tIntermediate1, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                                      Point3DReadOnly zIntermediate0, Point3DReadOnly zIntermediate1, Point3DReadOnly zf, Vector3DReadOnly zdf)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setQuinticTwoWaypoints(t0, tIntermediate0, tIntermediate1, tFinal, z0.getElement(index), zd0.getElement(index),
                                                     zIntermediate0.getElement(index), zIntermediate1.getElement(index), zf.getElement(index),
                                                     zdf.getElement(index));
   }

   default void setQuinticUsingIntermediateVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                                                                  Vector3DReadOnly zdIntermediate, Vector3DReadOnly zddIntermediate, Point3DReadOnly zFinal,
                                                                  Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setQuinticUsingIntermediateVelocityAndAcceleration(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                                 zdIntermediate.getElement(index), zddIntermediate.getElement(index),
                                                                                 zFinal.getElement(index), zdFinal.getElement(index));
   }

   default void setQuinticUsingWayPoint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0,
                                       Point3DReadOnly zIntermediate, Point3DReadOnly zf, Vector3DReadOnly zdf)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setQuinticUsingWayPoint(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index),
                                                      zIntermediate.getElement(index), zf.getElement(index), zdf.getElement(index));
   }

   default void setQuinticUsingWayPoint2(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0,
                                        Point3DReadOnly zIntermediate, Vector3DReadOnly zdIntermediate, Point3DReadOnly zf)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setQuinticUsingWayPoint2(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index),
                                                       zIntermediate.getElement(index), zdIntermediate.getElement(index), zf.getElement(index));
   }

   default void setSeptic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                         Point3DReadOnly zIntermediate0, Vector3DReadOnly zdIntermediate0, Point3DReadOnly zIntermediate1, Vector3DReadOnly zdIntermediate1,
                         Point3DReadOnly zf, Vector3DReadOnly zdf)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setSeptic(t0, tIntermediate0, tIntermediate1, tFinal, z0.getElement(index), zd0.getElement(index),
                                        zIntermediate0.getElement(index), zdIntermediate0.getElement(index), zIntermediate1.getElement(index),
                                        zdIntermediate1.getElement(index), zf.getElement(index), zdf.getElement(index));

   }

   default void setSepticInitialAndFinalAcceleration(double t0, double tIntermediate0, double tIntermediate1, double tFinal, Point3DReadOnly z0,
                                                    Vector3DReadOnly zd0, Vector3DReadOnly zdd0, Point3DReadOnly zIntermediate0, Point3DReadOnly zIntermediate1,
                                                    Point3DReadOnly zf, Vector3DReadOnly zdf, Vector3DReadOnly zddf)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setSepticInitialAndFinalAcceleration(t0, tIntermediate0, tIntermediate1, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                   zdd0.getElement(index), zIntermediate0.getElement(index), zIntermediate1.getElement(index),
                                                                   zf.getElement(index), zdf.getElement(index), zddf.getElement(index));

   }

   default void setQuinticWithZeroTerminalAcceleration(double t0, double tFinal, Tuple3DReadOnly z0, Tuple3DReadOnly zd0, Tuple3DReadOnly zFinal,
                                                       Tuple3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setQuinticWithZeroTerminalAcceleration(t0, tFinal, z0.getElement(index), zd0.getElement(index), zFinal.getElement(index),
                                                                     zdFinal.getElement(index));
   }

   default void setSexticUsingWaypoint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0,
                                      Point3DReadOnly zIntermediate, Point3DReadOnly zf, Vector3DReadOnly zdf, Vector3DReadOnly zddf)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).setSexticUsingWaypoint(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index),
                                                     zIntermediate.getElement(index), zf.getElement(index), zdf.getElement(index), zddf.getElement(index));

   }

   default void setSexticUsingWaypointVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                                                             Vector3DReadOnly zdd0, Vector3DReadOnly zdIntermediate, Vector3DReadOnly zddIntermediate,
                                                             Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (Axis3D axis : Axis3D.values)
      {
         int index = axis.ordinal();
         getAxis(index).setSexticUsingWaypointVelocityAndAcceleration(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                            zdd0.getElement(index), zdIntermediate.getElement(index),
                                                                            zddIntermediate.getElement(index), zFinal.getElement(index),
                                                                            zdFinal.getElement(index));
      }
   }


}
