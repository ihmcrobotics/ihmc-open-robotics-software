package us.ihmc.robotics.math.trajectories;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.commons.MathTools;

public class Trajectory3D
{
   protected final Trajectory xTrajectory;
   protected final Trajectory yTrajectory;
   protected final Trajectory zTrajectory;

   public Trajectory3D(int maximumNumberOfCoefficients)
   {
      xTrajectory = new Trajectory(maximumNumberOfCoefficients);
      yTrajectory = new Trajectory(maximumNumberOfCoefficients);
      zTrajectory = new Trajectory(maximumNumberOfCoefficients);
   }

   public Trajectory3D(Trajectory xTrajectory, Trajectory yTrajectory, Trajectory zTrajectory)
   {
      this.xTrajectory = xTrajectory;
      this.yTrajectory = yTrajectory;
      this.zTrajectory = zTrajectory;
   }

   public Trajectory3D(Trajectory[] trajectory)
   {
      if (trajectory.length != 3)
         throw new RuntimeException("Expected 3 YoTrajectories for representing the three axes X, Y, and Z, but had: " + trajectory.length
               + " YoTrajectories.");

      this.xTrajectory = trajectory[0];
      this.yTrajectory = trajectory[1];
      this.zTrajectory = trajectory[2];
   }

   public Trajectory3D(List<Trajectory> trajectory)
   {
      if (trajectory.size() != 3)
         throw new RuntimeException("Expected 3 YoTrajectories for representing the three axes X, Y, and Z, but had: " + trajectory.size()
               + " YoTrajectories.");

      this.xTrajectory = trajectory.get(0);
      this.yTrajectory = trajectory.get(1);
      this.zTrajectory = trajectory.get(2);
   }

   private final Point3DReadOnly position = new Point3DReadOnly()
   {
      @Override
      public double getX()
      {
         return xTrajectory.getPosition();
      }

      @Override
      public double getY()
      {
         return yTrajectory.getPosition();
      }

      @Override
      public double getZ()
      {
         return zTrajectory.getPosition();
      }

      @Override
      public String toString()
      {
         return "X: " + getX() + " Y: " + getY() + " Z: " + getZ();
      }
   };

   private final Vector3DReadOnly velocity = new Vector3DReadOnly()
   {
      @Override
      public double getX()
      {
         return xTrajectory.getVelocity();
      }

      @Override
      public double getY()
      {
         return yTrajectory.getVelocity();
      }

      @Override
      public double getZ()
      {
         return zTrajectory.getVelocity();
      }
   };

   private final Vector3DReadOnly acceleration = new Vector3DReadOnly()
   {
      @Override
      public double getX()
      {
         return xTrajectory.getAcceleration();
      }

      @Override
      public double getY()
      {
         return yTrajectory.getAcceleration();
      }

      @Override
      public double getZ()
      {
         return zTrajectory.getAcceleration();
      }
   };

   private double xIntegralResult = Double.NaN;
   private double yIntegralResult = Double.NaN;
   private double zIntegralResult = Double.NaN;

   private final Tuple3DReadOnly integralResult = new Tuple3DReadOnly()
   {
      @Override
      public double getX()
      {
         return xIntegralResult;
      }

      @Override
      public double getY()
      {
         return yIntegralResult;
      }

      @Override
      public double getZ()
      {
         return zIntegralResult;
      }
   };

   public static Trajectory3D[] createTrajectory3DArray(Trajectory[] xTrajectory, Trajectory[] yTrajectory, Trajectory[] zTrajectory)
   {
      if (xTrajectory.length != yTrajectory.length || xTrajectory.length != zTrajectory.length)
         throw new RuntimeException("Cannot handle different number of trajectories for the different axes.");

      Trajectory3D[] yoTrajectory3Ds = new Trajectory3D[xTrajectory.length];

      for (int i = 0; i < xTrajectory.length; i++)
      {
         yoTrajectory3Ds[i] = new Trajectory3D(xTrajectory[i], yTrajectory[i], zTrajectory[i]);
      }
      return yoTrajectory3Ds;
   }

   public static Trajectory3D[] createTrajectory3DArray(List<Trajectory> xTrajectory, List<Trajectory> yTrajectory, List<Trajectory> zTrajectory)
   {
      if (xTrajectory.size() != yTrajectory.size() || xTrajectory.size() != zTrajectory.size())
         throw new RuntimeException("Cannot handle different number of trajectories for the different axes.");

      Trajectory3D[] yoTrajectory3Ds = new Trajectory3D[xTrajectory.size()];

      for (int i = 0; i < xTrajectory.size(); i++)
      {
         yoTrajectory3Ds[i] = new Trajectory3D(xTrajectory.get(i), yTrajectory.get(i), zTrajectory.get(i));
      }
      return yoTrajectory3Ds;
   }

   public static List<Trajectory3D> createTrajectory3DList(Trajectory[] xTrajectory, Trajectory[] yTrajectory, Trajectory[] zTrajectory)
   {
      if (xTrajectory.length != yTrajectory.length || xTrajectory.length != zTrajectory.length)
         throw new RuntimeException("Cannot handle different number of trajectories for the different axes.");

      List<Trajectory3D> yoTrajectory3Ds = new ArrayList<>(xTrajectory.length);

      for (int i = 0; i < xTrajectory.length; i++)
      {
         yoTrajectory3Ds.add(new Trajectory3D(xTrajectory[i], yTrajectory[i], zTrajectory[i]));
      }
      return yoTrajectory3Ds;
   }

   public static List<Trajectory3D> createTrajectory3DList(List<Trajectory> xTrajectory, List<Trajectory> yTrajectory, List<Trajectory> zTrajectory)
   {
      if (xTrajectory.size() != yTrajectory.size() || xTrajectory.size() != zTrajectory.size())
         throw new RuntimeException("Cannot handle different number of trajectories for the different axes.");

      List<Trajectory3D> yoTrajectory3Ds = new ArrayList<>(xTrajectory.size());

      for (int i = 0; i < xTrajectory.size(); i++)
      {
         yoTrajectory3Ds.add(new Trajectory3D(xTrajectory.get(i), yTrajectory.get(i), zTrajectory.get(i)));
      }
      return yoTrajectory3Ds;
   }

   public void compute(double t)
   {
      xTrajectory.compute(t);
      yTrajectory.compute(t);
      zTrajectory.compute(t);
   }

   public Point3DReadOnly getPosition()
   {
      return position;
   }

   public Vector3DReadOnly getVelocity()
   {
      return velocity;
   }

   public Vector3DReadOnly getAcceleration()
   {
      return acceleration;
   }

   public Tuple3DReadOnly getIntegral(double from, double to)
   {
      xIntegralResult = xTrajectory.getIntegral(from, to);
      yIntegralResult = yTrajectory.getIntegral(from, to);
      zIntegralResult = zTrajectory.getIntegral(from, to);
      return integralResult;
   }

   public Trajectory getTrajectory(Axis axis)
   {
      return getTrajectory(axis.ordinal());
   }

   public Trajectory getTrajectory(int index)
   {
      switch (index)
      {
      case 0:
         return getTrajectoryX();
      case 1:
         return getTrajectoryY();
      case 2:
         return getTrajectoryZ();
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   public Trajectory getTrajectoryX()
   {
      return xTrajectory;
   }

   public Trajectory getTrajectoryY()
   {
      return yTrajectory;
   }

   public Trajectory getTrajectoryZ()
   {
      return zTrajectory;
   }

   public void setTime(double tInital, double tFinal)
   {
      setInitialTime(tInital);
      setFinalTime(tFinal);
   }

   public void setInitialTime(double tInitial)
   {
      for (int i = 0; i < 3; i++)
         getTrajectory(i).setInitialTime(tInitial);
   }

   public void setFinalTime(double tFinal)
   {
      for (int i = 0; i < 3; i++)
         getTrajectory(i).setFinalTime(tFinal);
   }

   public double getInitialTime()
   {
      if (MathTools.epsilonCompare(xTrajectory.getInitialTime(), yTrajectory.getInitialTime(), Epsilons.ONE_THOUSANDTH)
            && MathTools.epsilonCompare(xTrajectory.getInitialTime(), zTrajectory.getInitialTime(), Epsilons.ONE_THOUSANDTH))
         return xTrajectory.getInitialTime();
      else
      {
         //PrintTools.warn("Trajectory initial times do not match. Using X trajectory times for computation");
         return xTrajectory.getInitialTime();
      }
   }

   public double getFinalTime()
   {
      if (MathTools.epsilonCompare(xTrajectory.getFinalTime(), yTrajectory.getFinalTime(), Epsilons.ONE_THOUSANDTH)
            && MathTools.epsilonCompare(xTrajectory.getFinalTime(), zTrajectory.getFinalTime(), Epsilons.ONE_THOUSANDTH))
         return xTrajectory.getFinalTime();
      else
      {
         //PrintTools.warn("Trajectory final times do not match. Using X trajectory times for computation");
         return xTrajectory.getFinalTime();
      }
   }

   public double getInitialTime(Axis dir)
   {
      return getTrajectory(dir).getInitialTime();
   }

   public double getInitialTime(int index)
   {
      return getTrajectory(index).getInitialTime();
   }

   public double getFinalTime(Axis dir)
   {
      return getTrajectory(dir).getFinalTime();
   }

   public double getFinalTime(int index)
   {
      return getTrajectory(index).getFinalTime();
   }

   public boolean timeIntervalContains(double timeToCheck, double EPSILON)
   {
      return (xTrajectory.timeIntervalContains(timeToCheck, EPSILON) && yTrajectory.timeIntervalContains(timeToCheck, EPSILON)
            && zTrajectory.timeIntervalContains(timeToCheck, EPSILON));
   }

   public boolean timeIntervalContains(double timeToCheck)
   {
      return (xTrajectory.timeIntervalContains(timeToCheck) && yTrajectory.timeIntervalContains(timeToCheck) && zTrajectory.timeIntervalContains(timeToCheck));
   }

   /**
    * Returns the number of coefficients for the trajectory if it is the same for all axes. If not then returns -1
    * @return
    */
   public int getNumberOfCoefficients()
   {
      if (xTrajectory.getNumberOfCoefficients() == yTrajectory.getNumberOfCoefficients()
            && xTrajectory.getNumberOfCoefficients() == zTrajectory.getNumberOfCoefficients())
         return xTrajectory.getNumberOfCoefficients();
      else
         return -1;
   }

   public int getNumberOfCoefficients(Axis dir)
   {
      return getTrajectory(dir).getNumberOfCoefficients();
   }

   public int getNumberOfCoefficients(int index)
   {
      return getTrajectory(index).getNumberOfCoefficients();
   }

   public void reset()
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).reset();
   }

   public void set(Trajectory3D other)
   {
      xTrajectory.set(other.getTrajectoryX());
      yTrajectory.set(other.getTrajectoryY());
      zTrajectory.set(other.getTrajectoryZ());
   }

   public void setZero()
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setZero();
   }

   public void setConstant(double t0, double tFinal, Point3DReadOnly z)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setConstant(t0, tFinal, z.getElement(index));
   }

   public void setCubic(double t0, double tFinal, Point3DReadOnly z0, Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setCubic(t0, tFinal, z0.getElement(index), zFinal.getElement(index));
   }

   public void setCubic(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setCubic(t0, tFinal, z0.getElement(index), zd0.getElement(index), zFinal.getElement(index), zdFinal.getElement(index));
   }

   public void setCubicInitialPositionThreeFinalConditions(double t0, double tFinal, Point3DReadOnly z0, Point3DReadOnly zFinal, Vector3DReadOnly zdFinal,
                                                           Vector3DReadOnly zddFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setCubicInitialPositionThreeFinalConditions(t0, tFinal, z0.getElement(index), zFinal.getElement(index),
                                                                            zdFinal.getElement(index), zddFinal.getElement(index));
   }

   public void setCubicThreeInitialConditionsFinalPosition(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0,
                                                           Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setCubicThreeInitialConditionsFinalPosition(t0, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index),
                                                                            zFinal.getElement(index));
   }

   public void setCubicUsingFinalAccelerationButNotFinalPosition(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdFinal,
                                                                 Vector3DReadOnly zddFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setCubicUsingFinalAccelerationButNotFinalPosition(t0, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                                  zdFinal.getElement(index), zddFinal.getElement(index));
   }

   public void setCubicUsingIntermediatePoints(double t0, double tIntermediate1, double tIntermediate2, double tFinal, Point3DReadOnly z0,
                                               Point3DReadOnly zIntermediate1, Point3DReadOnly zIntermediate2, Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setCubicUsingIntermediatePoints(t0, tIntermediate1, tIntermediate2, tFinal, z0.getElement(index),
                                                                zIntermediate1.getElement(index), zIntermediate2.getElement(index), zFinal.getElement(index));
   }

   public void setCubicUsingIntermediatePoint(double t0, double tIntermediate1, double tFinal, Point3DReadOnly z0, Point3DReadOnly zIntermediate1,
                                              Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setCubicUsingIntermediatePoint(t0, tIntermediate1, tFinal, z0.getElement(index), zIntermediate1.getElement(index),
                                                               zFinal.getElement(index));
   }

   public void setCubicWithIntermediatePositionAndFinalVelocityConstraint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0,
                                                                          Point3DReadOnly zIntermediate, Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setCubicWithIntermediatePositionAndFinalVelocityConstraint(t0, tIntermediate, tFinal, z0.getElement(index),
                                                                                           zIntermediate.getElement(index), zFinal.getElement(index),
                                                                                           zdFinal.getElement(index));
   }

   public void setCubicWithIntermediatePositionAndInitialVelocityConstraint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0,
                                                                            Vector3DReadOnly zd0, Point3DReadOnly zIntermediate, Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setCubicWithIntermediatePositionAndInitialVelocityConstraint(t0, tIntermediate, tFinal, z0.getElement(index),
                                                                                             zd0.getElement(index), zIntermediate.getElement(index),
                                                                                             zFinal.getElement(index));
   }

   public void setCubicBezier(double t0, double tFinal, Point3DReadOnly z0, Point3DReadOnly zR1, Point3DReadOnly zR2, Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setCubicBezier(t0, tFinal, z0.getElement(index), zR1.getElement(index), zR2.getElement(index), zFinal.getElement(index));
   }

   public void setInitialPositionVelocityZeroFinalHighOrderDerivatives(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                                                                       Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setInitialPositionVelocityZeroFinalHighOrderDerivatives(t0, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                                        zFinal.getElement(index), zdFinal.getElement(index));
   }

   public void setLinear(double t0, double tFinal, Point3DReadOnly z0, Point3DReadOnly zf)
   {
      for (Axis axis : Axis.values)
      {
         int index = axis.ordinal();
         getTrajectory(index).setLinear(t0, tFinal, z0.getElement(index), zf.getElement(index));
      }
   }

   public void setNonic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                        Point3DReadOnly zIntermediate0, Vector3DReadOnly zdIntermediate0, Point3DReadOnly zIntermediate1, Vector3DReadOnly zdIntermediate1,
                        Point3DReadOnly zf, Vector3DReadOnly zdf)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setNonic(t0, tIntermediate0, tIntermediate1, tFinal, z0.getElement(index), zd0.getElement(index),
                                         zIntermediate0.getElement(index), zdIntermediate0.getElement(index), zIntermediate1.getElement(index),
                                         zdIntermediate1.getElement(index), zf.getElement(index), zdf.getElement(index));

   }

   public void setQuadratic(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setQuadratic(t0, tFinal, z0.getElement(index), zd0.getElement(index), zFinal.getElement(index));
   }

   public void setQuadraticUsingInitialAcceleration(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setQuadraticUsingInitialAcceleration(t0, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index));
   }

   public void setQuadraticUsingIntermediatePoint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Point3DReadOnly zIntermediate,
                                                  Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setQuadraticUsingIntermediatePoint(t0, tIntermediate, tFinal, z0.getElement(index), zIntermediate.getElement(index),
                                                                   zFinal.getElement(index));
   }

   public void setQuadraticWithFinalVelocityConstraint(double t0, double tFinal, Point3DReadOnly z0, Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setQuadraticWithFinalVelocityConstraint(t0, tFinal, z0.getElement(index), zFinal.getElement(index), zdFinal.getElement(index));
   }

   public void setQuartic(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0, Point3DReadOnly zFinal,
                          Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setQuartic(t0, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index), zFinal.getElement(index),
                                           zdFinal.getElement(index));

   }

   public void setQuarticUsingFinalAcceleration(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zFinal,
                                                Vector3DReadOnly zdFinal, Vector3DReadOnly zddFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setQuarticUsingFinalAcceleration(t0, tFinal, z0.getElement(index), zd0.getElement(index), zFinal.getElement(index),
                                                                 zdFinal.getElement(index), zddFinal.getElement(index));
   }

   public void setQuarticUsingIntermediateVelocity(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                                                   Vector3DReadOnly zdIntermediate, Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setQuarticUsingIntermediateVelocity(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                    zdIntermediate.getElement(index), zFinal.getElement(index), zdFinal.getElement(index));

   }

   public void setQuarticUsingMidPoint(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zMid, Point3DReadOnly zFinal,
                                       Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setQuarticUsingMidPoint(t0, tFinal, z0.getElement(index), zd0.getElement(index), zMid.getElement(index),
                                                        zFinal.getElement(index), zdFinal.getElement(index));

   }

   public void setQuarticUsingOneIntermediateVelocity(double t0, double tIntermediate0, double tIntermediate1, double tFinal, Point3DReadOnly z0,
                                                      Point3DReadOnly zIntermediate0, Point3DReadOnly zIntermediate1, Point3DReadOnly zFinal,
                                                      Vector3DReadOnly zdIntermediate1)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setQuarticUsingOneIntermediateVelocity(t0, tIntermediate0, tIntermediate1, tFinal, z0.getElement(index),
                                                                       zIntermediate0.getElement(index), zIntermediate1.getElement(index),
                                                                       zFinal.getElement(index), zdIntermediate1.getElement(index));

   }

   public void setQuarticUsingWayPoint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zIntermediate,
                                       Point3DReadOnly zf, Vector3DReadOnly zdf)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setQuarticUsingWayPoint(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index), zIntermediate.getElement(index),
                                                        zf.getElement(index), zdf.getElement(index));
   }

   public void setQuintic(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0, Point3DReadOnly zf, Vector3DReadOnly zdf,
                          Vector3DReadOnly zddf)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setQuintic(t0, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index), zf.getElement(index),
                                           zdf.getElement(index), zddf.getElement(index));
   }

   public void setQuinticTwoWaypoints(double t0, double tIntermediate0, double tIntermediate1, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                                      Point3DReadOnly zIntermediate0, Point3DReadOnly zIntermediate1, Point3DReadOnly zf, Vector3DReadOnly zdf)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setQuinticTwoWaypoints(t0, tIntermediate0, tIntermediate1, tFinal, z0.getElement(index), zd0.getElement(index),
                                                       zIntermediate0.getElement(index), zIntermediate1.getElement(index), zf.getElement(index),
                                                       zdf.getElement(index));
   }

   public void setQuinticUsingIntermediateVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                                                                  Vector3DReadOnly zdIntermediate, Vector3DReadOnly zddIntermediate, Point3DReadOnly zFinal,
                                                                  Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setQuinticUsingIntermediateVelocityAndAcceleration(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                                   zdIntermediate.getElement(index), zddIntermediate.getElement(index),
                                                                                   zFinal.getElement(index), zdFinal.getElement(index));
   }

   public void setQuinticUsingWayPoint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0,
                                       Point3DReadOnly zIntermediate, Point3DReadOnly zf, Vector3DReadOnly zdf)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setQuinticUsingWayPoint(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index),
                                                        zIntermediate.getElement(index), zf.getElement(index), zdf.getElement(index));
   }

   public void setQuinticUsingWayPoint2(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0,
                                        Point3DReadOnly zIntermediate, Vector3DReadOnly zdIntermediate, Point3DReadOnly zf)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setQuinticUsingWayPoint2(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index),
                                                         zIntermediate.getElement(index), zdIntermediate.getElement(index), zf.getElement(index));
   }

   public void setSeptic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                         Point3DReadOnly zIntermediate0, Vector3DReadOnly zdIntermediate0, Point3DReadOnly zIntermediate1, Vector3DReadOnly zdIntermediate1,
                         Point3DReadOnly zf, Vector3DReadOnly zdf)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setSeptic(t0, tIntermediate0, tIntermediate1, tFinal, z0.getElement(index), zd0.getElement(index),
                                          zIntermediate0.getElement(index), zdIntermediate0.getElement(index), zIntermediate1.getElement(index),
                                          zdIntermediate1.getElement(index), zf.getElement(index), zdf.getElement(index));

   }

   public void setSepticInitialAndFinalAcceleration(double t0, double tIntermediate0, double tIntermediate1, double tFinal, Point3DReadOnly z0,
                                                    Vector3DReadOnly zd0, Vector3DReadOnly zdd0, Point3DReadOnly zIntermediate0, Point3DReadOnly zIntermediate1,
                                                    Point3DReadOnly zf, Vector3DReadOnly zdf, Vector3DReadOnly zddf)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setSepticInitialAndFinalAcceleration(t0, tIntermediate0, tIntermediate1, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                     zdd0.getElement(index), zIntermediate0.getElement(index), zIntermediate1.getElement(index),
                                                                     zf.getElement(index), zdf.getElement(index), zddf.getElement(index));

   }

   public void setQuinticWithZeroTerminalAcceleration(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zFinal,
                                                     Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setQuinticWithZeroTerminalAcceleration(t0, tFinal, z0.getElement(index), zd0.getElement(index), zFinal.getElement(index),
                                                                      zdFinal.getElement(index));
   }

   public void setSexticUsingWaypoint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0,
                                      Point3DReadOnly zIntermediate, Point3DReadOnly zf, Vector3DReadOnly zdf, Vector3DReadOnly zddf)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setSexticUsingWaypoint(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index),
                                                       zIntermediate.getElement(index), zf.getElement(index), zdf.getElement(index), zddf.getElement(index));

   }

   public void setSexticUsingWaypointVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                                                             Vector3DReadOnly zdd0, Vector3DReadOnly zdIntermediate, Vector3DReadOnly zddIntermediate,
                                                             Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (Axis axis : Axis.values)
      {
         int index = axis.ordinal();
         getTrajectory(index).setSexticUsingWaypointVelocityAndAcceleration(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                              zdd0.getElement(index), zdIntermediate.getElement(index),
                                                                              zddIntermediate.getElement(index), zFinal.getElement(index),
                                                                              zdFinal.getElement(index));
      }
   }

   @Override
   public String toString()
   {
      return "X: " + xTrajectory.toString() + "\n" + "Y: " + yTrajectory.toString() + "\n" + "Z: " + zTrajectory.toString();
   }

   public String toString2()
   {
      return "X: " + xTrajectory.toString2() + "\n" + "Y: " + yTrajectory.toString2() + "\n" + "Z: " + zTrajectory.toString2();
   }

   public void getDerivative(int order, double x, Tuple3DBasics dTrajectory)
   {
      dTrajectory.set(xTrajectory.getDerivative(order, x), yTrajectory.getDerivative(order, x), zTrajectory.getDerivative(order, x));
   }

   public void getDerivative(Trajectory3D dervTraj, int order)
   {
      for (Axis axis : Axis.values)
      {
         int index = axis.ordinal();
         getTrajectory(index).getDerivative(dervTraj.getTrajectory(index), order);
      }
   }

   public void getStartPoint(Point3D positionToPack)
   {
      compute(getInitialTime());
      positionToPack.set(getPosition());
   }

   public void getEndPoint(Point3D positionToPack)
   {
      compute(getFinalTime());
      positionToPack.set(getPosition());
   }
   
   public boolean isValidTrajectory()
   {
      return (getTrajectoryX().isValidTrajectory() && getTrajectoryY().isValidTrajectory() && getTrajectoryZ().isValidTrajectory());
   }
   
}
