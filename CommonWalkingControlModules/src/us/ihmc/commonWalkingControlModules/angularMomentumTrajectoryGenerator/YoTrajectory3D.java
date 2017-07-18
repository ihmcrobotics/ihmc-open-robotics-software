package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoTrajectory3D
{
   protected final YoTrajectory xTrajectory;
   protected final YoTrajectory yTrajectory;
   protected final YoTrajectory zTrajectory;

   public YoTrajectory3D(String name, int maximumNumberOfCoefficients, YoVariableRegistry registry)
   {
      xTrajectory = new YoTrajectory(name + "X", maximumNumberOfCoefficients, registry);
      yTrajectory = new YoTrajectory(name + "Y", maximumNumberOfCoefficients, registry);
      zTrajectory = new YoTrajectory(name + "Z", maximumNumberOfCoefficients, registry);
   }

   public YoTrajectory3D(YoTrajectory xTrajectory, YoTrajectory yTrajectory, YoTrajectory zTrajectory)
   {
      this.xTrajectory = xTrajectory;
      this.yTrajectory = yTrajectory;
      this.zTrajectory = zTrajectory;
   }

   public YoTrajectory3D(YoTrajectory[] yoTrajectory)
   {
      if (yoTrajectory.length != 3)
         throw new RuntimeException("Expected 3 YoTrajectories for representing the three axes X, Y, and Z, but had: " + yoTrajectory.length
               + " YoTrajectories.");

      this.xTrajectory = yoTrajectory[0];
      this.yTrajectory = yoTrajectory[1];
      this.zTrajectory = yoTrajectory[2];
   }

   public YoTrajectory3D(List<YoTrajectory> yoTrajectory)
   {
      if (yoTrajectory.size() != 3)
         throw new RuntimeException("Expected 3 YoTrajectories for representing the three axes X, Y, and Z, but had: " + yoTrajectory.size()
               + " YoTrajectories.");

      this.xTrajectory = yoTrajectory.get(0);
      this.yTrajectory = yoTrajectory.get(1);
      this.zTrajectory = yoTrajectory.get(2);
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

   public static YoTrajectory3D[] createYoTrajectory3DArray(YoTrajectory[] xTrajectory, YoTrajectory[] yTrajectory, YoTrajectory[] zTrajectory)
   {
      if (xTrajectory.length != yTrajectory.length || xTrajectory.length != zTrajectory.length)
         throw new RuntimeException("Cannot handle different number of trajectories for the different axes.");

      YoTrajectory3D[] yoTrajectory3Ds = new YoTrajectory3D[xTrajectory.length];

      for (int i = 0; i < xTrajectory.length; i++)
      {
         yoTrajectory3Ds[i] = new YoTrajectory3D(xTrajectory[i], yTrajectory[i], zTrajectory[i]);
      }
      return yoTrajectory3Ds;
   }

   public static YoTrajectory3D[] createYoTrajectory3DArray(List<YoTrajectory> xTrajectory, List<YoTrajectory> yTrajectory, List<YoTrajectory> zTrajectory)
   {
      if (xTrajectory.size() != yTrajectory.size() || xTrajectory.size() != zTrajectory.size())
         throw new RuntimeException("Cannot handle different number of trajectories for the different axes.");

      YoTrajectory3D[] yoTrajectory3Ds = new YoTrajectory3D[xTrajectory.size()];

      for (int i = 0; i < xTrajectory.size(); i++)
      {
         yoTrajectory3Ds[i] = new YoTrajectory3D(xTrajectory.get(i), yTrajectory.get(i), zTrajectory.get(i));
      }
      return yoTrajectory3Ds;
   }

   public static List<YoTrajectory3D> createYoTrajectory3DList(YoTrajectory[] xTrajectory, YoTrajectory[] yTrajectory, YoTrajectory[] zTrajectory)
   {
      if (xTrajectory.length != yTrajectory.length || xTrajectory.length != zTrajectory.length)
         throw new RuntimeException("Cannot handle different number of trajectories for the different axes.");

      List<YoTrajectory3D> yoTrajectory3Ds = new ArrayList<>(xTrajectory.length);

      for (int i = 0; i < xTrajectory.length; i++)
      {
         yoTrajectory3Ds.add(new YoTrajectory3D(xTrajectory[i], yTrajectory[i], zTrajectory[i]));
      }
      return yoTrajectory3Ds;
   }

   public static List<YoTrajectory3D> createYoTrajectory3DList(List<YoTrajectory> xTrajectory, List<YoTrajectory> yTrajectory, List<YoTrajectory> zTrajectory)
   {
      if (xTrajectory.size() != yTrajectory.size() || xTrajectory.size() != zTrajectory.size())
         throw new RuntimeException("Cannot handle different number of trajectories for the different axes.");

      List<YoTrajectory3D> yoTrajectory3Ds = new ArrayList<>(xTrajectory.size());

      for (int i = 0; i < xTrajectory.size(); i++)
      {
         yoTrajectory3Ds.add(new YoTrajectory3D(xTrajectory.get(i), yTrajectory.get(i), zTrajectory.get(i)));
      }
      return yoTrajectory3Ds;
   }

   public void compute(double t)
   {
      xTrajectory.compute(t);
      yTrajectory.compute(t);
      zTrajectory.compute(t);
   }

   public void add(YoTrajectory3D traj1, YoTrajectory3D traj2)
   {
      TrajectoryMathTools.add(this, traj1, traj2);
   }

   public void add(YoTrajectory3D addTraj)
   {
      add(this, addTraj);
   }

   public void addByTrimming(YoTrajectory3D traj1, YoTrajectory3D traj2)
   {
      TrajectoryMathTools.addByTrimming(this, traj1, traj2);
   }

   public void addByTrimming(YoTrajectory3D addTraj)
   {
      addByTrimming(this, addTraj);
   }

   public void subtract(YoTrajectory3D traj1, YoTrajectory3D traj2)
   {
      TrajectoryMathTools.subtract(this, traj1, traj2);
   }

   public void subtract(YoTrajectory3D subTraj)
   {
      subtract(this, subTraj);
   }

   public void subtractByTrimming(YoTrajectory3D traj1, YoTrajectory3D traj2)
   {
      TrajectoryMathTools.subtractByTrimming(this, traj1, traj2);
   }

   public void subtractByTrimming(YoTrajectory3D subTraj)
   {
      subtractByTrimming(this, subTraj);
   }

   public void dotProduct(YoTrajectory3D traj1, YoTrajectory3D traj2)
   {
      TrajectoryMathTools.dotProduct(this, traj1, traj2);
   }

   public void dotProduct(YoTrajectory3D dotPTraj)
   {
      dotProduct(this, dotPTraj);
   }

   public void dotProductByTrimming(YoTrajectory3D traj1, YoTrajectory3D traj2)
   {
      TrajectoryMathTools.dotProductByTrimming(this, traj1, traj2);
   }

   public void dotProductByTrimming(YoTrajectory3D dotPTraj)
   {
      dotProductByTrimming(this, dotPTraj);
   }

   public void crossProduct(YoTrajectory3D traj1, YoTrajectory3D traj2)
   {
      TrajectoryMathTools.crossProduct(this, traj1, traj2);
   }

   public void crossProduct(YoTrajectory3D crossPTraj)
   {
      crossProduct(this, crossPTraj);
   }

   public void crossProductByTrimming(YoTrajectory3D traj1, YoTrajectory3D traj2)
   {
      TrajectoryMathTools.crossProductByTrimming(this, traj1, traj2);
   }

   public void crossProductByTrimming(YoFrameTrajectory3D crossPTraj)
   {
      crossProduct(this, crossPTraj);
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

   public YoTrajectory getYoTrajectory(Direction direction)
   {
      switch (direction)
      {
      case X:
         return getYoTrajectoryX();
      case Y:
         return getYoTrajectoryY();
      case Z:
         return getYoTrajectoryZ();
      default:
         throw new IndexOutOfBoundsException(direction.toString());
      }
   }

   public YoTrajectory getYoTrajectory(int index)
   {
      switch (index)
      {
      case 0:
         return getYoTrajectoryX();
      case 1:
         return getYoTrajectoryY();
      case 2:
         return getYoTrajectoryZ();
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   public YoTrajectory getYoTrajectoryX()
   {
      return xTrajectory;
   }

   public YoTrajectory getYoTrajectoryY()
   {
      return yTrajectory;
   }

   public YoTrajectory getYoTrajectoryZ()
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
         getYoTrajectory(i).setInitialTime(tInitial);
   }

   public void setFinalTime(double tFinal)
   {
      for (int i = 0; i < 3; i++)
         getYoTrajectory(i).setFinalTime(tFinal);
   }

   public double getInitialTime()
   {
      if (xTrajectory.getInitialTime() == yTrajectory.getInitialTime() && xTrajectory.getInitialTime() == zTrajectory.getInitialTime())
         return xTrajectory.getInitialTime();
      else
         return xTrajectory.getInitialTime();
   }

   public double getFinalTime()
   {
      if (xTrajectory.getFinalTime() == yTrajectory.getFinalTime() && xTrajectory.getFinalTime() == zTrajectory.getFinalTime())
         return xTrajectory.getFinalTime();
      else
         return xTrajectory.getFinalTime();
   }

   public double getInitialTime(Direction dir)
   {
      return getYoTrajectory(dir).getInitialTime();
   }

   public double getInitialTime(int index)
   {
      return getYoTrajectory(index).getInitialTime();
   }

   public double getFinalTime(Direction dir)
   {
      return getYoTrajectory(dir).getFinalTime();
   }

   public double getFinalTime(int index)
   {
      return getYoTrajectory(index).getFinalTime();
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

   public int getNumberOfCoefficients(Direction dir)
   {
      return getYoTrajectory(dir).getNumberOfCoefficients();
   }

   public int getNumberOfCoefficients(int index)
   {
      return getYoTrajectory(index).getNumberOfCoefficients();
   }

   public void reset()
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).reset();
   }

   public void set(YoTrajectory3D other)
   {
      xTrajectory.set(other.getYoTrajectoryX());
      yTrajectory.set(other.getYoTrajectoryY());
      zTrajectory.set(other.getYoTrajectoryZ());
   }

   public void setConstant(double t0, double tFinal, Point3DReadOnly z)
   {
      for(int index = 0; index < 3; index ++)
         getYoTrajectory(index).setConstant(t0, tFinal, z.getElement(index));
   }

   public void setCubic(double t0, double tFinal, Point3DReadOnly z0, Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setCubic(t0, tFinal, z0.getElement(index), zFinal.getElement(index));
   }

   public void setCubic(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setCubic(t0, tFinal, z0.getElement(index), zd0.getElement(index), zFinal.getElement(index), zdFinal.getElement(index));
   }

   public void setCubicInitialPositionThreeFinalConditions(double t0, double tFinal, Point3DReadOnly z0, Point3DReadOnly zFinal, Vector3DReadOnly zdFinal,
                                                           Vector3DReadOnly zddFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setCubicInitialPositionThreeFinalConditions(t0, tFinal, z0.getElement(index), zFinal.getElement(index),
                                                                            zdFinal.getElement(index), zddFinal.getElement(index));
   }

   public void setCubicThreeInitialConditionsFinalPosition(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0,
                                                           Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setCubicThreeInitialConditionsFinalPosition(t0, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index),
                                                                            zFinal.getElement(index));
   }

   public void setCubicUsingFinalAccelerationButNotFinalPosition(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdFinal,
                                                                 Vector3DReadOnly zddFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setCubicUsingFinalAccelerationButNotFinalPosition(t0, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                                  zdFinal.getElement(index), zddFinal.getElement(index));
   }

   public void setCubicUsingIntermediatePoints(double t0, double tIntermediate1, double tIntermediate2, double tFinal, Point3DReadOnly z0,
                                               Point3DReadOnly zIntermediate1, Point3DReadOnly zIntermediate2, Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setCubicUsingIntermediatePoints(t0, tIntermediate1, tIntermediate2, tFinal, z0.getElement(index),
                                                                zIntermediate1.getElement(index), zIntermediate2.getElement(index), zFinal.getElement(index));
   }

   public void setCubicUsingIntermediatePoint(double t0, double tIntermediate1, double tFinal, Point3DReadOnly z0, Point3DReadOnly zIntermediate1,
                                              Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setCubicUsingIntermediatePoint(t0, tIntermediate1, tFinal, z0.getElement(index), zIntermediate1.getElement(index),
                                                               zFinal.getElement(index));
   }

   public void setCubicWithIntermediatePositionAndFinalVelocityConstraint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0,
                                                                          Point3DReadOnly zIntermediate, Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setCubicWithIntermediatePositionAndFinalVelocityConstraint(t0, tIntermediate, tFinal, z0.getElement(index),
                                                                                           zIntermediate.getElement(index), zFinal.getElement(index),
                                                                                           zdFinal.getElement(index));
   }

   public void setCubicWithIntermediatePositionAndInitialVelocityConstraint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0,
                                                                            Vector3DReadOnly zd0, Point3DReadOnly zIntermediate, Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setCubicWithIntermediatePositionAndInitialVelocityConstraint(t0, tIntermediate, tFinal, z0.getElement(index),
                                                                                             zd0.getElement(index), zIntermediate.getElement(index),
                                                                                             zFinal.getElement(index));
   }

   public void setCubicBezier(double t0, double tFinal, Point3DReadOnly z0, Point3DReadOnly zR1, Point3DReadOnly zR2, Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setCubicBezier(t0, tFinal, z0.getElement(index), zR1.getElement(index), zR2.getElement(index), zFinal.getElement(index));
   }

   public void setInitialPositionVelocityZeroFinalHighOrderDerivatives(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                                                                       Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setInitialPositionVelocityZeroFinalHighOrderDerivatives(t0, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                                        zFinal.getElement(index), zdFinal.getElement(index));
   }

   public void setLinear(double t0, double tFinal, Point3DReadOnly z0, Point3DReadOnly zf)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setLinear(t0, tFinal, z0.getElement(index), zf.getElement(index));
   }

   public void setNonic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                        Point3DReadOnly zIntermediate0, Vector3DReadOnly zdIntermediate0, Point3DReadOnly zIntermediate1, Vector3DReadOnly zdIntermediate1,
                        Point3DReadOnly zf, Vector3DReadOnly zdf)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setNonic(t0, tIntermediate0, tIntermediate1, tFinal, z0.getElement(index), zd0.getElement(index),
                                         zIntermediate0.getElement(index), zdIntermediate0.getElement(index), zIntermediate1.getElement(index),
                                         zdIntermediate1.getElement(index), zf.getElement(index), zdf.getElement(index));

   }

   public void setQuadratic(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setQuadratic(t0, tFinal, z0.getElement(index), zd0.getElement(index), zFinal.getElement(index));
   }

   public void setQuadraticUsingInitialAcceleration(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setQuadraticUsingInitialAcceleration(t0, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index));
   }

   public void setQuadraticUsingIntermediatePoint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Point3DReadOnly zIntermediate,
                                                  Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setQuadraticUsingIntermediatePoint(t0, tIntermediate, tFinal, z0.getElement(index), zIntermediate.getElement(index),
                                                                   zFinal.getElement(index));
   }

   public void setQuadraticWithFinalVelocityConstraint(double t0, double tFinal, Point3DReadOnly z0, Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setQuadraticWithFinalVelocityConstraint(t0, tFinal, z0.getElement(index), zFinal.getElement(index), zdFinal.getElement(index));
   }

   public void setQuartic(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0, Point3DReadOnly zFinal,
                          Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setQuartic(t0, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index), zFinal.getElement(index),
                                           zdFinal.getElement(index));

   }

   public void setQuarticUsingFinalAcceleration(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zFinal,
                                                Vector3DReadOnly zdFinal, Vector3DReadOnly zddFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setQuarticUsingFinalAcceleration(t0, tFinal, z0.getElement(index), zd0.getElement(index), zFinal.getElement(index),
                                                                 zdFinal.getElement(index), zddFinal.getElement(index));
   }

   public void setQuarticUsingIntermediateVelocity(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                                                   Vector3DReadOnly zdIntermediate, Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setQuarticUsingIntermediateVelocity(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                    zdIntermediate.getElement(index), zFinal.getElement(index), zdFinal.getElement(index));

   }

   public void setQuarticUsingMidPoint(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zMid, Point3DReadOnly zFinal,
                                       Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setQuarticUsingMidPoint(t0, tFinal, z0.getElement(index), zd0.getElement(index), zMid.getElement(index),
                                                        zFinal.getElement(index), zdFinal.getElement(index));

   }

   public void setQuarticUsingOneIntermediateVelocity(double t0, double tIntermediate0, double tIntermediate1, double tFinal, Point3DReadOnly z0,
                                                      Point3DReadOnly zIntermediate0, Point3DReadOnly zIntermediate1, Point3DReadOnly zFinal,
                                                      Vector3DReadOnly zdIntermediate1)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setQuarticUsingOneIntermediateVelocity(t0, tIntermediate0, tIntermediate1, tFinal, z0.getElement(index),
                                                                       zIntermediate0.getElement(index), zIntermediate1.getElement(index),
                                                                       zFinal.getElement(index), zdIntermediate1.getElement(index));

   }

   public void setQuarticUsingWayPoint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zIntermediate,
                                       Point3DReadOnly zf, Vector3DReadOnly zdf)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setQuarticUsingWayPoint(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index), zIntermediate.getElement(index),
                                                        zf.getElement(index), zdf.getElement(index));
   }

   public void setQuintic(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0, Point3DReadOnly zf, Vector3DReadOnly zdf,
                          Vector3DReadOnly zddf)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setQuintic(t0, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index), zf.getElement(index),
                                           zdf.getElement(index), zddf.getElement(index));
   }

   public void setQuinticTwoWaypoints(double t0, double tIntermediate0, double tIntermediate1, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                                      Point3DReadOnly zIntermediate0, Point3DReadOnly zIntermediate1, Point3DReadOnly zf, Vector3DReadOnly zdf)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setQuinticTwoWaypoints(t0, tIntermediate0, tIntermediate1, tFinal, z0.getElement(index), zd0.getElement(index),
                                                       zIntermediate0.getElement(index), zIntermediate1.getElement(index), zf.getElement(index),
                                                       zdf.getElement(index));
   }

   public void setQuinticUsingIntermediateVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                                                                  Vector3DReadOnly zdIntermediate, Vector3DReadOnly zddIntermediate, Point3DReadOnly zFinal,
                                                                  Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setQuinticUsingIntermediateVelocityAndAcceleration(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                                   zdIntermediate.getElement(index), zddIntermediate.getElement(index),
                                                                                   zFinal.getElement(index), zdFinal.getElement(index));
   }

   public void setQuinticUsingWayPoint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0,
                                       Point3DReadOnly zIntermediate, Point3DReadOnly zf, Vector3DReadOnly zdf)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setQuinticUsingWayPoint(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index),
                                                        zIntermediate.getElement(index), zf.getElement(index), zdf.getElement(index));
   }

   public void setQuinticUsingWayPoint2(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0,
                                        Point3DReadOnly zIntermediate, Vector3DReadOnly zdIntermediate, Point3DReadOnly zf)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setQuinticUsingWayPoint2(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index),
                                                         zIntermediate.getElement(index), zdIntermediate.getElement(index), zf.getElement(index));
   }

   public void setSeptic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                         Point3DReadOnly zIntermediate0, Vector3DReadOnly zdIntermediate0, Point3DReadOnly zIntermediate1, Vector3DReadOnly zdIntermediate1,
                         Point3DReadOnly zf, Vector3DReadOnly zdf)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setSeptic(t0, tIntermediate0, tIntermediate1, tFinal, z0.getElement(index), zd0.getElement(index),
                                          zIntermediate0.getElement(index), zdIntermediate0.getElement(index), zIntermediate1.getElement(index),
                                          zdIntermediate1.getElement(index), zf.getElement(index), zdf.getElement(index));

   }

   public void setSepticInitialAndFinalAcceleration(double t0, double tIntermediate0, double tIntermediate1, double tFinal, Point3DReadOnly z0,
                                                    Vector3DReadOnly zd0, Vector3DReadOnly zdd0, Point3DReadOnly zIntermediate0, Point3DReadOnly zIntermediate1,
                                                    Point3DReadOnly zf, Vector3DReadOnly zdf, Vector3DReadOnly zddf)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setSepticInitialAndFinalAcceleration(t0, tIntermediate0, tIntermediate1, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                     zdd0.getElement(index), zIntermediate0.getElement(index), zIntermediate1.getElement(index),
                                                                     zf.getElement(index), zdf.getElement(index), zddf.getElement(index));

   }

   public void setSexticUsingWaypoint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0,
                                      Point3DReadOnly zIntermediate, Point3DReadOnly zf, Vector3DReadOnly zdf, Vector3DReadOnly zddf)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setSexticUsingWaypoint(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index),
                                                       zIntermediate.getElement(index), zf.getElement(index), zdf.getElement(index), zddf.getElement(index));

   }

   public void setSexticUsingWaypointVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                                                             Vector3DReadOnly zdd0, Vector3DReadOnly zdIntermediate, Vector3DReadOnly zddIntermediate,
                                                             Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).setSexticUsingWaypointVelocityAndAcceleration(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                              zdd0.getElement(index), zdIntermediate.getElement(index),
                                                                              zddIntermediate.getElement(index), zFinal.getElement(index),
                                                                              zdFinal.getElement(index));

   }

   public String toString()
   {
      return "X: " + xTrajectory.toString() + "\n" + "Y: " + yTrajectory.toString() + "\n" + "Z: " + zTrajectory.toString();
   }

   public void getDerivative(YoTrajectory3D dervTraj)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).getDerivative(dervTraj.getYoTrajectory(index));
   }

   public void getDerivative(YoTrajectory3D dervTraj, int order)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).getDerivative(dervTraj.getYoTrajectory(index), order);
   }

   public void getIntegral(YoTrajectory3D integralTraj)
   {
      for(int index = 0; index < 3; index++)
         getYoTrajectory(index).getIntegral(integralTraj.getYoTrajectory(index));
   }

   public void addTimeOffset(double deltaT)
   {
      for (int index = 0; index < 3; index++)
         getYoTrajectory(index).addTimeOffset(deltaT);
   }

   public void addTimeOffset(YoTrajectory3D trajToCopy, double deltaT)
   {
      set(trajToCopy);
      addTimeOffset(deltaT);
   }
}
