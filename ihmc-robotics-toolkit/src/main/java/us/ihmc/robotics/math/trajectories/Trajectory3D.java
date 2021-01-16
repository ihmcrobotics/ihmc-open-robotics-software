package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class Trajectory3D extends Axis3DPositionTrajectoryGenerator implements Polynomial3DInterface
{
   protected final Trajectory xTrajectory;
   protected final Trajectory yTrajectory;
   protected final Trajectory zTrajectory;
   protected final Trajectory[] trajectories;

   public Trajectory3D(int maximumNumberOfCoefficients)
   {
      this(new Trajectory(maximumNumberOfCoefficients), new Trajectory(maximumNumberOfCoefficients), new Trajectory(maximumNumberOfCoefficients));
   }

   public Trajectory3D(Trajectory[] trajectory)
   {
      this(trajectory[0], trajectory[1], trajectory[2]);

      if (trajectory.length != 3)
         throw new RuntimeException("Expected 3 YoTrajectories for representing the three axes X, Y, and Z, but had: " + trajectory.length
               + " YoTrajectories.");
   }

   public Trajectory3D(List<Trajectory> trajectory)
   {
      this(trajectory.get(0), trajectory.get(1), trajectory.get(2));

      if (trajectory.size() != 3)
         throw new RuntimeException("Expected 3 YoTrajectories for representing the three axes X, Y, and Z, but had: " + trajectory.size()
               + " YoTrajectories.");
   }

   public Trajectory3D(Trajectory xTrajectory, Trajectory yTrajectory, Trajectory zTrajectory)
   {
      super(xTrajectory, yTrajectory, zTrajectory);

      this.xTrajectory = xTrajectory;
      this.yTrajectory = yTrajectory;
      this.zTrajectory = zTrajectory;
      trajectories = new Trajectory[] {xTrajectory, yTrajectory, zTrajectory};
   }

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

   public Tuple3DReadOnly getIntegral(double from, double to)
   {
      xIntegralResult = xTrajectory.getIntegral(from, to);
      yIntegralResult = yTrajectory.getIntegral(from, to);
      zIntegralResult = zTrajectory.getIntegral(from, to);
      return integralResult;
   }

   @Override
   public PolynomialInterface getAxis(int index)
   {
      return getTrajectory(index);
   }

   public Trajectory getTrajectory(Axis3D axis)
   {
      return getTrajectory(axis.ordinal());
   }

   public Trajectory getTrajectory(int index)
   {
      return trajectories[index];
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

   public void offsetTrajectoryPosition(double offsetX, double offsetY, double offsetZ)
   {
      getTrajectoryX().offsetTrajectoryPosition(offsetX);
      getTrajectoryY().offsetTrajectoryPosition(offsetY);
      getTrajectoryZ().offsetTrajectoryPosition(offsetZ);
   }

   public void setInitialTimeMaintainingBounds(double tInitial)
   {
      for (int i = 0; i < 3; i++)
         getTrajectory(i).setInitialTimeMaintainingBounds(tInitial);
   }

   public void setFinalTimeMaintainingBounds(double tFinal)
   {
      for (int i = 0; i < 3; i++)
         getTrajectory(i).setFinalTimeMaintainingBounds(tFinal);
   }
   /**
    * Returns the number of coefficients for the trajectory if it is the same for all axes. If not
    * then returns -1
    *
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

   public int getNumberOfCoefficients(Axis3D dir)
   {
      return getTrajectory(dir).getNumberOfCoefficients();
   }

   public int getNumberOfCoefficients(int index)
   {
      return getTrajectory(index).getNumberOfCoefficients();
   }

   public void getCoefficients(int i, DMatrixRMaj coefficientsToPack)
   {
      for (int ordinal = 0; ordinal < 3; ordinal++)
      {
         coefficientsToPack.set(ordinal, 0, getTrajectory(ordinal).getCoefficient(i));
      }
   }

   public void reset()
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).reset();
   }

   public void set(Trajectory3D other)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).set(other.getTrajectory(index));
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

   public double getDerivative(int index, int order, double time)
   {
      return getTrajectory(index).getDerivative(order, time);
   }

   public void getStartPoint(Point3DBasics positionToPack)
   {
      compute(getTimeInterval().getStartTime());
      positionToPack.set(getPosition());
   }

   public void getEndPoint(Point3DBasics positionToPack)
   {
      compute(getTimeInterval().getEndTime());
      positionToPack.set(getPosition());
   }

   public boolean isValidTrajectory()
   {
      return (getTrajectoryX().isValidTrajectory() && getTrajectoryY().isValidTrajectory() && getTrajectoryZ().isValidTrajectory());
   }

   @Override
   public void showVisualization()
   {
   }

   @Override
   public void hideVisualization()
   {
   }
}
