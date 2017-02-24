package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.trajectories.YoPolynomial;

/**
 * Trajectory generator that generates a cubic spline in 2D space to go from one 2D point to another
 * and going through two waypoints.
 */
/**
 * @author Sylvain Bertrand
 *
 */
public class YoFourPointCubicSpline1D
{
   private static final double EPS = 1e-3;
   private static final int numberOfCoefficients = 4;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   /** Internal solver used to compute the polynomial coefficients. */
   private final YoPolynomial spline;
   /** Current value of the trajectory. */
   private final DoubleYoVariable y;
   /** Current first derivative value of the trajectory. */
   private final DoubleYoVariable yDot;
   /** Current second derivative value of the trajectory. */
   private final DoubleYoVariable yDDot;

   private final Point2D start = new Point2D();
   private final Point2D end = new Point2D();

   /**
    * Creates a new trajectory generator.
    * <p>
    * The trajectory generator can then be used to generate trajectories as follows:
    * <ul>
    * <li>call {@link #initialize(Point2D, Point2D, Point2D, Point2D)} to provide information about
    * the trajectory to be generated.
    * <li>call {@link #compute(double)} to calculate the trajectory state at the given query
    * coordinate.
    * <li>call {@link #getY()}, {@link #getYDot()}, {@link #getYDDot()} to retrieve the trajectory
    * value, first and second derivatives at the query coordinate it was calculated.
    * </ul>
    * 
    * @param namePrefix prefix used to create the internal {@code YoVariable}s.
    * @param parentRegistry registry to which the registry of this trajectory generator will be
    *           attached.
    */
   public YoFourPointCubicSpline1D(String namePrefix, YoVariableRegistry parentRegistry)
   {
      spline = new YoPolynomial(namePrefix + "CubicSpline", numberOfCoefficients, registry);
      y = new DoubleYoVariable(namePrefix + "Value", registry);
      yDot = new DoubleYoVariable(namePrefix + "Dot", registry);
      yDDot = new DoubleYoVariable(namePrefix + "DDot", registry);

      parentRegistry.addChild(registry);
   }

   /**
    * Provides the information about the trajectory to generate.
    * <p>
    * The user still need to call the method {@link #compute(double)} to actually calculate the
    * trajectory state at a given query point.
    * </p>
    * <p>
    * The trajectory output is a cubic spline of the y-coordinate in function of the x-coordinate.
    * </p>
    * 
    * @param start the starting point of the trajectory. Not modified.
    * @param firstWaypoint the first waypoint the trajectory has to go through. Not modified.
    * @param secondWaypoint the second waypoint the trajectory has to go through. Not modified.
    * @param end the end point of the trajectory. Not modified.
    * @throws RuntimeException if {@code start.getX() > firstWaypoint.getX()}
    * @throws RuntimeException if {@code firstWaypoint.getX() > secondWaypoint.getX()}
    * @throws RuntimeException if {@code secondWaypoint.getX() > end.getX()}
    */
   public void initialize(Point2DReadOnly start, Point2DReadOnly firstWaypoint, Point2DReadOnly secondWaypoint, Point2DReadOnly end)
   {
      double startX = start.getX();
      double startY = start.getY();
      double firstWaypointX = firstWaypoint.getX();
      double firstWaypointY = firstWaypoint.getY();
      double secondWaypointX = secondWaypoint.getX();
      double secondWaypointY = secondWaypoint.getY();
      double endX = end.getX();
      double endY = end.getY();

      initialize(startX, startY, firstWaypointX, firstWaypointY, secondWaypointX, secondWaypointY, endX, endY);
   }

   /**
    * Provides the information about the trajectory to generate.
    * <p>
    * The user still need to call the method {@link #compute(double)} to actually calculate the
    * trajectory state at a given query point.
    * </p>
    * <p>
    * The trajectory output is a cubic spline of the y-coordinate in function of the x-coordinate.
    * </p>
    * 
    * @param startX the x-coordinate of the starting point of the trajectory.
    * @param startY the y-coordinate of the starting point of the trajectory.
    * @param firstWaypointX the x-coordinate of the first waypoint the trajectory has to go through.
    * @param firstWaypointY the y-coordinate of the first waypoint the trajectory has to go through.
    * @param secondWaypointX the x-coordinate of the second waypoint the trajectory has to go through.
    * @param secondWaypointY the y-coordinate of the second waypoint the trajectory has to go through.
    * @param endX the x-coordinate of the end point of the trajectory.
    * @param endY the y-coordinate of the end point of the trajectory.
    * @throws RuntimeException if {@code startX > firstWaypointX}
    * @throws RuntimeException if {@code firstWaypointX > secondWaypointX}
    * @throws RuntimeException if {@code secondWaypointX > endX}
    */
   public void initialize(double startX, double startY, double firstWaypointX, double firstWaypointY, double secondWaypointX, double secondWaypointY,
                          double endX, double endY)
   {
      if (startX > firstWaypointX)
         throw new RuntimeException("x values must be in increasing order!");
      else if (firstWaypointX > secondWaypointX)
         throw new RuntimeException("x values must be in increasing order!");
      else if (secondWaypointX > endX)
         throw new RuntimeException("x values must be in increasing order!");

      this.start.set(startX, startY);
      this.end.set(endX, endY);

      spline.setCubicUsingIntermediatePoints(startX, firstWaypointX, secondWaypointX, endX, startY, firstWaypointY, secondWaypointY, endY);
   }

   /**
    * Computes the trajectory state for a given query point {@code xQuery} and stores the result in
    * the internal memory.
    * <p>
    * Edge cases:
    * <ul>
    * <li>{@code xQuery < start.getX()+} {@link #EPS}, the first and derivative of the trajectory
    * are assumed to be equal to zero.
    * <li>{@code xQuery > end.getX()-} {@link #EPS}, the first and derivative of the trajectory are
    * assumed to be equal to zero.
    * </ul>
    * </p>
    * 
    * @param xQuery the coordinate at which the the trajectory state (y, yDot, yDDot) is to be
    *           calculated.
    */
   public void compute(double xQuery)
   {
      if (xQuery < start.getX() + EPS)
      {
         y.set(start.getY());
         yDot.set(0.0);
         yDDot.set(0.0);
      }
      else if (xQuery > end.getX() - EPS)
      {
         y.set(end.getY());
         yDot.set(0.0);
         yDDot.set(0.0);
      }
      else
      {
         spline.compute(xQuery);
         y.set(spline.getPosition());
         yDot.set(spline.getVelocity());
         yDDot.set(spline.getAcceleration());
      }
   }

   /**
    * Gets the last trajectory value calculated with {@link #compute(double)}.
    * 
    * @return the trajectory value.
    */
   public double getY()
   {
      return y.getDoubleValue();
   }

   /**
    * Gets the last trajectory first derivative value calculated with {@link #compute(double)}.
    * 
    * @return the trajectory first derivative value.
    */
   public double getYDot()
   {
      return yDot.getDoubleValue();
   }

   /**
    * Gets the last trajectory second derivative value calculated with {@link #compute(double)}.
    * 
    * @return the trajectory second derivative value.
    */
   public double getYDDot()
   {
      return yDDot.getDoubleValue();
   }
}
