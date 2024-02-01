package us.ihmc.behaviors.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

import java.util.ArrayList;

/**
 * This class represents the robot and stores the state of the agent in
 * the Monte Carlo Planner including the current and past locations of the agent
 * as well as for handling the virtual sensor data available on the agent for simulated
 * rollouts in the Monte Carlo Tree Search.
 */
public class MonteCarloPlanningAgent
{
   /**
    * The current position of the agent.
    */
   private final Point2D position = new Point2D();
   /**
    * The previous position of the agent.
    */
   private final Point2D previousPosition = new Point2D();
   /**
    * The average position of the agent. Used for rewarding the planner for moving away from the previously taken path.
    */
   private final Point2D averagePosition = new Point2D();
   /**
    * The virtual range sensor on the agent. Used for internal simulation rollouts in the Monte Carlo Tree Search.
    */
   private final RangeScanner rangeScanner = new RangeScanner(24, 20);
   /**
    * Stores the virtual range sensor scans for the agent. These hold simulated range values based on the
    * simulated agent position and the world occupancy state.
    */
   private final ArrayList<Point2DReadOnly> scanPoints = new ArrayList<>();

   public MonteCarloPlanningAgent(Point2DReadOnly position)
   {
      this.position.set(position);
      this.previousPosition.set(position);
      this.averagePosition.set(position);
   }

   public void changeStateTo(Point2DReadOnly newState)
   {
      previousPosition.set(position);
      position.set(newState);

      averagePosition.interpolate(position, 0.05);
   }

   public void changeStateTo(double xPosition, double yPosition)
   {
      previousPosition.set(xPosition, yPosition);
      position.set(xPosition, yPosition);

      averagePosition.interpolate(position, 0.05);
   }

   public void measure(MonteCarloPlanningWorld world)
   {
      scanPoints.clear();
      scanPoints.addAll(rangeScanner.scan(position, world));
   }

   public void setMeasurements(ArrayList<Point2DReadOnly> measurements)
   {
      scanPoints.clear();
      scanPoints.addAll(measurements);
   }

   public Point2DReadOnly getPosition()
   {
      return position;
   }

   public RangeScanner getRangeScanner()
   {
      return rangeScanner;
   }

   public Point2DReadOnly getAveragePosition()
   {
      return averagePosition;
   }

   public Point2DReadOnly getPreviousPosition()
   {
      return previousPosition;
   }

   public ArrayList<Point2DReadOnly> getScanPoints()
   {
      return scanPoints;
   }
}
