package us.ihmc.robotics.math.trajectories;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;

/**
 * This class does a cubic interpolation between the provided waypoints.
 * 
 * Procedure for use:
 * 1. setWaypoints(double[] timeAtWaypoints, double[] positions, double[] velocities)
 *    clears the generator and appends the given waypoints
 * 2. setInitialCondition(double initialPosition, double initialVelocity)
 *    informs the generator of the initial position and velocity
 * 3. compute(double time)
 * 4. getValue(), getVelocity, and getAcceleration()
 */
public class MultipleWaypointsTrajectoryGenerator implements DoubleTrajectoryGenerator
{
   private static final boolean DEBUG = false;

   private final YoVariableRegistry registry;

   private final DoubleYoVariable currentTrajectoryTime;
   private final DoubleYoVariable currentPosition;
   private final DoubleYoVariable currentVelocity;
   private final DoubleYoVariable currentAcceleration;
   
   private final IntegerYoVariable currentSubTrajectoryIndex;
   private final DoubleYoVariable currentSubTrajectoryStartPosition;
   private final DoubleYoVariable currentSubTrajectoryStartVelocity;
   private final DoubleYoVariable currentSubTrajectoryStartTime;
   private final DoubleYoVariable currentSubTrajectoryEndPosition;
   private final DoubleYoVariable currentSubTrajectoryEndVelocity;
   private final DoubleYoVariable currentSubTrajectoryEndTime;

   private final IntegerYoVariable numberOfWaypoints;
   private int size = 0;
   
   private final TDoubleArrayList timeAtWaypoints = new TDoubleArrayList();
   private final TDoubleArrayList positionAtWaypoints = new TDoubleArrayList();
   private final TDoubleArrayList velocityAtWaypoints = new TDoubleArrayList();
   private final YoPolynomial subTrajectory;

   public MultipleWaypointsTrajectoryGenerator(String namePrefix, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.currentTrajectoryTime = new DoubleYoVariable(namePrefix + "TrajectoryTime", registry);

      this.currentPosition = new DoubleYoVariable(namePrefix + "TrajectoryPosition", registry);
      this.currentVelocity = new DoubleYoVariable(namePrefix + "TrajectoryVelocity", registry);
      this.currentAcceleration = new DoubleYoVariable(namePrefix + "TrajectoryAcceleration", registry);
      
      this.currentSubTrajectoryIndex = new IntegerYoVariable(namePrefix + "CurrentSubTrajectoryIndex", registry);
      this.currentSubTrajectoryStartPosition = new DoubleYoVariable(namePrefix + "CurrentSubTrajectoryStartPosition", registry);
      this.currentSubTrajectoryStartVelocity = new DoubleYoVariable(namePrefix + "CurrentSubTrajectoryStartVelocity", registry);
      this.currentSubTrajectoryStartTime = new DoubleYoVariable(namePrefix + "CurrentSubTrajectoryStartTime", registry);
      this.currentSubTrajectoryEndPosition = new DoubleYoVariable(namePrefix + "CurrentSubTrajectoryEndPosition", registry);
      this.currentSubTrajectoryEndVelocity = new DoubleYoVariable(namePrefix + "CurrentSubTrajectoryEndVelocity", registry);
      this.currentSubTrajectoryEndTime = new DoubleYoVariable(namePrefix + "CurrentSubTrajectoryEndTime", registry);

      numberOfWaypoints = new IntegerYoVariable(namePrefix + "NumberOfWaypoints", registry);

      timeAtWaypoints.add(Double.NaN);
      positionAtWaypoints.add(Double.NaN);
      velocityAtWaypoints.add(Double.NaN);
      subTrajectory = new YoPolynomial(namePrefix + "SubPolynomial", 4, registry);
      
      parentRegistry.addChild(registry);
   }
   
   private void grow()
   {
      size++;
      timeAtWaypoints.add(Double.NaN);
      positionAtWaypoints.add(Double.NaN);
      velocityAtWaypoints.add(Double.NaN);
   }

   public void clear()
   {
      numberOfWaypoints.set(0);

      for (int i = 0; i < size; i++)
      {
         timeAtWaypoints.set(i, Double.NaN);
         positionAtWaypoints.set(i, Double.NaN);
         velocityAtWaypoints.set(i, Double.NaN);
      }
      
      initialize();
   }

   /**
    * This function appends a waypoint to the current trajectory. If the time value is invalid it will return false
    * and all waypoints will be cleared.
    * 
    * @param timeAtWaypoint
    * @param position
    * @param velocity
    * @return success
    */
   public boolean appendWaypoint(double timeAtWaypoint, double position, double velocity)
   {
      // the index is shifted 1 place because the index == 0 is used to store the current value of the variable.
      int index = numberOfWaypoints.getIntegerValue() + 1;
      
      while (numberOfWaypoints.getIntegerValue() + 1 > size)
      {
         grow();
      }
      
      // check if waypoint time is valid
      boolean timeValid = checkTimeValid(timeAtWaypoint);
      
      if (!timeValid || DEBUG)
      {
         System.out.format("Trying to add waypoint at time=%f position=%f velocity=%f - last waypoint time=%f\n", 
               timeAtWaypoint, position, velocity, timeAtWaypoints.get(index - 1));
         
         if (!timeValid)
         {
            System.err.println(this.getClass().getSimpleName() + " can not append waypoint: time for consecutive waypoints "
                  + "must be increasing.");
            
            clear();
            return false;
         }
      }

      timeAtWaypoints.set(index, timeAtWaypoint);
      positionAtWaypoints.set(index, position);
      velocityAtWaypoints.set(index, velocity);

      numberOfWaypoints.increment();
      return true;
   }
   
   private boolean checkTimeValid(double nextTime)
   {
      int currentTimeIndex = numberOfWaypoints.getIntegerValue();
      double currentTime;
      if (currentTimeIndex == 0)
      {
         currentTime = 0;
      }
      else
      {
         currentTime = timeAtWaypoints.get(currentTimeIndex);
      }
      
      return nextTime > currentTime;
   }

   public void appendWaypoints(double[] timeAtWaypoints, double[] positions, double[] velocities)
   {
      if (timeAtWaypoints.length != positions.length || positions.length != velocities.length)
         throw new RuntimeException("Arguments are inconsistent.");

      for (int i = 0; i < timeAtWaypoints.length; i++)
      {
         boolean success = appendWaypoint(timeAtWaypoints[i], positions[i], velocities[i]);
         if (!success)
         {
            return;
         }
      }
   }

   public void setWaypoints(double[] timeAtWaypoints, double[] positions, double[] velocities)
   {
      clear();
      appendWaypoints(timeAtWaypoints, positions, velocities);
   }

   @Override
   public void initialize()
   {
      currentTrajectoryTime.set(0.0);
      currentSubTrajectoryIndex.set(-1);
   }
   
   private void initializeSubTrajectory(int subTrajectoryIndex)
   {
      currentSubTrajectoryStartPosition.set(positionAtWaypoints.get(subTrajectoryIndex));
      currentSubTrajectoryStartVelocity.set(velocityAtWaypoints.get(subTrajectoryIndex));
      currentSubTrajectoryStartTime.set(timeAtWaypoints.get(subTrajectoryIndex));
      currentSubTrajectoryEndPosition.set(positionAtWaypoints.get(subTrajectoryIndex + 1));
      currentSubTrajectoryEndVelocity.set(velocityAtWaypoints.get(subTrajectoryIndex + 1));
      currentSubTrajectoryEndTime.set(timeAtWaypoints.get(subTrajectoryIndex + 1));
      
      subTrajectory.setCubic(currentSubTrajectoryStartTime.getDoubleValue(), currentSubTrajectoryEndTime.getDoubleValue(),
            currentSubTrajectoryStartPosition.getDoubleValue(), currentSubTrajectoryStartVelocity.getDoubleValue(),
            currentSubTrajectoryEndPosition.getDoubleValue(), currentSubTrajectoryEndVelocity.getDoubleValue());
   }

   /**
    * call this function to let the trajectory generator know what the initial condition at time zero is. You
    * can not set this information as a waypoint.
    * 
    * @param initialPosition
    * @param initialVelocity
    */
   public void setInitialCondition(double initialPosition, double initialVelocity)
   {
      timeAtWaypoints.set(0, 0.0);
      positionAtWaypoints.set(0, initialPosition);
      velocityAtWaypoints.set(0, initialVelocity);
      
      initialize();
   }

   @Override
   public void compute(double time)
   {
      if (numberOfWaypoints.getIntegerValue() == 0)
      {
         currentPosition.set(positionAtWaypoints.get(0));
         currentVelocity.set(0.0);
         currentAcceleration.set(0.0);
         return;
      }

      double firstT = timeAtWaypoints.get(0);
      int lastWaypoint = numberOfWaypoints.getIntegerValue();
      int lastSegment = lastWaypoint - 1;
      double lastT  = timeAtWaypoints.get(lastWaypoint);

      if (time <= firstT)
      {
         updateCurrent(0, time);
      }
      else if(time >= lastT)
      {
         updateCurrent(lastSegment, lastT);
      }
      else
      {
         for (int i = 0; i <= lastSegment; i++)
         {
            if (time < timeAtWaypoints.get(i+1))
            {
               updateCurrent(i, time);
               break;
            }
         }
      }

      this.currentTrajectoryTime.set(time);
   }

   private void updateCurrent(int subTrajectoryIndex, double time)
   {
      if (currentSubTrajectoryIndex.getIntegerValue() != subTrajectoryIndex)
      {
         initializeSubTrajectory(subTrajectoryIndex);
         currentSubTrajectoryIndex.set(subTrajectoryIndex);
      }
      
      subTrajectory.compute(time);
      
      double position = subTrajectory.getPosition();
      double velocity = subTrajectory.getVelocity();
      double acceleration = subTrajectory.getAcceleration();
      
      if(Double.isNaN(position) | Double.isNaN(velocity) | Double.isNaN(acceleration))
      {
         throw new RuntimeException(getClass().getSimpleName() + ": Something is NaN in the trajectory generator");
      }
      if (DEBUG)
      {
         System.out.format("goint to waypoint %d time %.3f: pos=%.3f, vel=%.3f\n", subTrajectoryIndex, time, position, velocity);
      }

      currentPosition.set(position);
      currentVelocity.set(velocity);
      currentAcceleration.set(acceleration);
   }

   @Override
   public boolean isDone()
   {
      if (numberOfWaypoints.getIntegerValue() == 0)
         return true;

      double tFinal = timeAtWaypoints.get(numberOfWaypoints.getIntegerValue() - 1);
      return currentTrajectoryTime.getDoubleValue() > tFinal;
   }

   @Override
   public double getValue()
   {
      return currentPosition.getDoubleValue();
   }

   @Override
   public double getVelocity()
   {
      return currentVelocity.getDoubleValue();
   }

   @Override
   public double getAcceleration()
   {
      return currentAcceleration.getDoubleValue();
   }
}
