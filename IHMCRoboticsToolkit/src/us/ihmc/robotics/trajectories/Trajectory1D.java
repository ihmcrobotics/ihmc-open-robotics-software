package us.ihmc.robotics.trajectories;

import org.apache.commons.lang3.tuple.ImmutablePair;

import java.util.ArrayList;
import java.util.Collections;

public abstract class Trajectory1D
{

   protected final ArrayList<Waypoint1D> waypoints = new ArrayList<Waypoint1D>();
   double currentTime = -1;
   protected boolean needsCompute = true;
   protected double maxVelocity;
   protected double maxAcceleration;

   protected final Waypoint1D lastInterpolatedWaypoint = new Waypoint1D();

   public Trajectory1D(double maxVel, double maxAcc) 
   {
      maxVelocity = maxVel;
      maxAcceleration = maxAcc;
   }

   public double getLastTime()
   {
      try{
         if( needsCompute ){
            buildTrajectory(true);
         }
      }
      catch(Exception e){
         e.printStackTrace();
      }

      if( waypoints.size() > 0)
      {
         return waypoints.get( waypoints.size()-1 ).absTime;
      }
      return 0;
   }

   public int getNumberOfWaypoints()
   {
      return waypoints.size();
   }

   public int getNumberOfSegments()
   {
      return waypoints.size() - 1;
   }

   public double getCurrentTime(){
      return currentTime;
   }

   public ImmutablePair<Boolean,Waypoint1D> getNextInterpolatedPoint(double deltaT)
   {
      Boolean finished;
      if( currentTime < 0 && waypoints.size() > 0 )
      {
         currentTime = waypoints.get(0).absTime;

         if(currentTime == Waypoint1D.DONT_CARE)
         {
            currentTime = deltaT;
         }
      }
      else {
         currentTime += deltaT;
      }
      if( currentTime >= waypoints.get(waypoints.size() -1 ).absTime)
      {
         finished = new Boolean(true);
      }
      else{
         finished = new Boolean(false); 
      }
      return new ImmutablePair<Boolean,Waypoint1D>(finished, getInterpolatedPointAtTime( currentTime ) );
   }

   /*
    * The implementation of this method must update lastInterpolatedWaypoint.
    */
   abstract public Waypoint1D getInterpolatedPointAtTime(double absTime);

   /*
    * The implementation should check if needsCompute == true and take the proper action 
    */
   abstract public void buildTrajectory(boolean allowTimeStretching)  throws Exception;


   /*
    * This method will also sort the list. You can assume that the points are always monotonic in time.
    */
   public void addWaypoint(Waypoint1D point)
   {
      // make a copy, don't use the reference.
      waypoints.add( new Waypoint1D(point) );
      Collections.sort(waypoints);
      needsCompute = true;
   }

   public Waypoint1D getLastInterpolatedWaypoint(){
      return new Waypoint1D( lastInterpolatedWaypoint );
   }

   public void clear(){
      waypoints.clear();
      currentTime = -1;
   }

  /* public void cleanButKeepLast(){
      waypoints.clear();
      waypoints.add(  new Waypoint1D( lastInterpolatedWaypoint ) );
   }*/
   
   public abstract void printSegmentsInfo();

}
