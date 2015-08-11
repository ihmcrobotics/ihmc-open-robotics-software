package us.ihmc.robotics.trajectories;

import org.apache.commons.lang3.tuple.ImmutablePair;

import java.util.ArrayList;

public class TrajectoryND
{

   public class WaypointND{
      public double absTime;
      public double[] position;
      public double[] velocity;
      public double[] acceleration;

      public WaypointND(int dimension)
      {
         position = new double[dimension];
         velocity = new double[dimension];
         acceleration = new double[dimension];
      }
   }

   static public enum TrajectoryImplementation { POLYNOMIAL_3RD_ORDER }; 

   protected ArrayList<Trajectory1D> trajectory = new ArrayList<Trajectory1D>();
   protected ArrayList<String> trajectoryNames = null;

   protected int dimension = 0;
   private boolean needsCompute = true;
   protected double maxAcceleration;
   protected double maxVelocity;

   public TrajectoryND(int dimension, double maxVel, double maxAcc)
   {
      this.dimension = dimension;
      this.maxVelocity = maxVel;
      this.maxAcceleration = maxAcc;

      for(int i=0; i<dimension; i++ )
      {
         trajectory.add( new PolynomialTrajectory(maxVel, maxAcc) );
      }

   }

   public void addNames(ArrayList<String> names )
   {
      trajectoryNames = names;
   }

   public int getNumDimensions(){
      return dimension;
   }

   public int getNumWaypoints(){
      if( dimension>0 )
         return trajectory.get(0).waypoints.size();
      else
         return 0;
   }

   public WaypointND getWaypoint(int index)
   {
      if( needsCompute )
      {
         System.out.println("TrajectoryND.getWaypoint [ERROR]: you might want to call buildTrajectory first");
         return null;
      }

      WaypointND waypoint = new WaypointND(dimension);

      // just use the first one, since they are supposed to match
      waypoint.absTime = trajectory.get(0).waypoints.get(index).absTime;

      for(int i=0; i<dimension; i++)
      {
         waypoint.position[i]     = trajectory.get(i).waypoints.get(index).position;
         waypoint.velocity[i]     = trajectory.get(i).waypoints.get(index).velocity;
         waypoint.acceleration[i] = trajectory.get(i).waypoints.get(index).acceleration;
      }
      return waypoint;
   }

   public void addWaypoint( double[] pos ) throws Exception
   {
      if( pos.length != dimension)
      {
         throw new Exception("wrong size of input. check Trajectory.getNumDimensions");
      }

      for (int i=0; i< dimension; i++){
         trajectory.get(i).addWaypoint( new Waypoint1D( pos[i]) );
      }
   }
   
   public void addWaypoint(double[] time, double[] position, double[] velocity ) throws Exception
   {
      if( time.length != dimension || position.length != dimension || velocity.length != dimension )
      {
         throw new Exception("wrong size of input. check Trajectory.getNumDimensions");
      }

      for (int i=0; i< dimension; i++){
         trajectory.get(i).addWaypoint( new Waypoint1D( time[i], position[i], velocity[i], Waypoint1D.DONT_CARE ) );
      }
   }

   public void clear(){
      for (int i=0; i< dimension; i++){
         trajectory.get(i).clear();
      }
   }

   public void buildTrajectory() throws Exception 
   {
      buildTrajectory( 0.1);
   }

   public void buildTrajectory(double minimumTime) throws Exception 
   {
      int numOfWaypoints = trajectory.get(0).getNumberOfWaypoints();

      needsCompute = false;
      //first get an estimated trajectory for each of them

      for (int i=0; i< dimension; i++){
         trajectory.get(i).buildTrajectory( true );
      }
      
      double originalVelocity[] = new double[ dimension*numOfWaypoints ];
      
      for (int i=0; i< dimension; i++)
      {
         for(int index = 0; index<numOfWaypoints; index++ )
         {
            originalVelocity[ index + i*numOfWaypoints] =  trajectory.get(i).waypoints.get(index).velocity;
         }
      }

      // second. check that absTime is the same (max) for all of them or change it accordingly
      for (int p=0; p<numOfWaypoints; p++ )
      {
         double maxAbsTime = -1e10;
         for (int i=0; i< dimension; i++)
         {
            double absTime = trajectory.get(i).waypoints.get(p).absTime;

            if( maxAbsTime < absTime ){
               maxAbsTime = absTime;
            }
         }
         for (int i=0; i< dimension; i++)
         {
            double timeIncrement = maxAbsTime - trajectory.get(i).waypoints.get(p).absTime;
            // apply to this and all the next ones
            for(int index = p; index<numOfWaypoints; index++ )
            {
               trajectory.get(i).waypoints.get(index).absTime += timeIncrement;
            }
         }
         if( p == numOfWaypoints - 1 ) // last point
         {
            double lastTime = trajectory.get(0).waypoints.get(numOfWaypoints-1).absTime ;
            
            if( lastTime < minimumTime)
            {
               double warpFactor = (minimumTime/lastTime);
               
               for (int i=0; i< dimension; i++)
               {
                  // apply to this and all the next ones
                  for(int index = p; index<numOfWaypoints; index++ )
                  {
                     trajectory.get(i).waypoints.get(index).absTime *= warpFactor;
                  }
               }
            }
         }
      }

      // third, calculate again but keep the times as they are.
      for (int i=0; i< dimension; i++)
      {
         for(int index = 0; index<numOfWaypoints; index++ )
         {
            // need to calculate all the velocities again.
            trajectory.get(i).waypoints.get(index).velocity = originalVelocity[ index + i*numOfWaypoints ];
         }
         trajectory.get(i).buildTrajectory( false );
      }

   }

   public ImmutablePair<Boolean, WaypointND> getNextInterpolatedPoints(double deltaT)
   {
      try{
         if( needsCompute )
         {
            buildTrajectory();
         }
      }
      catch(Exception e){
         e.printStackTrace();
      }

      WaypointND waypoint = new WaypointND(dimension);

      boolean finished = true;

      for(int i=0; i<dimension; i++)
      {
         ImmutablePair<Boolean, Waypoint1D> result = trajectory.get(i).getNextInterpolatedPoint( deltaT );
         waypoint.absTime      =  result.getRight().absTime;
         waypoint.position[i]  =  result.getRight().position;
         waypoint.velocity[i]  =  result.getRight().velocity;
         waypoint.acceleration[i] = result.getRight().acceleration;

         if( result.getLeft().booleanValue() == false ){
            finished = false;
         }
      }   
      return new ImmutablePair<Boolean, WaypointND>( new Boolean(finished), waypoint );
   }


   public Waypoint1D[] getInterpolatedPointsAtTime(double absTime) throws Exception
   {
      if( needsCompute )
      {
         buildTrajectory();
      }
      Waypoint1D[] points = new Waypoint1D[dimension];


      for (int i=0; i< dimension; i++){
         points[i] = trajectory.get(i).getInterpolatedPointAtTime( absTime );

         if( points[i] == null)
         {
            return null;
         }
      }

      return points;
   }
/*
   public void cleanButKeepLast()
   {
      for (int i=0; i< dimension; i++){
         trajectory.get(i).cleanButKeepLast();
      }
   }*/
}
