package us.ihmc.robotics.trajectories;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.AfterClass;
import org.junit.Test;

import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class PolynomialTrajectoryTest
{
   static private final boolean DEBUG = false;
   private final double dT  = 0.001;
   private final double EPS = 0.001;

   private double randomBetween(double min, double max)
   {
      return min + Math.random() * (max - min);
   }
   
   private void buildAndValidateTrajectory(double[] positions, double maximumVelocity, double maximumAcceleration ) throws Exception
   {
      PolynomialTrajectory trajectory = new PolynomialTrajectory(maximumVelocity, maximumAcceleration);
      
      for(int i=0; i<positions.length; i++ )
      {
         trajectory.addWaypoint(new Waypoint1D( positions[i] ));
      }
      
      // compute the trajectory     
      trajectory.buildTrajectory( true );
      
      double lastTime = trajectory.getLastTime();
      
      // check that you actually pass through the waypoints at the right time
      for(int i=0; i<positions.length; i++ )
      {
         double timeOfWaypoint = trajectory.waypoints.get(i).absTime;
         Waypoint1D interpolatedPoint = trajectory.getInterpolatedPointAtTime( timeOfWaypoint );
         assertEquals( interpolatedPoint.position, positions[i], EPS );
      }
      
      // check that you never exceed maximum velocity and acceleration and
      // compare to the one calculated as finite difference
      for(double t=0; t <= lastTime; t += dT )
      {
         double small_dT = dT/100.0;
         
         //get the result
         Waypoint1D P_prev = trajectory.getInterpolatedPointAtTime(t - small_dT);
         Waypoint1D P      = trajectory.getInterpolatedPointAtTime(t);
         Waypoint1D P_next = trajectory.getInterpolatedPointAtTime(t + small_dT);
         
         // check maximum acceleration and maximum velocity with a 2% margin.
         assertTrue(" velocity is too large: " + P.velocity,         Math.abs( P.velocity)     <= maximumVelocity*1.02 );
         assertTrue(" acceleration is too large: " + P.acceleration, Math.abs( P.acceleration) <= maximumAcceleration*1.02);
         
         double finiteVelocity     = (P_next.position - P_prev.position) / (2.0*small_dT);
         double finiteAcceleration = (P_next.position -2.0*P.position +P_prev.position) / (small_dT*small_dT);
         
         double acceptableDifference = Math.max( EPS, P.velocity * 0.02);
         assertEquals("finite velocity different from calculated velocity",  P.velocity,  finiteVelocity,  acceptableDifference);
         
         // Note: acceleration might be discontinuous at waypoints, so don't check close to them, because finiteAcceleration will be wrong
         boolean checkAcceleration = true;
         for (int a=0; a<positions.length; a++ )
         {
            if( Math.abs( t - trajectory.waypoints.get(a).absTime  ) <= small_dT )
            {
               checkAcceleration = false; 
               break;
            }
         }
         
         if( checkAcceleration )
         {
            acceptableDifference = Math.max( EPS, P.acceleration * 0.02);
            assertEquals("finite acceleration different from calculated acceleration", P.acceleration, finiteAcceleration, acceptableDifference);
         }
      }
   }
   
   @DeployableTestMethod
   @Test(timeout = 30000)
   public void testSimpleTwoPoints() throws Exception
   {
      double[] positions = new double[]{ 0, 1 };
      
      buildAndValidateTrajectory( positions, 10, 20 );
   }
   
   @DeployableTestMethod
   @Test(timeout = 30000)
   public void testMultiplePoints() throws Exception
   {
      double[] positions = new double[]{ 0, 1 ,3 ,4 ,6, -2, -4, 3, -1, 0 };
      
      buildAndValidateTrajectory( positions, 10, 20 );
   }
   
   @DeployableTestMethod
   @Test(timeout = 30000)
   public void testRandomPoints() throws Exception
   {
      for (int iter=0; iter< 100; iter++)
      {
         int numPoints = 2 +  (int) ( randomBetween( 0.0, 5.0) );
         double[] positions = new double[numPoints];
               
         for (int i=0; i<numPoints; i++ )
            positions[i] =  randomBetween( -10.0, 10.0);
         
         double maxVelocity =  randomBetween( 1.0, 50.0 ) ;
         double maxAcceleration =  randomBetween( 5.0, 100.0);
         
         buildAndValidateTrajectory( positions, maxVelocity, maxAcceleration );
      }
   }
   

  // @Ignore
   @DeployableTestMethod
   @Test(timeout = 3000)
   public void testBumpyResult() throws Exception
   {
      double maxVel = 2;
      double maxAcc = 10;
      PolynomialTrajectory trajectory = new PolynomialTrajectory(maxVel, maxAcc);

      trajectory.addWaypoint(new Waypoint1D(0.00));
      trajectory.addWaypoint(new Waypoint1D(0.004));
      trajectory.addWaypoint(new Waypoint1D(0.008));

      trajectory.addWaypoint(new Waypoint1D(0.032));
      trajectory.addWaypoint(new Waypoint1D(0.036));
      trajectory.addWaypoint(new Waypoint1D(0.040));

      trajectory.buildTrajectory(true);

      ImmutablePair<Boolean, Waypoint1D> interpolatedPoint = trajectory.getNextInterpolatedPoint(dT);
      double previousVelocity = interpolatedPoint.getRight().velocity;

      interpolatedPoint = trajectory.getNextInterpolatedPoint(dT);

      int numberOfBumps = 0;

      while (interpolatedPoint.getLeft().booleanValue() == false)
      {
         Waypoint1D wp = interpolatedPoint.getRight();

         double velocity = wp.velocity;
         if (Math.signum(velocity) != Math.signum(previousVelocity))
         {
            numberOfBumps++;
         }

         previousVelocity = velocity;

         interpolatedPoint = trajectory.getNextInterpolatedPoint(dT);
      }
      assertTrue("Monotonic position increments should not have inversion of velocity signum " + numberOfBumps, numberOfBumps <= 1);
   }

   
   @AfterClass
   static public void printExampleTrajectory() throws Exception
   {
            
      if (DEBUG)
      {
         /*
          * double maxVel = 2; double maxAcc = 10; PolynomialTrajectory
          * trajectory = new PolynomialTrajectory(maxVel, maxAcc);
          * trajectory.addWaypoint( new Waypoint1D( 0.00 ) );
          * trajectory.addWaypoint( new Waypoint1D( 0.004 ) );
          * trajectory.addWaypoint( new Waypoint1D( 0.008 ) );
          * trajectory.addWaypoint( new Waypoint1D( 0.032 ) );
          * trajectory.addWaypoint( new Waypoint1D( 0.036 ) );
          * trajectory.addWaypoint( new Waypoint1D( 0.040 ) );
          */
         double maxVel = 2;
         double maxAcc = 3;
         PolynomialTrajectory trajectory = new PolynomialTrajectory(maxVel, maxAcc);
         double DONT_CARE = Waypoint1D.DONT_CARE;

         trajectory.addWaypoint(new Waypoint1D(DONT_CARE, 1, 0.0, DONT_CARE));
         trajectory.addWaypoint(new Waypoint1D(DONT_CARE, 5, 0.0, DONT_CARE));
         trajectory.addWaypoint(new Waypoint1D(DONT_CARE, -5, 0.0, DONT_CARE));
         trajectory.addWaypoint(new Waypoint1D(DONT_CARE, -2, DONT_CARE, DONT_CARE));
         trajectory.addWaypoint(new Waypoint1D(DONT_CARE, -1, DONT_CARE, DONT_CARE));
         trajectory.addWaypoint(new Waypoint1D(DONT_CARE, 4, DONT_CARE, DONT_CARE));
         trajectory.addWaypoint(new Waypoint1D(DONT_CARE, 9, DONT_CARE, DONT_CARE));
         trajectory.addWaypoint(new Waypoint1D(DONT_CARE, 10, 0.0, DONT_CARE));
         trajectory.addWaypoint(new Waypoint1D(DONT_CARE, 0, 0.0, DONT_CARE));

         trajectory.buildTrajectory(true);

         try
         {
            File file = new File("polytest.txt");

            System.out.print("Start");

            // if file doesnt exists, then create it
            if (!file.exists())
            {
               file.createNewFile();
            }

            FileWriter fw = new FileWriter(file.getAbsoluteFile());
            BufferedWriter bw = new BufferedWriter(fw);

            ImmutablePair<Boolean, Waypoint1D> interpolatedPoint = trajectory.getNextInterpolatedPoint(0.001);

            while (interpolatedPoint.getLeft().booleanValue() == false)
            {
               Waypoint1D wp = interpolatedPoint.getRight();
               String content = trajectory.getCurrentTime() + " " + wp.position + " " + wp.velocity + " " + wp.acceleration + "\n";
               bw.write(content);
               System.out.print(content);

               interpolatedPoint = trajectory.getNextInterpolatedPoint(0.001);
            }

            bw.close();
            System.out.print("Done");

         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }

}
