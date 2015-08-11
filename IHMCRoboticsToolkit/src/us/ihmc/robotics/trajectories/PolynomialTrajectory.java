package us.ihmc.robotics.trajectories;

import java.util.ArrayList;

public class PolynomialTrajectory extends Trajectory1D
{
   final static private double EPS = 0.00001;

   class Poly{
      public double A;
      public double B;
      public double C;
      public double D;
      public double dT;  

      public double getPos(double t)
      {
         return t*(t*(A*t + B) + C) + D;
      }

      public double getVel(double t)
      {
         return t*(3*A*t + 2*B) + C;
      }

      public double getAcc(double t)
      {
         return (6*A*t + 2*B);
      }

      public double getMaxVel()
      {
         double maxVel = Math.max( Math.abs(C) , Math.abs(getVel(dT)));

         if ( Math.signum( B ) != Math.signum( getAcc(dT)) &&  Math.abs(A) > EPS)
         {
            double zeroCrossingT = -B/(3*A);
            maxVel = Math.max( maxVel, Math.abs(getVel(zeroCrossingT)));
         }
         return maxVel;
      }

      public double getMaxAcc()
      {
         //   System.out.format("\ngetMaxAcc: [%.3f]  %.3f\t%.3ff\t%.3ff\t%.3ff\t...... ", dT, A,B,C,D);
         return Math.max( Math.abs(B *2) ,  Math.abs(getAcc(dT) ) );
      }

      public void compute(double dT, double P0, double V0, double P1, double V1) throws Exception
      {
         this.dT = dT;

//         ///----------
//         if( false )
//         {
//            D = P0;
//            C = (P1-P0)/dT;
//            A = 0.0;
//            B = 0.0;
//            return;
//         }
//         //-----------
//         else{
            D = P0;
            C = V0;

            double C0 = (V1-V0) / dT;
            double C1 = ((P1-P0)/dT - V0);

            if( Math.abs(C0) < EPS && Math.abs(C1) < EPS )
            {
               A = 0.0;
               B = 0.0;
            }
            else
            {
               double m00 = 3.0*dT;
               double m01 = 2.0;
               double m10 = dT*dT;
               double m11 = dT;

               double Det  = m00*m11 - m01*m10;
               double DetA = C0*m11 -  C1*m01;
               double DetB = m00*C1 -  m10*C0;

               if( Math.abs(Det) < 0.000001 || Math.abs(dT) < 0.000001)
               {
                  System.out.format(">>%f %f \n  %f %f %f %f  \n  %f %f\n",C0, C1,  m00,m11, m01 , m10, m00*m11,  m01*m10  );

                  throw new Exception("Cant solve polynomial segment given: "+
                        dT + " // " + P0 + " / " + V0 + " // " + P1 + " / " + V1  + " // " + Det );
               }
               A = DetA/Det;
               B = DetB/Det; 
            }
//         }
      }
   }

   ArrayList<Poly> polyList = new ArrayList<Poly>();

   public void printSegmentsInfo()
   {
      int index = 0;
      for ( Poly poly: polyList)
      {
         System.out.format( "%.3f\t%.3f\t%.3f\t / ", poly.dT, poly.getMaxVel(), poly.getMaxAcc() );
         System.out.format( "%.3f\t%.3f\t | ", waypoints.get(index).position,  waypoints.get(index+1).position);
         index++;
      }
      System.out.println();
   }

   public PolynomialTrajectory(double maxVel, double maxAcc)
   {
      super(maxVel, maxAcc);
   }

   private void proposeUnknownVelocities(double trajectoryMaxVel, boolean[] unknownVelocity)
   {
      if( unknownVelocity[0] )
      {
         Waypoint1D Wini = waypoints.get(0);
         Wini.absTime  = 0.0;
         Wini.velocity = 0.0;

         Waypoint1D Wfin = waypoints.get( waypoints.size() -1 );
         Wfin.velocity = 0.0;
      }

      //----------------------------------
      for (int i = 1; i < waypoints.size() -1; i++) 
      {
         if( unknownVelocity[i] )
         {
            Waypoint1D W     = waypoints.get(i);  
            Waypoint1D Wprev = waypoints.get(i-1);
            Waypoint1D Wnext = waypoints.get(i+1);

            double dP_prev = W.position     - Wprev.position;
            double dP_next = Wnext.position - W.position;  

            if( Math.signum(dP_next) != Math.signum( dP_prev ) )
            {
               W.velocity = 0.0;
            }
            else{
               double dT_prev = W.absTime     - Wprev.absTime;
               double dT_next = Wnext.absTime - W.absTime;

               double vel_prev = 0.0;
               double vel_next = 0.0;

               if( Math.abs( dT_prev ) > EPS)
                  vel_prev = (dP_prev / dT_prev);

               if( Math.abs( dT_next ) > EPS)
                  vel_next = (dP_next / dT_next);


               W.velocity = (vel_prev*dT_next + vel_next*dT_prev) / ( dT_prev + dT_next ) ;
               //  W.velocity = (vel_prev + vel_next) / ( 2.0 ) ;
               //  W.velocity = (vel_prev*dT_prev + vel_next*dT_next) / ( dT_prev + dT_next ) ;

               if( W.velocity > trajectoryMaxVel ){
                  W.velocity = trajectoryMaxVel;
               }
            }
         }
      }
   }

   private void proposeUnknownTimes( double trajectoryMaxVel)
   {
      if(waypoints.get(0).absTime == Waypoint1D.DONT_CARE)
      {
         waypoints.get(0).absTime = 0;
      }

      for (int i = 1; i < waypoints.size() ; i++) 
      {
         if( waypoints.get(i).absTime == Waypoint1D.DONT_CARE )
         {
            Waypoint1D Wprev = waypoints.get(i-1);
            Waypoint1D W     = waypoints.get(i);

            double dT = Math.abs(1.0* (W.position - Wprev.position)/ trajectoryMaxVel);
            if( dT < 0.01) dT = 0.01;
            W.absTime = Wprev.absTime + dT;
         }
      }
   }

   private void computePolyList(double[] dT,  boolean[] unknownVelocity) throws Exception
   {
      for (int i = 0; i < waypoints.size() -1 ; i++) 
      {
         waypoints.get(i+1).absTime = waypoints.get(i).absTime + dT[i];
      }
      proposeUnknownVelocities(maxVelocity, unknownVelocity);

      for (int i = 1; i < waypoints.size() ; i++) 
      {
         Waypoint1D Wprev = waypoints.get(i-1);
         Waypoint1D W     = waypoints.get(i);
         W.absTime = Wprev.absTime + dT[i-1];
      }

      for (int i = 0; i < polyList.size() ; i++) 
      {
         Waypoint1D W0 = waypoints.get(i);
         Waypoint1D W1 = waypoints.get(i+1);
         Poly poly = polyList.get(i);
         poly.compute(dT[i], W0.position, W0.velocity, W1.position, W1.velocity);
      }
   }

   @Override
   public void buildTrajectory(boolean allowTimeStretching) throws Exception
   {
      if( getNumberOfWaypoints() < 2)
      {
         return ;
      }

      boolean[] unknownVelocity = new boolean[ waypoints.size()];

      for (int i = 0; i < waypoints.size() ; i++) 
      {
         unknownVelocity[i] = ( waypoints.get(i).velocity == Waypoint1D.DONT_CARE);
      }

      proposeUnknownTimes(maxVelocity);
      proposeUnknownVelocities(maxVelocity, unknownVelocity);

      double[] dT = new double[ waypoints.size() -1 ];

      while( polyList.size() < getNumberOfSegments() ) 
      {
         polyList.add( new Poly() );
      }

      for (int i = 0; i < getNumberOfSegments() ; i++) 
      {
         Waypoint1D W0 = waypoints.get(i);
         Waypoint1D W1 = waypoints.get(i+1);
         dT[i] = W1.absTime - W0.absTime;
      }

      computePolyList(dT, unknownVelocity);

      if( allowTimeStretching )
      {
         boolean repeat = false;
         boolean recompute = false;

         do{
            repeat = false;

            for (int i = 0; i < polyList.size() ; i++) 
            {
               Poly poly = polyList.get(i);
               double vel = poly.getMaxVel();

               if( vel > maxVelocity*1.01)
               {
                  dT[i] *= vel/maxVelocity;
                  repeat = true;
                  recompute = true;
               }
            }

            if( recompute ){
               computePolyList(dT, unknownVelocity);
               recompute = false;
            }

            for (int i = 0; i < polyList.size() ; i++) 
            {
               Poly poly = polyList.get(i);
               double acc = poly.getMaxAcc();

               if( acc > maxAcceleration*1.01){
                  dT[i] *= 1.01*Math.sqrt(acc/maxAcceleration);
                  repeat = true;
                  recompute = true;
               }
            }

            if( recompute ){
               computePolyList(dT, unknownVelocity);
               recompute = false;
            }

         }while(repeat); 
      }

      needsCompute = false;
   }

   @Override
   public Waypoint1D getInterpolatedPointAtTime(double absTime)
   {
      if( getNumberOfWaypoints() == 0)
      {
         return null;
      }
      if( getNumberOfWaypoints() == 1)
      {
         Waypoint1D temp = new Waypoint1D( );
         temp.absTime  = absTime;
         temp.position = waypoints.get(0).position;
         temp.velocity     = 0.0;
         temp.acceleration = 0.0;
      }

      try{
         if( needsCompute )
         {
            buildTrajectory(true);
         }
      }
      catch(Exception e){
         e.printStackTrace();  
      }

      int last = waypoints.size() - 1;
      if( absTime < waypoints.get(0).absTime)    return  waypoints.get(0);
      if( absTime > waypoints.get(last).absTime) return  waypoints.get(last);

      for (int i=0; i < waypoints.size()-1; i++)
      {
         Waypoint1D W0 = waypoints.get(i);
         Waypoint1D W1 = waypoints.get(i+1);
         if( W0.absTime <= absTime && absTime <= W1.absTime)
         {
            double t = absTime - W0.absTime;
            Poly poly = polyList.get(i);

            lastInterpolatedWaypoint.absTime      = absTime;
            lastInterpolatedWaypoint.position     = poly.getPos(t);
            lastInterpolatedWaypoint.velocity     = poly.getVel(t);
            lastInterpolatedWaypoint.acceleration = poly.getAcc(t);

            return new Waypoint1D( lastInterpolatedWaypoint );
         }
      }
      return null;
   }

}
