package us.ihmc.robotics.geometry;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class Capsule
{
   final public Point3d p1 = new Point3d();
   final public Point3d p2 = new Point3d();
   public double radius;

   public Capsule(Point3d p1_, Point3d p2_, double rad ){
      p1.set(p1_);
      p2.set(p2_);
      radius = rad;
   }
   
   public Capsule(Capsule other){
      p1.set( other.p1 );
      p2.set( other.p2 );
      radius = other.radius;
   }
   
   public void set(Capsule other){
      p1.set( other.p1 );
      p2.set( other.p2 );
      radius = other.radius;
   }
   
   public void transform( RigidBodyTransform trans){
      trans.transform( p1 );
      trans.transform( p2 );
   }

   static private double dot( Vector3d u, Vector3d v){
      return ( u.x * v.x + u.y * v.y + u.z * v.z);
   }
   
   static private Vector3d diff( Point3d u, Point3d v){
      return new Vector3d( u.x - v.x, u.y - v.y, u.z - v.z);
   }
   
   static private void sum( Point3d u, Vector3d v, Point3d res){
      res.set( u.x + v.x, u.y + v.y, u.z + v.z);
   }
   
   static private double dist(Point3d u, Point3d v ){
      double dx = u.x - v.x;
      double dy = u.y - v.y;
      double dz = u.z - v.z;
      return Math.sqrt(dx*dx + dy*dy + dz*dz);
   }

   // dist3D_Segment_to_Segment(): get the 3D minimum distance between 2 segments
   //    Input:  two 3D line segments S1 and S2
   //    Return: the shortest distance between S1 and S2
   static public double distanceQuery(Capsule C1, Capsule C2, Point3d dp1, Point3d dp2)
   {
      double SMALL_NUM = 0.00000001;
      
      Vector3d   u = diff(C1.p2 , C1.p1);
      Vector3d   v = diff(C2.p2 , C2.p1);
      Vector3d   w = diff(C1.p1 , C2.p1);
      double    a = dot(u,u);         // always >= 0
      double    b = dot(u,v);
      double    c = dot(v,v);         // always >= 0
      double    d = dot(u,w);
      double    e = dot(v,w);
      double    D = a*c - b*b;        // always >= 0
      double    sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
      double    tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

      // compute the line parameters of the two closest points
      if (D < SMALL_NUM) { // the lines are almost parallel
         sN = 0.0;         // force using point P0 on segment S1
         sD = 1.0;         // to prevent possible division by 0.0 later
         tN = e;
         tD = c;
      }
      else {                 // get the closest points on the infinite lines
         sN = (b*e - c*d);
         tN = (a*e - b*d);
         if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
         }
         else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
         }
      }

      if (tN < 0.0) {            // tc < 0 => the t=0 edge is visible
         tN = 0.0;
         // recompute sc for this edge
         if (-d < 0.0)
            sN = 0.0;
         else if (-d > a)
            sN = sD;
         else {
            sN = -d;
            sD = a;
         }
      }
      else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
         tN = tD;
         // recompute sc for this edge
         if ((-d + b) < 0.0)
            sN = 0;
         else if ((-d + b) > a)
            sN = sD;
         else {
            sN = (-d +  b);
            sD = a;
         }
      }
      // finally do the division to get sc and tc
      sc = (Math.abs(sN) < SMALL_NUM ? 0.0 :  sN / sD);
      tc = (Math.abs(tN) < SMALL_NUM ? 0.0 : tN / tD);

      // get the difference of the two closest points 
      u.scale(sc);
      sum( C1.p1, u, dp1);
      
      v.scale(tc);
      sum( C2.p1, v, dp2);
      
      Vector3d dP = diff( dp2, dp1);
      dP.normalize();
      double distance_radius_zero = dist(dp1, dp2); 

      if( Math.abs( C1.radius ) > SMALL_NUM )
      {
         dP.scale( C1.radius );
         dp1.add( dP );
            
         dP.scale( -C2.radius / C1.radius );
         dp2.add( dP );
      }
      else{
         dP.scale( -C2.radius );
         dp2.add( dP );
      }
      double distA = distance_radius_zero - C1.radius - C2.radius;
      double distB = diff(dp1, dp2).length();
      double distC = dist(dp1, dp2);
 
      return (distance_radius_zero - C1.radius - C2.radius);   // return the closest distance
   }

}
