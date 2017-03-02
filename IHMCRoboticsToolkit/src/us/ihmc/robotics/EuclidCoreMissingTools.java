package us.ihmc.robotics;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

public class EuclidCoreMissingTools
{

   public static void floorToGivenPrecision(Tuple3DBasics tuple3d, double precision)
   {
      tuple3d.setX(MathTools.floorToGivenPrecision(tuple3d.getX(), precision));
      tuple3d.setY(MathTools.floorToGivenPrecision(tuple3d.getY(), precision));
      tuple3d.setZ(MathTools.floorToGivenPrecision(tuple3d.getZ(), precision));
      
   }

   public static void roundToGivenPrecision(Tuple3DBasics tuple3d, double precision)
   {
      tuple3d.setX(MathTools.roundToPrecision(tuple3d.getX(), precision));
      tuple3d.setY(MathTools.roundToPrecision(tuple3d.getY(), precision));
      tuple3d.setZ(MathTools.roundToPrecision(tuple3d.getZ(), precision));
   }

   public static boolean isFinite(Tuple3DBasics tuple)
   {
      return Double.isFinite(tuple.getX()) && Double.isFinite(tuple.getY()) && Double.isFinite(tuple.getZ());
   }

}
