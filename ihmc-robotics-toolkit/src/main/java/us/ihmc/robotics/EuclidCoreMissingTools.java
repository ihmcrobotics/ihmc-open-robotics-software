package us.ihmc.robotics;

import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextDouble;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextMatrix3D;
import static us.ihmc.euclid.tools.EuclidCoreTools.normSquared;

import java.lang.reflect.Field;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
public class EuclidCoreMissingTools
{
   public static final String DEGREE_SYMBOL = "\u00B0";

   public static void floorToGivenPrecision(Tuple3DBasics tuple3d, double precision)
   {
      tuple3d.setX(MathTools.floorToPrecision(tuple3d.getX(), precision));
      tuple3d.setY(MathTools.floorToPrecision(tuple3d.getY(), precision));
      tuple3d.setZ(MathTools.floorToPrecision(tuple3d.getZ(), precision));
   }

   public static void roundToGivenPrecision(Tuple3DBasics tuple3d, double precision)
   {
      tuple3d.setX(MathTools.roundToPrecision(tuple3d.getX(), precision));
      tuple3d.setY(MathTools.roundToPrecision(tuple3d.getY(), precision));
      tuple3d.setZ(MathTools.roundToPrecision(tuple3d.getZ(), precision));
   }

   public static void floorToGivenPrecision(Tuple2DBasics tuple2d, double precision)
   {
      tuple2d.setX(MathTools.floorToPrecision(tuple2d.getX(), precision));
      tuple2d.setY(MathTools.floorToPrecision(tuple2d.getY(), precision));
   }

   public static void roundToGivenPrecision(Tuple2DBasics tuple2d, double precision)
   {
      tuple2d.setX(MathTools.roundToPrecision(tuple2d.getX(), precision));
      tuple2d.setY(MathTools.roundToPrecision(tuple2d.getY(), precision));
   }

   // *** NOTE ***: The 4x4 output matrix produced by this method assumes a Quaternion component ordering of:
   //   Quat = [ Qs
   //            Qx
   //            Qy
   //            Qz ]
   public static DMatrixRMaj quaternionDotToOmegaTransform(QuaternionReadOnly rotatingFrameQuaternion)
   {
      double qs = rotatingFrameQuaternion.getS();
      double qx = rotatingFrameQuaternion.getX();
      double qy = rotatingFrameQuaternion.getY();
      double qz = rotatingFrameQuaternion.getZ();

      DMatrixRMaj E = new DMatrixRMaj(4,4);

      E.set(0,0, qs); E.set(0,1, qx); E.set(0,2, qy); E.set(0,3, qz);
      E.set(1,0,-qx); E.set(1,1, qs); E.set(1,2, qz); E.set(1,3,-qy);
      E.set(2,0,-qy); E.set(2,1,-qz); E.set(2,2, qs); E.set(2,3, qx);
      E.set(3,0,-qz); E.set(3,1, qy); E.set(3,2,-qx); E.set(3,3, qs);
      
      return E;
   }

   /**
    * Get the orientation as yaw pitch roll String but they are in degrees.
    * Says yaw-pitch-roll.
    */
   public static String getYawPitchRollStringDegrees(Orientation3DBasics orientation3DBasics)
   {
      // Degree symbol placed at the end so you don't have to remove it when copy and pasting
      return EuclidCoreIOTools.getYawPitchRollString(EuclidCoreIOTools.DEFAULT_FORMAT,
                                                     Math.toDegrees(orientation3DBasics.getYaw()),
                                                     Math.toDegrees(orientation3DBasics.getPitch()),
                                                     Math.toDegrees(orientation3DBasics.getRoll())) + DEGREE_SYMBOL;
   }

   /**
    * Get the orientation as yaw pitch roll String but they are in degrees.
    * Doesn't say yaw-pitch-roll.
    */
   public static String getYawPitchRollValuesStringDegrees(Orientation3DBasics orientation3DBasics)
   {
      // Degree symbol placed at the end so you don't have to remove it when copy and pasting
      return EuclidCoreIOTools.getStringOf("(", ")", ", ",
                                           EuclidCoreIOTools.DEFAULT_FORMAT,
                                           Math.toDegrees(orientation3DBasics.getYaw()),
                                           Math.toDegrees(orientation3DBasics.getPitch()),
                                           Math.toDegrees(orientation3DBasics.getRoll())) + DEGREE_SYMBOL;
   }

   /**
    * Remove when this issue is fixed:
    * https://github.com/ihmcrobotics/euclid/issues/57
    */
   private static final Field referenceFrameHasBeenRemoved;
   static
   {
      try
      {
         referenceFrameHasBeenRemoved = ReferenceFrame.class.getDeclaredField("hasBeenRemoved");
         referenceFrameHasBeenRemoved.setAccessible(true);
      }
      catch (NoSuchFieldException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static boolean hasBeenRemoved(ReferenceFrame referenceFrame)
   {
      try
      {
         return referenceFrameHasBeenRemoved.getBoolean(referenceFrame);
      }
      catch (IllegalAccessException e)
      {
         throw new RuntimeException(e);
      }
   }

   /**
    * Remove when this issue is fixed:
    * https://github.com/ihmcrobotics/euclid/issues/57
    */
   private static final Field referenceFrameName;
   static
   {
      try
      {
         referenceFrameName = ReferenceFrame.class.getDeclaredField("frameName");
         referenceFrameName.setAccessible(true);
      }
      catch (NoSuchFieldException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static String frameName(ReferenceFrame referenceFrame)
   {
      try
      {
         return referenceFrameName.get(referenceFrame).toString();
      }
      catch (IllegalAccessException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static FrameVector3DReadOnly newLinkedFrameVector3DReadOnly(ReferenceFrameHolder referenceFrameHolder, DMatrixRMaj source)
   {
      return newLinkedFrameVector3DReadOnly(referenceFrameHolder, 0, source);
   }

   public static FrameVector3DReadOnly newLinkedFrameVector3DReadOnly(ReferenceFrameHolder referenceFrameHolder, int startIndex, DMatrixRMaj source)
   {
      int xIndex = startIndex;
      int yIndex = startIndex + 1;
      int zIndex = startIndex + 2;
      return EuclidFrameFactories.newLinkedFrameVector3DReadOnly(referenceFrameHolder,
                                                                 () -> source.get(xIndex, 0),
                                                                 () -> source.get(yIndex, 0),
                                                                 () -> source.get(zIndex, 0));
   }
}