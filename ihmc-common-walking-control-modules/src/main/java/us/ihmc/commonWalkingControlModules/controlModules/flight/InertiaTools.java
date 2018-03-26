package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.euclid.Axis;

/**
 * Some generic functions that help in converting to and from the 
 * 3x3 tensor notation and the 6x1 vector notation for a rotational inertia matrix
 * @author Apoorv Shrivastava
 * TODO Improve implementation by using enums ?
 */
public class InertiaTools
{
   /**
    * Prevent creation of objects
    */
   private InertiaTools()
   {

   }

   public enum InertiaComponentNames
   {
      XX, XY, XZ, YX, YY, YZ, ZX, ZY, ZZ;

      static final InertiaComponentNames[] values = values();

      public InertiaComponentNames getInertiaComponent(Axis axis1, Axis axis2)
      {
         return valueOf(axis1.toString() + axis2.toString());
      }
      
      public int getVectorNotationIndex(InertiaComponentNames inertiaComponent)
      {
         switch (this)
         {
         case XX: return 0;
         case YY: return 1;
         case ZZ: return 2;
         case XY:
         case YX: return 3;
         case YZ:
         case ZY: return 4;
         case XZ:
         case ZX: return 5;
         default:
            throw new RuntimeException("Unrecognized inertia component");
         }
      }
   }

   public static final int angularAxes = Axis.values.length;
   public static final int inertiaComponentsSize = InertiaComponentNames.values().length;
   private static final int[] firstCoordinateForTensorMapping = {0, 1, 2, 0, 1, 2};
   private static final int[] secondCoordinateForTensorMapping = {0, 1, 2, 1, 2, 0};
   private static final int[][] indiceForVectorMapping = {{0, 3, 5}, {3, 1, 4}, {5, 4, 2}};

   public static int getFirstCoordinateForMappingToTensor(int vectorIndex)
   {
      return firstCoordinateForTensorMapping[vectorIndex];
   }

   public static int getSecondCoordinateForMappingToTensor(int vectorIndex)
   {
      return secondCoordinateForTensorMapping[vectorIndex];
   }

   public static int getIndexForVectorMapping(int row, int column)
   {
      return indiceForVectorMapping[row][column];
   }
}
