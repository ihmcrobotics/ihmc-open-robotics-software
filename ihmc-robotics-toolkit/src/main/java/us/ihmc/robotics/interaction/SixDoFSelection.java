package us.ihmc.robotics.interaction;

import us.ihmc.euclid.Axis3D;

/**
 * A class for managing user interaction selections from 6 degrees of freedom.
 */
public enum SixDoFSelection
   {
      CENTER, LINEAR_X, LINEAR_Y, LINEAR_Z, ANGULAR_X, ANGULAR_Y, ANGULAR_Z;

      public boolean isCenter()
      {
         return this == CENTER;
      }

      public boolean isLinear()
      {
         return this == LINEAR_X || this == LINEAR_Y || this == LINEAR_Z;
      }

      public boolean isAngular()
      {
         return this == ANGULAR_X || this == ANGULAR_Y || this == ANGULAR_Z;
      }

      public Axis3D toAxis3D()
      {
         switch (this)
         {
            case CENTER:
               return null;
            case LINEAR_X:
            case ANGULAR_X:
               return Axis3D.X;
            case LINEAR_Y:
            case ANGULAR_Y:
               return Axis3D.Y;
            case LINEAR_Z:
            case ANGULAR_Z:
               return Axis3D.Z;
            default:
               throw new IllegalArgumentException("Unknown selection: " + this);
         }
      }

      public static SixDoFSelection toLinearSelection(Axis3D axis)
      {
         switch (axis)
         {
            case X:
               return LINEAR_X;
            case Y:
               return LINEAR_Y;
            case Z:
               return LINEAR_Z;
            default:
               throw new IllegalArgumentException("Unknown axis: " + axis);
         }
      }

      public static SixDoFSelection toAngularSelection(Axis3D axis)
      {
         switch (axis)
         {
            case X:
               return ANGULAR_X;
            case Y:
               return ANGULAR_Y;
            case Z:
               return ANGULAR_Z;
            default:
               throw new IllegalArgumentException("Unknown axis: " + axis);
         }
      }
   }