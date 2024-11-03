package us.ihmc.commonWalkingControlModules.controllerCore.data;

public enum SpaceData3D
{
   POSITION("Position")
   {
      @Override
      public SpaceData6D getSpaceData6D()
      {
         return SpaceData6D.POSE;
      }
   },
   ORIENTATION("Orientation")
   {
      @Override
      public SpaceData6D getSpaceData6D()
      {
         return SpaceData6D.POSE;
      }
   },
   ROTATION_VECTOR("RotationVector")
   {
      @Override
      public SpaceData6D getSpaceData6D()
      {
         return SpaceData6D.POSE;
      }
   },
   LINEAR_VELOCITY("LinearVelocity")
   {
      @Override
      public SpaceData6D getSpaceData6D()
      {
         return SpaceData6D.VELOCITY;
      }
   },
   ANGULAR_VELOCITY("AngularVelocity")
   {
      @Override
      public SpaceData6D getSpaceData6D()
      {
         return SpaceData6D.VELOCITY;
      }
   },
   LINEAR_ACCELERATION("LinearAcceleration")
   {
      @Override
      public SpaceData6D getSpaceData6D()
      {
         return SpaceData6D.ACCELERATION;
      }
   },
   ANGULAR_ACCELERATION("AngularAcceleration")
   {
      @Override
      public SpaceData6D getSpaceData6D()
      {
         return SpaceData6D.ACCELERATION;
      }
   },
   LINEAR_FORCE("LinearForce")
   {
      @Override
      public SpaceData6D getSpaceData6D()
      {
         return SpaceData6D.FORCE;
      }
   },
   ANGULAR_TORQUE("AngularTorque")
   {
      @Override
      public SpaceData6D getSpaceData6D()
      {
         return SpaceData6D.FORCE;
      }
   };

   private final String name;

   private SpaceData3D(String name)
   {
      this.name = name;
   }

   public String getName()
   {
      return name;
   }

   public abstract SpaceData6D getSpaceData6D();

   @Override
   public String toString()
   {
      return name;
   }
}