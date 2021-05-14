package us.ihmc.commonWalkingControlModules.controllerCore.data;

public enum SpaceData6D
{
   POSE("Pose")
   {
      @Override
      public SpaceData3D getAngular()
      {
         return SpaceData3D.ORIENTATION;
      }

      @Override
      public SpaceData3D getLinear()
      {
         return SpaceData3D.POSITION;
      }
   },
   VELOCITY("Velocity")
   {
      @Override
      public SpaceData3D getAngular()
      {
         return SpaceData3D.ANGULAR_VELOCITY;
      }

      @Override
      public SpaceData3D getLinear()
      {
         return SpaceData3D.LINEAR_VELOCITY;
      }
   },
   ACCELERATION("Acceleration")
   {
      @Override
      public SpaceData3D getAngular()
      {
         return SpaceData3D.ANGULAR_ACCELERATION;
      }

      @Override
      public SpaceData3D getLinear()
      {
         return SpaceData3D.LINEAR_ACCELERATION;
      }
   },
   FORCE("Force")
   {
      @Override
      public SpaceData3D getAngular()
      {
         return SpaceData3D.ANGULAR_TORQUE;
      }

      @Override
      public SpaceData3D getLinear()
      {
         return SpaceData3D.LINEAR_FORCE;
      }
   };

   private final String name;

   private SpaceData6D(String name)
   {
      this.name = name;
   }

   public String getName()
   {
      return name;
   }

   public abstract SpaceData3D getAngular();

   public abstract SpaceData3D getLinear();

   @Override
   public String toString()
   {
      return name;
   }
}
