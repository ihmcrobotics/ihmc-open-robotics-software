package us.ihmc.commonWalkingControlModules.controllerCore.data;

public enum Space
{
   POSITION("Position"),
   ORIENTATION("Orientation"),
   ROTATION_VECTOR("RotationVector"),
   LINEAR_VELOCITY("LinearVelocity"),
   ANGULAR_VELOCITY("AngularVelocity"),
   LINEAR_ACCELERATION("LinearAcceleration"),
   ANGULAR_ACCELERATION("AngularAcceleration"),
   LINEAR_FORCE("LinearForce"),
   ANGULAR_TORQUE("AngularTorque");

   private final String name;

   private Space(String name)
   {
      this.name = name;
   }

   public String getName()
   {
      return name;
   }

   @Override
   public String toString()
   {
      return name;
   }
}