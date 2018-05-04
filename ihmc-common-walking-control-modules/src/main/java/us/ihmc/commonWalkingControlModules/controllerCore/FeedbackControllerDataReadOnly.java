package us.ihmc.commonWalkingControlModules.controllerCore;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public interface FeedbackControllerDataReadOnly
{
   public enum Type
   {
      DESIRED("Desired"),
      CURRENT("Current"),
      FEEDFORWARD("FeedForward"),
      FEEDBACK("Feedback"),
      ACHIEVED("Achieved"),
      ERROR("Error"),
      ERROR_CUMULATED("ErrorCumulated"),
      ERROR_INTEGRATED("ErrorIntegrated");

      private final String name;

      private Type(String name)
      {
         this.name = name;
      }

      public String getName()
      {
         return name;
      }
   }

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

   /**
    * Retrieves if possible the position data about the center of mass.
    * <p>
    * If no feedback controller has used the requested data, it will not be available in which case
    * the method returns {@code false}.
    * </p>
    * 
    * @param positionDataToPack the {@link FramePoint3D} in which the position data is stored. Data is
    *           expressed in {@link ReferenceFrame#getWorldFrame()}. Modified.
    * @param type whether the current or desired position is requested, the other values in
    *           {@link Type} are not applicable.
    * @return whether the data is available or not.
    */
   boolean getCenterOfMassPositionData(FramePoint3D positionDataToPack, Type type);

   /**
    * Retrieves if possible the vector data about the center of mass.
    * <p>
    * If no feedback controller has used the requested data, it will not be available in which case
    * the method returns {@code false}.
    * </p>
    * 
    * @param vectorDataToPack the {@link FrameVector3D} in which the vector data is stored. Data is
    *           expressed in {@link ReferenceFrame#getWorldFrame()}. Modified.
    * @param type specifies the data type requested. Look up the options available in {@link Type}.
    * @param space specifies the physical quantity requested. Look up the options available in
    *           {@link Space}.
    * @return whether the data is available or not.
    */
   boolean getCenterOfMassVectorData(FrameVector3D vectorDataToPack, Type type, Space space);

   /**
    * Retrieves if possible the position data about a specific end-effector.
    * <p>
    * If no feedback controller has used the requested data, it will not be available in which case
    * the method returns {@code false}.
    * </p>
    * 
    * @param endEffector the end-effector for which the data is requested.
    * @param positionDataToPack the {@link FramePoint3D} in which the position data is stored. Data is
    *           expressed in {@link ReferenceFrame#getWorldFrame()}. Modified.
    * @param type whether the current or desired position is requested, the other values in
    *           {@link Type} are not applicable.
    * @return whether the data is available or not.
    */
   boolean getPositionData(RigidBody endEffector, FramePoint3D positionDataToPack, Type type);

   /**
    * Retrieves if possible the orientation data about a specific end-effector.
    * <p>
    * If no feedback controller has used the requested data, it will not be available in which case
    * the method returns {@code false}.
    * </p>
    * 
    * @param endEffector the end-effector for which the data is requested.
    * @param orientationDataToPack the {@link FrameQuaternion} in which the orientation data is
    *           stored. Data is expressed in {@link ReferenceFrame#getWorldFrame()}. Modified.
    * @param type whether the current or desired orientation is requested, the other values in
    *           {@link Type} are not applicable.
    * @return whether the data is available or not.
    */
   boolean getOrientationData(RigidBody endEffector, FrameQuaternion orientationDataToPack, Type type);

   /**
    * Retrieves if possible the vector data about a specific end-effector.
    * <p>
    * If no feedback controller has used the requested data, it will not be available in which case
    * the method returns {@code false}.
    * </p>
    * 
    * @param endEffector the end-effector for which the data is requested.
    * @param vectorDataToPack the {@link FrameVector3D} in which the vector data is stored. Data is
    *           expressed in {@link ReferenceFrame#getWorldFrame()}. Modified.
    * @param type specifies the data type requested. Look up the options available in {@link Type}.
    * @param space specifies the physical quantity requested. Look up the options available in
    *           {@link Space}.
    * @return whether the data is available or not.
    */
   boolean getVectorData(RigidBody endEffector, FrameVector3D vectorDataToPack, Type type, Space space);
}
