package us.ihmc.commonWalkingControlModules.controllerCore;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
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
      ERROR("Error");
   
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
      ANGULAR_ACCELERATION("AngularAcceleration");
   
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
    * Retrieves if possible the position data about a specific end-effector.
    * <p>
    * If no feedback controller has used the requested data, it will not be available in which case the method returns {@code false}.
    * </p>
    * 
    * @param endEffector the end-effector for which the data is requested.
    * @param positionDataToPack the {@link FramePoint} in which the position data is stored. Data is expressed in {@link ReferenceFrame#getWorldFrame()}. Modified. 
    * @param type whether the current or desired position is requested, the other values in {@link Type} are not applicable.
    * @return whether the data is available or not.
    */
   boolean getPositionData(RigidBody endEffector, FramePoint positionDataToPack, Type type);

   /**
    * Retrieves if possible the orientation data about a specific end-effector.
    * <p>
    * If no feedback controller has used the requested data, it will not be available in which case the method returns {@code false}.
    * </p>
    * 
    * @param endEffector the end-effector for which the data is requested.
    * @param orientationDataToPack the {@link FrameOrientation} in which the orientation data is stored. Data is expressed in {@link ReferenceFrame#getWorldFrame()}. Modified.
    * @param type whether the current or desired orientation is requested, the other values in {@link Type} are not applicable.
    * @return whether the data is available or not.
    */
   boolean getOrientationData(RigidBody endEffector, FrameOrientation orientationDataToPack, Type type);

   /**
    * Retrieves if possible the vector data about a specific end-effector.
    * <p>
    * If no feedback controller has used the requested data, it will not be available in which case the method returns {@code false}.
    * </p>
    * 
    * @param endEffector the end-effector for which the data is requested.
    * @param vectorDataToPack the {@link FrameVector} in which the vector data is stored. Data is expressed in {@link ReferenceFrame#getWorldFrame()}. Modified.
    * @param type specifies the data type requested. Look up the options available in {@link Type}.
    * @param space specifies the physical quantity requested. Look up the options available in {@link Space}.
    * @return whether the data is available or not.
    */
   boolean getVectorData(RigidBody endEffector, FrameVector vectorDataToPack, Type type, Space space);
}
