package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Arrays;
import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * This message setup the hand controller to activate a compliance module if at least one of the fields is set and deactivate if the message is empty.
 * To compliance module works only when the last hand command sent with the HandPosePacket is a hand pose (not joint angles).
 * Once activated, an integrator is used on the error in force/torque measured to keep adjusting the desired hand pose until the desired
 * force/torque are achieved or until the maximum correction is reached (set to 5cm for translation and 0.2rad for the orientation).
 * As it uses the measurements from wrist force sensors, a calibration of these is preferred prior to activation of compliance.
 */

public class HandComplianceControlParametersMessage extends Packet<HandComplianceControlParametersMessage>
{
   
   public RobotSide robotSide;

   /**
    * enableLinearCompliance allows to activate/deactivate the compliance in translation for each individual axes (X, Y, and Z).
    * The axes are in the hand control frame attached to the hand:
    *  - X refers to the axis perpendicular to the hand palm (e.g. forward/backward),
    *  - Y refers to the grasping axis (e.g. left/right),
    *  - Z refers to the axis orthogonal to the two other axes (e.g. up/down).
    * If the field is null, the linear compliance will be deactivated.
    */
   public boolean[] enableLinearCompliance;

   /**
    * enableAngularCompliance allows to activate/deactivate the compliance in orientation for each individual axes (X, Y, and Z).
    * The axes are in the hand control frame attached to the hand:
    *  - X refers to the axis perpendicular to the hand palm,
    *  - Y refers to the grasping axis,
    *  - Z refers to the axis orthogonal to the two other axes.
    * If the field is null, the angular compliance will be deactivated.
    */
   public boolean[] enableAngularCompliance;

   /**
    * desiredForce allows to set the desired force to be achieved on the hand for each individual axes (X, Y, and Z).
    * The axes are in the hand control frame attached to the hand:
    *  - X refers to the axis perpendicular to the hand palm (e.g. forward/backward),
    *  - Y refers to the grasping axis (e.g. left/right),
    *  - Z refers to the axis orthogonal to the two other axes (e.g. up/down).
    * If the field is null, the desired force will be set to zero.
    */
   public Vector3D32 desiredForce;

   /**
    * desiredTorque allows to set the desired torque to be achieved on the hand for each individual axes (X, Y, and Z).
    * The axes are in the hand control frame attached to the hand:
    *  - X refers to the axis perpendicular to the hand palm,
    *  - Y refers to the grasping axis,
    *  - Z refers to the axis orthogonal to the two other axes.
    * If the field is null, the desired torque will be set to zero.
    */
   public Vector3D32 desiredTorque;

   /**
    * wrenchDeadzones set the deadzones that are used on the force and torque measurements, respectively.
    * For instance, if wrenchDeadzones = {5.0, 0.5}, the controller will perceive only forces that are outside the range [-5N, 5N],
    * and torques that are outside the range [-0.5N.m, 0.5N.m].
    * As results, the compliance control will start adjusting the desired hand pose only for measured forces/torques greater
    * than the specified deadzones.
    * If the field is null, the deadzone won't be changed.
    * We have found that wrenchDeadzones = {10.0, 0.5} does not affect much the position control accuracy of the hand but still gives good compliance.
    */
   public float[] wrenchDeadzones;

   public HandComplianceControlParametersMessage()
   {
   }

   public HandComplianceControlParametersMessage(Random random)
   {
      robotSide = RandomNumbers.nextEnum(random, RobotSide.class);
      enableLinearCompliance = new boolean[3];
      for(int i = 0; i < 3; i++)
      {
         enableLinearCompliance[i] = random.nextBoolean();
      }

      enableAngularCompliance = new boolean[3];
      for(int i = 0; i < 3; i++)
      {
         enableAngularCompliance[i] = random.nextBoolean();
      }

      desiredForce = RandomGeometry.nextVector3D32(random);
      desiredTorque = RandomGeometry.nextVector3D32(random);

      wrenchDeadzones = new float[2];
      wrenchDeadzones[0] = random.nextFloat();
      wrenchDeadzones[1] = random.nextFloat();
   }

   // Send a packet with only robotSide to deactivate the compliant control.
   public HandComplianceControlParametersMessage(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public void setWrenchDeadzone(double forceDeadzone, double torqueDeadzone)
   {
      wrenchDeadzones = new float[]{(float) forceDeadzone, (float) torqueDeadzone};
   }

   public void setEnableLinearCompliance(boolean[] enableLinearCompliance)
   {
      this.enableLinearCompliance = enableLinearCompliance;
   }

   public void setEnableAngularCompliance(boolean[] enableAngularCompliance)
   {
      this.enableAngularCompliance = enableAngularCompliance;
   }

   public void setDesiredForce(Vector3D32 desiredForce)
   {
      this.desiredForce = desiredForce;
   }

   public void setDesiredTorque(Vector3D32 desiredTorque)
   {
      this.desiredTorque = desiredTorque;
   }

   public boolean isEmpty()
   {
      return enableLinearCompliance == null && enableAngularCompliance == null && desiredForce == null && desiredTorque == null;
   }

   public float[] getWrenchDeadzones()
   {
      return wrenchDeadzones;
   }

   public boolean[] getEnableLinearCompliance()
   {
      return enableLinearCompliance;
   }

   public boolean[] getEnableAngularCompliance()
   {
      return enableAngularCompliance;
   }

   public Vector3D32 getDesiredForce()
   {
      return desiredForce;
   }

   public Vector3D32 getDesiredTorque()
   {
      return desiredTorque;
   }

   @Override
   public boolean epsilonEquals(HandComplianceControlParametersMessage other, double epsilon)
   {
      if (robotSide != other.robotSide)
         return false;

      if (enableLinearCompliance == null && other.enableLinearCompliance != null)
         return false;
      else if (enableLinearCompliance != null && other.enableLinearCompliance == null)
         return false;
      else if (!Arrays.equals(enableLinearCompliance, other.enableLinearCompliance))
         return false;

      if (enableAngularCompliance == null && other.enableAngularCompliance != null)
         return false;
      else if (enableAngularCompliance != null && other.enableAngularCompliance == null)
         return false;
      else if (!Arrays.equals(enableAngularCompliance, other.enableAngularCompliance))
         return false;

      if (desiredForce == null && other.desiredForce != null)
         return false;
      else if (desiredForce != null && other.desiredForce == null)
         return false;
      else if (!desiredForce.epsilonEquals(other.desiredForce, (float) epsilon))
         return false;

      if (desiredTorque == null && other.desiredTorque != null)
         return false;
      else if (desiredTorque != null && other.desiredTorque == null)
         return false;
      else if (!desiredTorque.epsilonEquals(other.desiredTorque, (float) epsilon))
         return false;

      if (wrenchDeadzones == null && other.wrenchDeadzones != null)
         return false;
      else if (wrenchDeadzones != null && other.wrenchDeadzones == null)
         return false;
      else if (Math.abs(wrenchDeadzones[0] - other.wrenchDeadzones[0]) > epsilon)
         return false;
      else if (Math.abs(wrenchDeadzones[1] - other.wrenchDeadzones[1]) > epsilon)
         return false;

      return true;
   }

   @Override
   public String validateMessage()
   {
      if (robotSide == null)
      {
         return "The robotSide is missing.";
      }
      return null;
   }
}
