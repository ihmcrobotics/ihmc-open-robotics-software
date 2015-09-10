package us.ihmc.communication.packets.manipulation;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.random.RandomTools;

import javax.vecmath.Vector3f;
import java.util.Arrays;
import java.util.Random;

@ClassDocumentation("This message setup the hand controller to activate a compliance module if at least one of the fields is set and deactivate if the message is empty.\n"
                                  + "To compliance module works only when the last hand command sent with the HandPosePacket is a hand pose (not joint angles).\n"
                                  + "Once activated, an integrator is used on the error in force/torque measured to keep adjusting the desired hand pose until the desired\n"
                                  + "force/torque are achieved or until the maximum correction is reached (set to 5cm for translation and 0.2rad for the orientation).\n"
                                  + "As it uses the measurements from wrist force sensors, a calibration of these is preferred prior to activation of compliance.")
public class HandComplianceControlParametersPacket extends IHMCRosApiPacket<HandComplianceControlParametersPacket>
{
   
   public RobotSide robotSide;

   @FieldDocumentation("enableLinearCompliance allows to activate/deactivate the compliance in translation for each individual axes (X, Y, and Z).\n"
                                     + "The axes are in the hand control frame attached to the hand:\n"
                                     + " - X refers to the axis perpendicular to the hand palm (e.g. forward/backward),\n"
                                     + " - Y refers to the grasping axis (e.g. left/right),\n"
                                     + " - Z refers to the axis orthogonal to the two other axes (e.g. up/down).\n"
                                     + "If the field is null, the linear compliance will be deactivated.")
   public boolean[] enableLinearCompliance;
   @FieldDocumentation("enableAngularCompliance allows to activate/deactivate the compliance in orientation for each individual axes (X, Y, and Z).\n"
                                     + "The axes are in the hand control frame attached to the hand:\n"
                                     + " - X refers to the axis perpendicular to the hand palm,\n"
                                     + " - Y refers to the grasping axis,\n"
                                     + " - Z refers to the axis orthogonal to the two other axes.\n"
                                     + "If the field is null, the angular compliance will be deactivated.")
   public boolean[] enableAngularCompliance;


   @FieldDocumentation("desiredForce allows to set the desired force to be achieved on the hand for each individual axes (X, Y, and Z).\n"
                                     + "The axes are in the hand control frame attached to the hand:\n"
                                     + " - X refers to the axis perpendicular to the hand palm (e.g. forward/backward),\n"
                                     + " - Y refers to the grasping axis (e.g. left/right),\n"
                                     + " - Z refers to the axis orthogonal to the two other axes (e.g. up/down).\n"
                                     + "If the field is null, the desired force will be set to zero.")
   public Vector3f desiredForce;
   @FieldDocumentation("desiredTorque allows to set the desired torque to be achieved on the hand for each individual axes (X, Y, and Z).\n"
                                     + "The axes are in the hand control frame attached to the hand:\n"
                                     + " - X refers to the axis perpendicular to the hand palm,\n"
                                     + " - Y refers to the grasping axis,\n"
                                     + " - Z refers to the axis orthogonal to the two other axes.\n"
                                     + "If the field is null, the desired torque will be set to zero.")
   public Vector3f desiredTorque;

   @FieldDocumentation("wrenchDeadzones set the deadzones that are used on the force and torque measurements, respectively.\n"
                                     + "For instance, if wrenchDeadzones = {5.0, 0.5}, the controller will perceive only forces that are outside the range [-5N, 5N],\n"
                                     + "and torques that are outside the range [-0.5N.m, 0.5N.m].\n"
                                     + "As results, the compliance control will start adjusting the desired hand pose only for measured forces/torques greater\n"
                                     + "than the specified deadzones.\n"
                                     + "If the field is null, the deadzone won't be changed.\n"
                                     + "We have found that wrenchDeadzones = {10.0, 0.5} does not affect much the position control accuracy of the hand but still gives good compliance.")
   public float[] wrenchDeadzones;

   public HandComplianceControlParametersPacket()
   {
   }

   public HandComplianceControlParametersPacket(Random random)
   {
      robotSide = RandomTools.generateRandomEnum(random, RobotSide.class);
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

      desiredForce = RandomTools.generateRandomVector3f(random);
      desiredTorque = RandomTools.generateRandomVector3f(random);

      wrenchDeadzones = new float[2];
      wrenchDeadzones[0] = random.nextFloat();
      wrenchDeadzones[1] = random.nextFloat();
   }

   // Send a packet with only robotSide to deactivate the compliant control.
   public HandComplianceControlParametersPacket(RobotSide robotSide)
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

   public void setDesiredForce(Vector3f desiredForce)
   {
      this.desiredForce = desiredForce;
   }

   public void setDesiredTorque(Vector3f desiredTorque)
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

   public Vector3f getDesiredForce()
   {
      return desiredForce;
   }

   public Vector3f getDesiredTorque()
   {
      return desiredTorque;
   }

   @Override
   public boolean epsilonEquals(HandComplianceControlParametersPacket other, double epsilon)
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
}
