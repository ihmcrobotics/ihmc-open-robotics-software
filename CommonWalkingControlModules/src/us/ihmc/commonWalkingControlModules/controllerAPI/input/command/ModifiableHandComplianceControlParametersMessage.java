package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import java.util.Arrays;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandComplianceControlParametersMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class ModifiableHandComplianceControlParametersMessage
{
   private RobotSide robotSide;
   private boolean enable = false;

   private final boolean[] enableLinearCompliance = new boolean[] {false, false, false};
   private final boolean[] enableAngularCompliance = new boolean[] {false, false, false};

   private final Vector3d desiredForce = new Vector3d();
   private final Vector3d desiredTorque = new Vector3d();

   private double forceDeadZone = Double.NaN;
   private double torqueDeadZone = Double.NaN;

   public ModifiableHandComplianceControlParametersMessage()
   {
   }

   public void set(HandComplianceControlParametersMessage handComplianceControlParametersMessage)
   {
      robotSide = handComplianceControlParametersMessage.getRobotSide();

      if (handComplianceControlParametersMessage.isEmpty())
      {
         enable = false;
         Arrays.fill(enableLinearCompliance, false);
         Arrays.fill(enableAngularCompliance, false);
         forceDeadZone = Double.NaN;
         torqueDeadZone = Double.NaN;
      }
      else
      {
         enable = true;
         boolean[] newEnableLinearCompliance = handComplianceControlParametersMessage.getEnableLinearCompliance();
         boolean[] newEnableAngularCompliance = handComplianceControlParametersMessage.getEnableAngularCompliance();
         Vector3f newDesiredForce = handComplianceControlParametersMessage.getDesiredForce();
         Vector3f newDesiredTorque = handComplianceControlParametersMessage.getDesiredTorque();
         float[] newWrenchDeadzones = handComplianceControlParametersMessage.getWrenchDeadzones();

         if (newEnableLinearCompliance == null)
            Arrays.fill(enableLinearCompliance, false);
         else
            System.arraycopy(newEnableLinearCompliance, 0, enableLinearCompliance, 0, 3);
         
         if (newEnableAngularCompliance == null)
            Arrays.fill(enableAngularCompliance, false);
         else
            System.arraycopy(newEnableAngularCompliance, 0, enableAngularCompliance, 0, 3);

         if (newDesiredForce == null)
            desiredForce.set(0.0, 0.0, 0.0);
         else
            desiredForce.set(newDesiredForce);

         if (newDesiredTorque == null)
            desiredTorque.set(0.0, 0.0, 0.0);
         else
            desiredTorque.set(newDesiredTorque);

         if (newWrenchDeadzones == null)
         {
            forceDeadZone = Double.NaN;
            torqueDeadZone = Double.NaN;
         }
         else
         {
            forceDeadZone = newWrenchDeadzones[0];
            torqueDeadZone = newWrenchDeadzones[1];
         }
      }
   }

   public void set(ModifiableHandComplianceControlParametersMessage other)
   {
      robotSide = other.robotSide;
      enable = other.enable;
      System.arraycopy(other.enableLinearCompliance, 0, enableLinearCompliance, 0, 3);
      System.arraycopy(other.enableAngularCompliance, 0, enableAngularCompliance, 0, 3);
      desiredForce.set(other.desiredForce);
      desiredTorque.set(other.desiredTorque);
      forceDeadZone = other.forceDeadZone;
      torqueDeadZone = other.torqueDeadZone;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public boolean isEnable()
   {
      return enable;
   }

   public boolean[] getEnableLinearCompliance()
   {
      return enableLinearCompliance;
   }

   public boolean[] getEnableAngularCompliance()
   {
      return enableAngularCompliance;
   }

   public Vector3d getDesiredForce()
   {
      return desiredForce;
   }

   public Vector3d getDesiredTorque()
   {
      return desiredTorque;
   }

   public double getForceDeadZone()
   {
      return forceDeadZone;
   }

   public double getTorqueDeadZone()
   {
      return torqueDeadZone;
   }
}
