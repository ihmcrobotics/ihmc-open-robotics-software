package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Arrays;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandComplianceControlParametersMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandComplianceControlParametersCommand
      implements Command<HandComplianceControlParametersCommand, HandComplianceControlParametersMessage>
{
   private RobotSide robotSide;
   private boolean enable = false;

   private final boolean[] enableLinearCompliance = new boolean[] {false, false, false};
   private final boolean[] enableAngularCompliance = new boolean[] {false, false, false};

   private final Vector3D desiredForce = new Vector3D();
   private final Vector3D desiredTorque = new Vector3D();

   private double forceDeadZone = Double.NaN;
   private double torqueDeadZone = Double.NaN;

   public HandComplianceControlParametersCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      robotSide = null;
      enable = false;
      Arrays.fill(enableLinearCompliance, false);
      Arrays.fill(enableAngularCompliance, false);
      desiredForce.set(0.0, 0.0, 0.0);
      desiredTorque.set(0.0, 0.0, 0.0);
      forceDeadZone = Double.NaN;
      torqueDeadZone = Double.NaN;
   }

   @Override
   public void set(HandComplianceControlParametersMessage message)
   {
      robotSide = message.getRobotSide();

      if (message.isEmpty())
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
         boolean[] newEnableLinearCompliance = message.getEnableLinearCompliance();
         boolean[] newEnableAngularCompliance = message.getEnableAngularCompliance();
         Vector3D32 newDesiredForce = message.getDesiredForce();
         Vector3D32 newDesiredTorque = message.getDesiredTorque();
         float[] newWrenchDeadzones = message.getWrenchDeadzones();

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

   @Override
   public void set(HandComplianceControlParametersCommand other)
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

   public Vector3D getDesiredForce()
   {
      return desiredForce;
   }

   public Vector3D getDesiredTorque()
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

   @Override
   public Class<HandComplianceControlParametersMessage> getMessageClass()
   {
      return HandComplianceControlParametersMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null;
   }
}
