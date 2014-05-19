package us.ihmc.darpaRoboticsChallenge.controllers;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.IntegerYoVariable;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class DRCPushRobotController implements RobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable pushDuration = new DoubleYoVariable("pushDuration", registry);
   private final DoubleYoVariable pushForceMagnitude = new DoubleYoVariable("pushMagnitude", registry);
   private final YoFrameVector pushDirection = new YoFrameVector("pushDirection", worldFrame, registry);
   private final YoFrameVector pushForce = new YoFrameVector("pushForce", worldFrame, registry);
   private final DoubleYoVariable pushTimeSwitch = new DoubleYoVariable("pushTimeSwitch", registry);
   private final IntegerYoVariable pushNumber = new IntegerYoVariable("pushNumber", registry);
   private final BooleanYoVariable isBeingPushed = new BooleanYoVariable("isBeingPushed", registry);

   private final DoubleYoVariable yoTime;

   private final ExternalForcePoint forcePoint;
   private final Vector3d forceVector = new Vector3d();

   public DRCPushRobotController(SDFRobot pushableRobot, FullRobotModel fullRobotModel)
   {
      yoTime = pushableRobot.getYoTime();
      forcePoint = new ExternalForcePoint("pushTheRobot", new Vector3d(0, 0, 0.3), pushableRobot);
      String upperSpineJointName = fullRobotModel.getChest().getParentJoint().getName();
      pushableRobot.getJoint(upperSpineJointName).addExternalForcePoint(forcePoint);
      pushableRobot.setController(this);

      pushTimeSwitch.set(Double.NEGATIVE_INFINITY);
      pushForceMagnitude.set(0.0);
   }

   public void setPushDuration(double duration)
   {
      pushDuration.set(duration);
   }

   public void setPushForceMagnitude(double magnitude)
   {
      pushForceMagnitude.set(magnitude);
   }

   public void setPushForceDirection(Vector3d direction)
   {
      pushDirection.set(direction);
   }

   public void addPushButtonToSCS(final SimulationConstructionSet scs)
   {
      if (scs != null)
      {
         JButton button = new JButton("PushRobot");
         button.setToolTipText("Click to push the robot as defined in the variable 'PushVector'");

         ActionListener listener = new ActionListener()
         {
            @Override
            public void actionPerformed(ActionEvent e)
            {
               applyForce();
            }
         };

         button.addActionListener(listener);
         scs.addButton(button);
      }
   }

   public void applyForce(Vector3d direction, double magnitude, double duration)
   {
      setPushDuration(duration);
      setPushForceDirection(direction);
      setPushForceMagnitude(magnitude);
      applyForce();
   }

   public void applyForce()
   {
      System.out.println("Pushing the robot now");

      double length = pushDirection.length();
      if (length > 1e-5)
      {
         pushForce.set(pushDirection);
         pushForce.normalize();
         pushForce.scale(pushForceMagnitude.getDoubleValue());
      }
      else
      {
         pushForce.setToZero();
      }

      pushTimeSwitch.set(yoTime.getDoubleValue());
      pushNumber.increment();
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void doControl()
   {
      if (yoTime.getDoubleValue() <= pushTimeSwitch.getDoubleValue() + pushDuration.getDoubleValue())
      {
         isBeingPushed.set(true);
         pushForce.get(forceVector);
      }
      else
      {
         isBeingPushed.set(false);
         forceVector.set(0.0, 0.0, 0.0);
      }

      forcePoint.setForce(forceVector);
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

   @Override
   public String getDescription()
   {
      return registry.getName();
   }
}