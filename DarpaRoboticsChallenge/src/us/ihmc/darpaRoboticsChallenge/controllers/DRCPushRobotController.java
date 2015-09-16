package us.ihmc.darpaRoboticsChallenge.controllers;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.yoUtilities.graphics.YoGraphic;
import us.ihmc.yoUtilities.graphics.YoGraphicVector;

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
   private final DoubleYoVariable pushDelay = new DoubleYoVariable("pushDelay", registry);

   private final DoubleYoVariable yoTime;

   private StateTransitionCondition pushCondition = null;
   private final ExternalForcePoint forcePoint;
   private final Vector3d forceVector = new Vector3d();
   
   private final YoGraphicVector forceVisualizer;
   
   public DRCPushRobotController(SDFRobot pushableRobot, FullRobotModel fullRobotModel)
   {
      yoTime = pushableRobot.getYoTime();
      forcePoint = new ExternalForcePoint("pushTheRobot", new Vector3d(0, 0, 0.3), pushableRobot);
      String upperSpineJointName = fullRobotModel.getChest().getParentJoint().getName();
      pushableRobot.getJoint(upperSpineJointName).addExternalForcePoint(forcePoint);
      pushableRobot.setController(this);

      pushTimeSwitch.set(Double.NEGATIVE_INFINITY);
      pushForceMagnitude.set(0.0);
      
      forceVisualizer = new YoGraphicVector("pushForce", forcePoint.getYoPosition(), forcePoint.getYoForce(), 0.005, YoAppearance.DarkBlue());
   }
   
   public YoGraphic getForceVisualizer()
   {
      return forceVisualizer;
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
   
   public void setPushDelay(double delay)
   {
      pushDelay.set(delay);
   }

   public void addPushButtonToSCS(final SimulationConstructionSet scs)
   {
      if (scs != null)
      {
         JButton button = new JButton("PushRobot");
         button.setToolTipText("Click to push the robot as defined in the variables 'pushDirection' and 'pushMagnitude'");

         ActionListener listener = new ActionListener()
         {
            @Override
            public void actionPerformed(ActionEvent e)
            {
               pushCondition = null;
               applyForce();
            }
         };

         button.addActionListener(listener);
         scs.addButton(button);
      }
   }

   public void applyForce(Vector3d direction, double magnitude, double duration)
   {
      applyForceDelayed(null, 0.0, direction, magnitude, duration);
   }

   public void applyForceDelayed(StateTransitionCondition pushCondition, double timeDelay, Vector3d direction, double magnitude, double duration)
   {
      this.pushCondition = pushCondition;
      setPushDuration(duration);
      setPushForceDirection(direction);
      setPushForceMagnitude(magnitude);
      setPushDelay(timeDelay);
      applyForce();
   }

   private void applyForce()
   {      
      double length = pushDirection.length();
      if (length > 1e-5)
      {
         pushForce.set(pushDirection);
         pushForce.normalize();
         pushForce.scale(pushForceMagnitude.getDoubleValue());
         if(pushCondition == null)
         {
            pushTimeSwitch.set(yoTime.getDoubleValue());
         }
      }
      else
      {
         pushForce.setToZero();
         pushTimeSwitch.set(Double.NEGATIVE_INFINITY);
      }

      pushNumber.increment();
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void doControl()
   {
      if (pushCondition != null)
      {
         if (pushCondition.checkCondition())
         {
            pushTimeSwitch.set(yoTime.getDoubleValue() + pushDelay.getDoubleValue());
            pushCondition = null;
         }
      }

      if (yoTime.getDoubleValue() <= pushTimeSwitch.getDoubleValue() + pushDuration.getDoubleValue()
            && yoTime.getDoubleValue() >= pushTimeSwitch.getDoubleValue())
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