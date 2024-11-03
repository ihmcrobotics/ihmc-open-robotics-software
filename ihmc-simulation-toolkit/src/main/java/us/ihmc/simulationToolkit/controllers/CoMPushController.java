package us.ihmc.simulationToolkit.controllers;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import javax.swing.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.LinkedList;

public class CoMPushController implements RobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry;

   private final YoDouble pushDuration;
   private final YoDouble pushForceMagnitude;
   private final YoDouble pushTorqueMagnitude;

   private final YoFrameVector3D pushForceDirection;
   private final YoFrameVector3D pushTorqueDirection;

   private final YoFrameVector3D pushForce;
   private final YoFrameVector3D pushTorque;
   private final YoDouble pushTimeSwitch;
   private final YoInteger pushNumber;
   private final YoBoolean isBeingPushed;
   private final YoDouble pushDelay;

   private final YoDouble yoTime;

   private StateTransitionCondition pushCondition = null;
   private final ExternalForcePoint forcePoint;
   private final Vector3D forceVector = new Vector3D();
   private final Vector3D torqueVector = new Vector3D();

   private final YoGraphicVector forceVisualizer;

   private final LinkedList<DelayedPush> delayedPushs = new LinkedList<>();
   private final Joint pushJoint;

   private final CenterOfMassCalculator centerOfMassCalculator;

   public CoMPushController(FloatingRootJointRobot pushableRobot, FullHumanoidRobotModel fullRobotModel)
   {
      this(pushableRobot, fullRobotModel, 0.005);
   }

   public CoMPushController(FloatingRootJointRobot pushableRobot,  FullHumanoidRobotModel fullRobotModel, double visualScale)
   {
      String jointNameToApplyForce = fullRobotModel.getChest().getParentJoint().getName();

      yoTime = pushableRobot.getYoTime();
      registry = new YoRegistry(jointNameToApplyForce + "_" + getClass().getSimpleName());
      forcePoint = new ExternalForcePoint(jointNameToApplyForce + "_externalForcePoint", new Vector3D(), pushableRobot);


      centerOfMassCalculator = new CenterOfMassCalculator(fullRobotModel.getRootBody(), worldFrame);

      pushDuration = new YoDouble(jointNameToApplyForce + "_pushDuration", registry);
      pushForceMagnitude = new YoDouble(jointNameToApplyForce + "_pushForceMagnitude", registry);
      pushTorqueMagnitude = new YoDouble(jointNameToApplyForce + "_pushTorqueMagnitude", registry);
      pushForceDirection = new YoFrameVector3D(jointNameToApplyForce + "_pushForceDirection", worldFrame, registry);
      pushTorqueDirection = new YoFrameVector3D(jointNameToApplyForce + "_pushTorqueDirection", worldFrame, registry);
      pushForce = new YoFrameVector3D(jointNameToApplyForce + "_pushForce", worldFrame, registry);
      pushTorque = new YoFrameVector3D(jointNameToApplyForce + "_pushTorque", worldFrame, registry);

      pushTimeSwitch = new YoDouble(jointNameToApplyForce + "_pushTimeSwitch", registry);
      pushNumber = new YoInteger(jointNameToApplyForce + "_pushNumber", registry);
      isBeingPushed = new YoBoolean(jointNameToApplyForce + "_isBeingPushed", registry);
      pushDelay = new YoDouble(jointNameToApplyForce + "_pushDelay", registry);

      pushJoint = pushableRobot.getJoint((jointNameToApplyForce));
      pushJoint.addExternalForcePoint(forcePoint);
      pushableRobot.setController(this);

      pushTimeSwitch.set(Double.NEGATIVE_INFINITY);
      pushForceMagnitude.set(0.0);

      forceVisualizer = new YoGraphicVector(jointNameToApplyForce
            + "_pushForce", forcePoint.getYoPosition(), forcePoint.getYoForce(), visualScale, YoAppearance.DarkBlue());
   }

   public YoGraphic getForceVisualizer()
   {
      return forceVisualizer;
   }

   public int getPushNumber()
   {
      return pushNumber.getIntegerValue();
   }

   public void setPushDuration(double duration)
   {
      pushDuration.set(duration);
   }

   public void setPushForceMagnitude(double magnitude)
   {
      pushForceMagnitude.set(magnitude);
   }

   public void setPushTorqueMagnitude(double magnitude)
   {
      pushTorqueMagnitude.set(magnitude);
   }

   public void setPushForceDirection(Vector3DReadOnly direction)
   {
      pushForceDirection.set(direction);
   }

   public void setPushTorqueDirection(Vector3DReadOnly direction)
   {
      pushTorqueDirection.set(direction);
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

   public void applyForce(Vector3DReadOnly direction, double magnitude, double duration)
   {
      applyForceDelayed(null, 0.0, direction, magnitude, duration);
   }

   public void applyForceDelayed(StateTransitionCondition pushCondition, double timeDelay, Vector3DReadOnly direction, double magnitude, double duration)
   {
      this.pushCondition = pushCondition;
      setPushDuration(duration);
      setPushForceDirection(direction);
      setPushForceMagnitude(magnitude);
      setPushDelay(timeDelay);
      applyForce();
   }

   public void queueForceDelayed(StateTransitionCondition pushCondition, double timeDelay, Vector3DReadOnly direction, double magnitude, double duration)
   {
      if (delayedPushs.isEmpty())
         applyForceDelayed(pushCondition, timeDelay, direction, magnitude, duration);
      else
         delayedPushs.add(new DelayedPush(pushCondition, timeDelay, direction, magnitude, duration));
   }

   private void applyForce()
   {
      double forceLength = pushForceDirection.length();
      double torqueLength = pushTorqueDirection.length();

      if (forceLength > 1e-5 || torqueLength > 1e-5)
      {
         if (pushCondition == null)
            pushTimeSwitch.set(yoTime.getDoubleValue());
      }
      else
      {
         pushTimeSwitch.set(Double.NEGATIVE_INFINITY);
      }

      if (forceLength > 1e-5)
      {
         pushForce.set(pushForceDirection);
         pushForce.normalize();
         pushForce.scale(pushForceMagnitude.getDoubleValue());
      }
      else
      {
         pushForce.setToZero();
      }

      forceLength = pushTorqueDirection.length();
      if (forceLength > 1e-5)
      {
         pushTorque.set(pushTorqueDirection);
         pushTorque.normalize();
         pushTorque.scale(pushTorqueMagnitude.getDoubleValue());
      }
      else
      {
         pushTorque.setToZero();
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
         if (pushCondition.testCondition(yoTime.getDoubleValue()))
         {
            pushTimeSwitch.set(yoTime.getDoubleValue() + pushDelay.getDoubleValue());
            pushCondition = null;
         }
      }

      if (yoTime.getDoubleValue() <= pushTimeSwitch.getDoubleValue() + pushDuration.getDoubleValue()
            && yoTime.getDoubleValue() >= pushTimeSwitch.getDoubleValue())
      {
         isBeingPushed.set(true);
         forceVector.set(pushForce);
         torqueVector.set(pushTorque);
         pushNumber.decrement();
      }
      else
      {
         if (isBeingPushed.getValue() && !delayedPushs.isEmpty())
         {
            DelayedPush delayedPush = delayedPushs.pollFirst();
            applyForceDelayed(delayedPush.pushCondition, delayedPush.timeDelay, delayedPush.direction, delayedPush.magnitude, delayedPush.duration);
         }

         isBeingPushed.set(false);
         forceVector.set(0.0, 0.0, 0.0);
         torqueVector.setToZero();
      }

      forcePoint.setOffsetWorld(centerOfMassCalculator.getCenterOfMass());

      forcePoint.setForce(forceVector);
      forcePoint.setMoment(torqueVector);
   }

   @Override
   public YoRegistry getYoRegistry()
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

   private static class DelayedPush
   {
      private StateTransitionCondition pushCondition;
      private double timeDelay;
      private Vector3DReadOnly direction;
      private double magnitude;
      private double duration;

      public DelayedPush(StateTransitionCondition pushCondition, double timeDelay, Vector3DReadOnly direction, double magnitude, double duration)
      {
         this.pushCondition = pushCondition;
         this.timeDelay = timeDelay;
         this.direction = direction;
         this.magnitude = magnitude;
         this.duration = duration;
      }
   }
}