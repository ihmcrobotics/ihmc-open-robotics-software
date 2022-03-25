package us.ihmc.simulationToolkit.controllers;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.LinkedList;

import javax.swing.JButton;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.robot.ExternalWrenchPointDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicArrow3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.trackers.ExternalWrenchPoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class PushRobotControllerSCS2 implements Controller
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry;
   private final YoDouble pushDuration;
   private final YoDouble pushForceMagnitude;
   private final YoFrameVector3D pushDirection;
   private final YoFrameVector3D pushForce;
   private final YoDouble pushTimeSwitch;
   private final YoInteger pushNumber;
   private final YoBoolean isBeingPushed;
   private final YoDouble pushDelay;

   private final DoubleProvider yoTime;

   private StateTransitionCondition pushCondition = null;
   private final ExternalWrenchPoint forcePoint;
   private final Vector3D forceVector = new Vector3D();

   private final LinkedList<DelayedPush> delayedPushs = new LinkedList<>();

   private final String jointNameToApplyForce;

   private final double visualScale;

   public PushRobotControllerSCS2(DoubleProvider time, Robot pushableRobot, FullHumanoidRobotModel fullRobotModel)
   {
      this(time, pushableRobot, fullRobotModel.getChest().getParentJoint().getName(), new Vector3D(0, 0, 0.3), 0.005);
   }

   public PushRobotControllerSCS2(DoubleProvider time, Robot pushableRobot, String jointNameToApplyForce, Vector3DReadOnly forcePointOffset)
   {
      this(time, pushableRobot, jointNameToApplyForce, forcePointOffset, 0.005);
   }

   public PushRobotControllerSCS2(DoubleProvider time, Robot pushableRobot, String jointNameToApplyForce, Vector3DReadOnly forcePointOffset, double visualScale)
   {
      yoTime = time;
      this.jointNameToApplyForce = jointNameToApplyForce;
      this.visualScale = visualScale;
      registry = new YoRegistry(jointNameToApplyForce + "_" + getClass().getSimpleName());
      forcePoint = pushableRobot.getJoint(jointNameToApplyForce).getAuxialiryData()
                                .addExternalWrenchPoint(new ExternalWrenchPointDefinition(jointNameToApplyForce + "_externalForcePoint", forcePointOffset));

      pushDuration = new YoDouble(jointNameToApplyForce + "_pushDuration", registry);
      pushForceMagnitude = new YoDouble(jointNameToApplyForce + "_pushMagnitude", registry);
      pushDirection = new YoFrameVector3D(jointNameToApplyForce + "_pushDirection", worldFrame, registry);
      pushForce = new YoFrameVector3D(jointNameToApplyForce + "_pushForce", worldFrame, registry);
      pushTimeSwitch = new YoDouble(jointNameToApplyForce + "_pushTimeSwitch", registry);
      pushNumber = new YoInteger(jointNameToApplyForce + "_pushNumber", registry);
      isBeingPushed = new YoBoolean(jointNameToApplyForce + "_isBeingPushed", registry);
      pushDelay = new YoDouble(jointNameToApplyForce + "_pushDelay", registry);

      pushableRobot.getControllerManager().addController(this);

      pushTimeSwitch.set(Double.NEGATIVE_INFINITY);
      pushForceMagnitude.set(0.0);
   }

   public YoGraphicDefinition getForceVizDefinition()
   {
      YoGraphicArrow3DDefinition definition = new YoGraphicArrow3DDefinition();
      definition.setName(jointNameToApplyForce + "_pushForce");
      YoFramePoint3D position = forcePoint.getPose().getPosition();
      definition.setOrigin(new YoTuple3DDefinition(position.getYoX().getFullNameString(),
                                                   position.getYoY().getFullNameString(),
                                                   position.getYoZ().getFullNameString()));
      YoFrameVector3D force = forcePoint.getWrench().getLinearPart();
      definition.setDirection(new YoTuple3DDefinition(force.getYoX().getFullNameString(),
                                                      force.getYoY().getFullNameString(),
                                                      force.getYoZ().getFullNameString(),
                                                      force.getReferenceFrame().getNameId()));
      definition.setVisible(true);
      definition.setScaleLength(true);
      definition.setBodyLength(visualScale * 0.90);
      definition.setHeadLength(visualScale * 0.10);
      definition.setBodyRadius(0.015);
      definition.setHeadRadius(0.0375);
      return definition;
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

   public void setPushForceDirection(Vector3DReadOnly direction)
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
      double length = pushDirection.length();
      if (length > 1e-5)
      {
         pushForce.set(pushDirection);
         pushForce.normalize();
         pushForce.scale(pushForceMagnitude.getValue());
         if (pushCondition == null)
         {
            pushTimeSwitch.set(yoTime.getValue());
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
         if (pushCondition.testCondition(yoTime.getValue()))
         {
            pushTimeSwitch.set(yoTime.getValue() + pushDelay.getValue());
            pushCondition = null;
         }
      }

      if (yoTime.getValue() <= pushTimeSwitch.getValue() + pushDuration.getValue() && yoTime.getValue() >= pushTimeSwitch.getValue())
      {
         isBeingPushed.set(true);
         forceVector.set(pushForce);
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
      }

      forcePoint.getWrench().getLinearPart().setMatchingFrame(forcePoint.getFrame().getRootFrame(), forceVector);
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