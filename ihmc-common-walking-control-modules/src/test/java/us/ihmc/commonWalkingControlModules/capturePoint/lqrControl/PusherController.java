package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoInteger;

import javax.swing.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

public class PusherController implements RobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;
   private final YoDouble pushDuration;
   private final YoDouble pushForceMagnitude;
   private final YoFrameVector3D pushDirection;
   private final YoFrameVector3D pushForce;
   private final YoDouble pushTimeSwitch;
   private final YoInteger pushNumber;
   private final YoBoolean isBeingPushed;
   private final YoDouble pushDelay;
   private final YoBoolean applyPush;
   private final YoDouble yoTime;

   private StateTransitionCondition pushCondition = null;
   private final ExternalForcePoint forcePoint;
   private final Vector3D forceVector = new Vector3D();

   private final YoGraphicVector forceVisualizer;

   public PusherController(Robot pushableRobot, Joint joint, Vector3DReadOnly forcePointOffset, double visualScale)
   {
      String name = pushableRobot.getName() + "_" + joint.getName();
      yoTime = pushableRobot.getYoTime();
      registry = new YoVariableRegistry(name + "_" + getClass().getSimpleName());
      forcePoint = new ExternalForcePoint(name + "_externalForcePoint", forcePointOffset, pushableRobot);

      pushDuration = new YoDouble(name + "_pushDuration", registry);
      pushForceMagnitude = new YoDouble(name + "_pushMagnitude", registry);
      pushDirection = new YoFrameVector3D(name + "_pushDirection", worldFrame, registry);
      pushForce = new YoFrameVector3D(name + "_pushForce", worldFrame, registry);
      pushTimeSwitch = new YoDouble(name + "_pushTimeSwitch", registry);
      pushNumber = new YoInteger(name + "_pushNumber", registry);
      isBeingPushed = new YoBoolean(name + "_isBeingPushed", registry);
      pushDelay = new YoDouble(name + "_pushDelay", registry);
      applyPush = new YoBoolean(name + "_applyPush", registry);
      applyPush.addVariableChangedListener(v -> {
         if (applyPush.getBooleanValue())
         {
            pushCondition = null;
            applyForce();
            applyPush.set(false);
         }
      });

      joint.addExternalForcePoint(forcePoint);
      pushableRobot.setController(this);

      pushTimeSwitch.set(Double.NEGATIVE_INFINITY);
      pushForceMagnitude.set(0.0);

      forceVisualizer = new YoGraphicVector(name + "_pushForce", forcePoint.getYoPosition(), forcePoint.getYoForce(), visualScale, YoAppearance.DarkBlue());
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

   public void setPushForceDirection(Vector3DReadOnly direction)
   {
      pushDirection.set(direction);
   }

   public void setPushDelay(double delay)
   {
      pushDelay.set(delay);
   }

   public void bindSCSPushButton(JButton button)
   {
      ActionListener listener = v -> applyPush.set(true);
      button.addActionListener(listener);
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

   private void applyForce()
   {
      double length = pushDirection.length();
      if (length > 1e-5)
      {
         pushForce.set(pushDirection);
         pushForce.normalize();
         pushForce.scale(pushForceMagnitude.getDoubleValue());
         if (pushCondition == null)
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
         if (pushCondition.testCondition(yoTime.getDoubleValue()))
         {
            pushTimeSwitch.set(yoTime.getDoubleValue() + pushDelay.getDoubleValue());
            pushCondition = null;
         }
      }

      if (yoTime.getDoubleValue() <= pushTimeSwitch.getDoubleValue() + pushDuration.getDoubleValue() && yoTime.getDoubleValue() >= pushTimeSwitch
            .getDoubleValue())
      {
         isBeingPushed.set(true);
         forceVector.set(pushForce);
         pushNumber.decrement();
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