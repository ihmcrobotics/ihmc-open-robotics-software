package us.ihmc.simulationToolkit.controllers;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class PushRobotController implements RobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;
   private final DoubleYoVariable pushDuration;
   private final DoubleYoVariable pushForceMagnitude;
   private final YoFrameVector pushDirection;
   private final YoFrameVector pushForce;
   private final DoubleYoVariable pushTimeSwitch;
   private final IntegerYoVariable pushNumber;
   private final BooleanYoVariable isBeingPushed;
   private final DoubleYoVariable pushDelay;

   private final DoubleYoVariable yoTime;

   private StateTransitionCondition pushCondition = null;
   private final ExternalForcePoint forcePoint;
   private final Vector3D forceVector = new Vector3D();
   
   private final YoGraphicVector forceVisualizer;
   
   public PushRobotController(FloatingRootJointRobot pushableRobot, FullRobotModel fullRobotModel)
   {
      this(pushableRobot, fullRobotModel.getChest().getParentJoint().getName(), new Vector3D(0, 0, 0.3));
   }
   
   public PushRobotController(FloatingRootJointRobot pushableRobot, String jointNameToApplyForce, Vector3D forcePointOffset)
   {
      yoTime = pushableRobot.getYoTime();
      registry = new YoVariableRegistry(jointNameToApplyForce + "_" + getClass().getSimpleName());
      forcePoint = new ExternalForcePoint(jointNameToApplyForce + "_externalForcePoint", forcePointOffset, pushableRobot);
      
      pushDuration = new DoubleYoVariable(jointNameToApplyForce + "_pushDuration", registry);
      pushForceMagnitude = new DoubleYoVariable(jointNameToApplyForce + "_pushMagnitude", registry);
      pushDirection = new YoFrameVector(jointNameToApplyForce + "_pushDirection", worldFrame, registry);
      pushForce = new YoFrameVector(jointNameToApplyForce + "_pushForce", worldFrame, registry);
      pushTimeSwitch = new DoubleYoVariable(jointNameToApplyForce + "_pushTimeSwitch", registry);
      pushNumber = new IntegerYoVariable(jointNameToApplyForce + "_pushNumber", registry);
      isBeingPushed = new BooleanYoVariable(jointNameToApplyForce + "_isBeingPushed", registry);
      pushDelay = new DoubleYoVariable(jointNameToApplyForce + "_pushDelay", registry);
      
      pushableRobot.getJoint(jointNameToApplyForce).addExternalForcePoint(forcePoint);
      pushableRobot.setController(this);

      pushTimeSwitch.set(Double.NEGATIVE_INFINITY);
      pushForceMagnitude.set(0.0);
      
      forceVisualizer = new YoGraphicVector(jointNameToApplyForce + "_pushForce", forcePoint.getYoPosition(), forcePoint.getYoForce(), 0.005, YoAppearance.DarkBlue());
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

   public void setPushForceDirection(Vector3D direction)
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

   public void applyForce(Vector3D direction, double magnitude, double duration)
   {
      applyForceDelayed(null, 0.0, direction, magnitude, duration);
   }

   public void applyForceDelayed(StateTransitionCondition pushCondition, double timeDelay, Vector3D direction, double magnitude, double duration)
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