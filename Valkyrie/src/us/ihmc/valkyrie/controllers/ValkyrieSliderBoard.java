package us.ihmc.valkyrie.controllers;

import java.io.IOException;

import net.java.games.input.Component;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.visualization.WalkControllerSliderBoard;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.robotDescription.OneDoFJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationConstructionSetTools.joystick.BooleanYoVariableJoystickEventListener;
import us.ihmc.simulationConstructionSetTools.joystick.DoubleYoVariableJoystickEventListener;
import us.ihmc.simulationconstructionset.util.CommonNames;
import us.ihmc.simulationConstructionSetTools.util.inputdevices.SliderBoardConfigurationManager;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosFloat32MultiArraySubscriber;

/**
 * Created by dstephen on 2/28/14.
 */
public class ValkyrieSliderBoard
{
   public enum ValkyrieSliderBoardType
   {
      ON_BOARD_POSITION, TORQUE_PD_CONTROL, WALKING, GRAVITY_COMPENSATION
   }

   private final EnumYoVariable<ValkyrieSliderBoardSelectableJoints> selectedJoint;


   public ValkyrieSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, DRCRobotModel drcRobotModel, ValkyrieSliderBoardType sliderBoardType)
   {
      selectedJoint = new EnumYoVariable<>("selectedJoint", registry, ValkyrieSliderBoardSelectableJoints.class);
      selectedJoint.set(ValkyrieSliderBoardSelectableJoints.RightKneeExtensor);


      final SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);

      switch (sliderBoardType)
      {
      case ON_BOARD_POSITION:
         setupSliderBoardForOnBoardPositionControl(registry, drcRobotModel.getRobotDescription(), sliderBoardConfigurationManager);

         break;

      case TORQUE_PD_CONTROL:
         setupSliderBoardForForceControl(registry, drcRobotModel.getRobotDescription(), sliderBoardConfigurationManager);

         break;

      case WALKING:
         new WalkControllerSliderBoard(scs, registry, drcRobotModel);
         setupJoyStickAndTreadmill(registry);
         break;
      }

      sliderBoardConfigurationManager.loadConfiguration(selectedJoint.getEnumValue().toString());
   }

   public static void setupJoyStickAndTreadmill(YoVariableRegistry registry)
   {
      Joystick joystickUpdater;
      try
      {
         joystickUpdater = new Joystick();
      }
      catch (IOException e)
      {
         System.out.println("No joystick detected.");
         return;
      }

      double deadZone = 0.02;
      final double desiredVelocityX_Bias = 0.0;
      double desiredVelocityY_Bias = 0.0;
      final double desiredHeadingDot_Bias = 0.0;

      final double minHeadingDot = -0.1, minVelocityX = -0.2, maxHeadingDot = 0.1, maxVelocityX = 0.2;
      boolean signFlip = true;

      // Start the heading and speed updater thread
      //--(!) Requires data from MultisenseHeadingSubscriber
      final ValkyrieHeadingUpdater valkyrieHeadingUpdater = new ValkyrieHeadingUpdater(registry, minHeadingDot, maxHeadingDot, minVelocityX, maxVelocityX,
            signFlip);
      final Thread headingUpdaterThread = new Thread(valkyrieHeadingUpdater);
      headingUpdaterThread.start();
      //--------------------
      // Speed and Heading Controller
      //--------------------
      final BooleanYoVariable isMultisenseControllingSpeedAndHeading = new BooleanYoVariable("isMultisenseControllingSpeedAndHeading", registry);
      final DoubleYoVariable headingDotConstant = new DoubleYoVariable("headingDotConstant", registry);
      final DoubleYoVariable velocityXConstant = new DoubleYoVariable("velocityXConstant", registry);

      final DoubleYoVariable desiredHeadingDot = (DoubleYoVariable) registry.getVariable("RateBasedDesiredHeadingControlModule", "desiredHeadingDot");
      final DoubleYoVariable desiredVelocityX = (DoubleYoVariable) registry.getVariable("ManualDesiredVelocityControlModule", "desiredVelocityX");
      if (desiredVelocityX == null || joystickUpdater == null)
         return;

      isMultisenseControllingSpeedAndHeading.set(false);
      isMultisenseControllingSpeedAndHeading.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            // Reset heading and speed when toggling controller
            if (v.getValueAsDouble() != 0)
            {
               desiredVelocityX.set(desiredVelocityX_Bias);
               desiredHeadingDot.set(desiredHeadingDot_Bias);
            }

         }
      });

      headingDotConstant.set(0.5);
      velocityXConstant.set(0.1);
      desiredHeadingDot.set(desiredHeadingDot_Bias);

      desiredHeadingDot.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            // Overwrite joystick's input if it is disabled for heading control.
            if (isMultisenseControllingSpeedAndHeading.getBooleanValue())
               desiredHeadingDot.set(valkyrieHeadingUpdater.currentHeadingDot);
         }
      });

      desiredVelocityX.set(desiredVelocityX_Bias);
      joystickUpdater.addJoystickEventListener(new DoubleYoVariableJoystickEventListener(desiredVelocityX, joystickUpdater.findComponent(Component.Identifier.Axis.Y),
            minVelocityX + desiredVelocityX_Bias, maxVelocityX + desiredVelocityX_Bias, deadZone, true));
      desiredVelocityX.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            // Overwrite joystick's input if it is disabled for speed control.
            if (isMultisenseControllingSpeedAndHeading.getBooleanValue())
               v.setValueFromDouble(valkyrieHeadingUpdater.currentVelocityX);
            if (v.getValueAsDouble() < minVelocityX)
               v.setValueFromDouble(minVelocityX, false);
         }
      });

      DoubleYoVariable desiredVelocityY = (DoubleYoVariable) registry.getVariable("ManualDesiredVelocityControlModule", "desiredVelocityY");
      desiredVelocityY.set(desiredVelocityY_Bias);
      joystickUpdater.addJoystickEventListener(new DoubleYoVariableJoystickEventListener(desiredVelocityY, joystickUpdater.findComponent(Component.Identifier.Axis.X),
            -0.1 + desiredVelocityY_Bias, 0.1 + desiredVelocityY_Bias, deadZone, true));

      joystickUpdater.addJoystickEventListener(new DoubleYoVariableJoystickEventListener(desiredHeadingDot, joystickUpdater.findComponent(Component.Identifier.Axis.RZ),
            minHeadingDot + desiredHeadingDot_Bias, maxHeadingDot + desiredHeadingDot_Bias, deadZone, signFlip));

      BooleanYoVariable walk = (BooleanYoVariable) registry.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper", "walk");
      joystickUpdater.addJoystickEventListener(new BooleanYoVariableJoystickEventListener(walk, joystickUpdater.findComponent(Component.Identifier.Button.TRIGGER), true));

   }

   private void setupSliderBoardForForceControl(YoVariableRegistry registry, RobotDescription robotDescription,
         final SliderBoardConfigurationManager sliderBoardConfigurationManager)
   {
      for (ValkyrieSliderBoardSelectableJoints jointId : ValkyrieSliderBoardSelectableJoints.values())
      {
         String jointName = jointId.toString();
         if (!jointName.contains("Ibeo"))
         {
            String pdControllerBaseName = jointName + "ValkyrieJointPDController";

            // knobs

            sliderBoardConfigurationManager.setKnob(1, selectedJoint, 0, ValkyrieSliderBoardSelectableJoints.values().length - 1);

            // sliders
            OneDoFJointDescription jointDescription = (OneDoFJointDescription) robotDescription.getJointDescription(jointName);
            sliderBoardConfigurationManager.setSlider(1, pdControllerBaseName + "_q_d", registry,
                  jointDescription.getLowerLimit(), jointDescription.getUpperLimit());
            sliderBoardConfigurationManager.setSlider(2, "kp_" + pdControllerBaseName, registry, 0.0, 2000.0);
            sliderBoardConfigurationManager.setSlider(3, "kd_" + pdControllerBaseName, registry, 0.0, 600.0);

            sliderBoardConfigurationManager.saveConfiguration(jointId.toString());
         }

      }

      selectedJoint.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            System.out.println("loading configuration " + selectedJoint.getEnumValue());
            sliderBoardConfigurationManager.loadConfiguration(selectedJoint.getEnumValue().toString());
         }
      });
   }


   static final String[] untunableOrNonRotaryJoints = new String[] { "WaistExtensor", "WaistLateral", "Ankle", "Neck", "Forearm", "Wrist" };

   private boolean isTunableRotaryJoint(String jointName)
   {
      boolean ret = true;

      for (String s : untunableOrNonRotaryJoints)
      {
         if (jointName.contains(s))
         {
            ret = false;
         }
      }

      return ret;
   }

   static final String[] tunableLinearActuatorJoint = new String[] { "WaistExtensor", "WaistLateral", "Ankle" };

   private boolean isTunableLinearActuatorJoint(String jointName)
   {
      for (String s : tunableLinearActuatorJoint)
      {
         if (jointName.contains(s))
         {
            return true;
         }
      }

      return false;
   }

   private void setupSliderBoardForOnBoardPositionControl(YoVariableRegistry registry, RobotDescription robotDescription,
         final SliderBoardConfigurationManager sliderBoardConfigurationManager)
   {
      for (ValkyrieSliderBoardSelectableJoints jointId : ValkyrieSliderBoardSelectableJoints.values())
      {
         String jointName = jointId.toString();
         System.out.println(jointName);

         // knobs

         sliderBoardConfigurationManager.setKnob(1, selectedJoint, 0, ValkyrieSliderBoardSelectableJoints.values().length - 1);

         // sliders
         OneDoFJointDescription jointDescription = (OneDoFJointDescription) robotDescription.getJointDescription(jointName);
         sliderBoardConfigurationManager.setSlider(1, jointName + CommonNames.q_d, registry, jointDescription.getLowerLimit(),
               jointDescription.getUpperLimit());
         sliderBoardConfigurationManager.setSlider(2, jointName + CommonNames.qd_d, registry, -9, 9);

         sliderBoardConfigurationManager.saveConfiguration(jointId.toString());
      }

      selectedJoint.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            System.out.println("loading configuration " + selectedJoint.getEnumValue());
            sliderBoardConfigurationManager.loadConfiguration(selectedJoint.getEnumValue().toString());
         }
      });
   }

   public static class MultisenseHeadingSubscriber extends RosFloat32MultiArraySubscriber
   {
      YoVariableRegistry registry;

      double angleRadians, magnitudeMeters;

      public MultisenseHeadingSubscriber(RosMainNode rosnode, String msgTopic, YoVariableRegistry inputRegistry)
      {
         super();
         registry = inputRegistry;
         angleRadians = 0.0;
         magnitudeMeters = 0.0;
         DoubleYoVariable multisenseHeading = new DoubleYoVariable("multisenseHeading", registry);
         DoubleYoVariable multisenseMagnitude = new DoubleYoVariable("multisenseMagnitude", registry);
         multisenseHeading.setValueFromDouble(0.0);
         multisenseMagnitude.setValueFromDouble(0.0);

         rosnode.attachSubscriber(msgTopic, this);

      }

      @Override
      public void onNewMessage(std_msgs.Float32MultiArray msg)
      {
         if (msg._TYPE.equals(this.getMessageType()))
         {
            java.util.List<std_msgs.MultiArrayDimension> dims = msg.getLayout().getDim();
            float[] data = msg.getData();

            if (dims.size() != 2)
               System.out.println("ERROR: Multisense message data is not a 2d array.");

            if (!(dims.get(0).getSize() == 1 && dims.get(1).getSize() == 2))
               System.out.println("ERROR: Multisense message data is not a 1x2 vector array.");

            magnitudeMeters = data[dims.get(0).getSize() * 0 + dims.get(1).getStride() * 0];
            angleRadians = data[dims.get(0).getSize() * 0 + dims.get(1).getStride() * 1];

            DoubleYoVariable vh = (DoubleYoVariable) registry.getVariable("multisenseHeading");
            DoubleYoVariable vm = (DoubleYoVariable) registry.getVariable("multisenseMagnitude");
            vh.setValueFromDouble(angleRadians);
            vm.setValueFromDouble(magnitudeMeters);

         }
         else
         {
            System.out.println("ERROR: Invalid message. Expecting " + this.getMessageType());
         }

      }

      @Override
      public String toString()
      {
         return "<" + magnitudeMeters + "m, " + angleRadians + "rad>";
      }

      public double getAngleInDegrees()
      {
         return this.angleRadians * 180.0 / Math.PI;
      }

   }

   public static class ValkyrieHeadingUpdater extends Thread
   {
      private final double EPSILON = 0.05;
      YoVariableHolder registry;
      double minHeadingDot, maxHeadingDot, minVeclocity, maxVelocity, currentHeadingDot, currentVelocityX;
      boolean signFlip;

      public ValkyrieHeadingUpdater(YoVariableHolder holder, double minHeadingDot, double maxHeadingDot, double minVelocity, double maxVelocity,
            boolean signFlip)
      {
         registry = holder;
         this.minHeadingDot = minHeadingDot;
         this.maxHeadingDot = maxHeadingDot;
         this.minVeclocity = minVelocity;
         this.maxVelocity = maxVelocity;
         if (signFlip)
         {
            this.minHeadingDot = -1 * maxHeadingDot;
            this.maxHeadingDot = -1 * minHeadingDot;
         }
         this.currentHeadingDot = 0.0;
         this.currentVelocityX = 0.0;
         this.signFlip = signFlip;
      }

      public void run()
      {
         while (true)
         {
            final DoubleYoVariable headingDotConstant = (DoubleYoVariable) registry.getVariable("headingDotConstant");
            final DoubleYoVariable velocityXConstant = (DoubleYoVariable) registry.getVariable("velocityXConstant");
            BooleanYoVariable multisenseControlsSpeedAndHeading = (BooleanYoVariable) registry.getVariable("isMultisenseControllingSpeedAndHeading");

            DoubleYoVariable desiredHeadingDot = (DoubleYoVariable) registry.getVariable("RateBasedDesiredHeadingControlModule", "desiredHeadingDot");
            DoubleYoVariable desiredVelocityX = (DoubleYoVariable) registry.getVariable("ManualDesiredVelocityControlModule", "desiredVelocityX");

            DoubleYoVariable multisenseHeading = (DoubleYoVariable) registry.getVariable("multisenseHeading");
            DoubleYoVariable multisenseMagnitude = (DoubleYoVariable) registry.getVariable("multisenseMagnitude");

            if (multisenseControlsSpeedAndHeading == null || desiredHeadingDot == null || desiredVelocityX == null || multisenseHeading == null
                  || multisenseMagnitude == null)
            {
               continue;
            }

            if (multisenseControlsSpeedAndHeading.getBooleanValue())
            {
               if (Math.abs(multisenseHeading.getDoubleValue()) > EPSILON)
               {
                  currentHeadingDot = multisenseHeading.getDoubleValue() * headingDotConstant.getDoubleValue();

                  if (currentHeadingDot < minHeadingDot)
                     currentHeadingDot = minHeadingDot;
                  if (currentHeadingDot > maxHeadingDot)
                     currentHeadingDot = maxHeadingDot;
               }
               else
               {
                  currentHeadingDot = 0.0;
               }

               double newVelocity = multisenseMagnitude.getDoubleValue() * velocityXConstant.getDoubleValue();
               //--(!) stop at half a meter from centroid of obj
               if (multisenseMagnitude.getValueAsDouble() > 0.5)
               {
                  //-- This precaution may not be crucial, but
                  //-- ramp up velocity for large jumps
                  double deltaVelocity = newVelocity - currentVelocityX;
                  if (Math.abs(deltaVelocity) > .1)
                     currentVelocityX += deltaVelocity / Math.abs(deltaVelocity) * 0.05;
                  else
                     currentVelocityX = newVelocity;

                  if (currentVelocityX < minVeclocity)
                     currentVelocityX = minVeclocity;
                  if (currentVelocityX > maxVelocity)
                     currentVelocityX = maxVelocity;
               }
               else
               {
                  currentVelocityX = 0.0;
               }

               desiredHeadingDot.set(currentHeadingDot);
               desiredVelocityX.set(currentVelocityX);
            }

            try
            {
               Thread.sleep(100); // 10Hz
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }
         }
      }

   }

}
