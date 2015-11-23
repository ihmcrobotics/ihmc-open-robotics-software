package us.ihmc.valkyrie.controllers;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Unmarshaller;

import net.java.games.input.Component;
import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.CommonNames;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.InverseDynamicsJointController;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.visualization.WalkControllerSliderBoard;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.joystick.BooleanYoVariableJoystickEventListener;
import us.ihmc.simulationconstructionset.joystick.DoubleYoVariableJoystickEventListener;
import us.ihmc.simulationconstructionset.joystick.JoystickUpdater;
import us.ihmc.simulationconstructionset.util.inputdevices.SliderBoardConfigurationManager;
import us.ihmc.simulationconstructionset.util.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosFloat32MultiArraySubscriber;
import us.ihmc.valkyrie.configuration.ValkyrieConfigurationRoot;
import us.ihmc.valkyrie.kinematics.urdf.Interface;
import us.ihmc.valkyrie.kinematics.urdf.Transmission;
import us.ihmc.valkyrie.kinematics.urdf.URDFRobotRoot;

/**
 * Created by dstephen on 2/28/14.
 */
public class ValkyrieSliderBoard
{
   public enum ValkyrieSliderBoardType
   {
      ON_BOARD_POSITION, TORQUE_PD_CONTROL, WALKING, TUNING, GRAVITY_COMPENSATION
   }

   private final EnumYoVariable<ValkyrieSliderBoardSelectableJoints> selectedJoint, controllerSelectedJoint;

   private final LinkedHashMap<String, IntegerYoVariable> storedTurboIndex = new LinkedHashMap<>();

   private final IntegerYoVariable remoteTurboIndex;

   @SuppressWarnings("unchecked")
   public ValkyrieSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, DRCRobotModel drcRobotModel, ValkyrieSliderBoardType sliderBoardType)
   {
      selectedJoint = new EnumYoVariable<>("selectedJoint", registry, ValkyrieSliderBoardSelectableJoints.class);
      selectedJoint.set(ValkyrieSliderBoardSelectableJoints.RightKneeExtensor);

      controllerSelectedJoint = (EnumYoVariable<ValkyrieSliderBoardSelectableJoints>) registry.getVariable("ValkyrieSliderBoardController", "selectedJoint");

      remoteTurboIndex = (IntegerYoVariable) registry.getVariable("ValkyrieSliderBoardController", "turboIndex");

      final SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);

      switch (sliderBoardType)
      {
      case ON_BOARD_POSITION:
         setupSliderBoardForOnBoardPositionControl(registry, drcRobotModel.getGeneralizedRobotModel(), sliderBoardConfigurationManager);

         break;

      case TORQUE_PD_CONTROL:
         setupSliderBoardForForceControl(registry, drcRobotModel.getGeneralizedRobotModel(), sliderBoardConfigurationManager);

         break;

      case WALKING:
         new WalkControllerSliderBoard(scs, registry, drcRobotModel);
         setupJoyStickAndTreadmill(registry);
         break;

      case TUNING:
         try
         {
            setupSliderBoardForForceControlTuning(registry, drcRobotModel.getGeneralizedRobotModel(), sliderBoardConfigurationManager);
         }
         catch (JAXBException e)
         {
            e.printStackTrace();
         }

         break;
      case GRAVITY_COMPENSATION:

         new InverseDynamicsJointController.GravityCompensationSliderBoard(scs, drcRobotModel.createFullRobotModel(), registry,
               CommonNames.doIHMCControlRatio.toString(), 0.0, 1.0);

         break;
      }

      sliderBoardConfigurationManager.loadConfiguration(selectedJoint.getEnumValue().toString());
   }

   public static void setupJoyStickAndTreadmill(YoVariableRegistry registry)
   {
      final JoystickUpdater joystickUpdater = new JoystickUpdater();
      Thread thread = new Thread(joystickUpdater);
      thread.start();

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
      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredVelocityX, joystickUpdater.findComponent(Component.Identifier.Axis.Y),
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
      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredVelocityY, joystickUpdater.findComponent(Component.Identifier.Axis.X),
            -0.1 + desiredVelocityY_Bias, 0.1 + desiredVelocityY_Bias, deadZone, true));

      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredHeadingDot, joystickUpdater.findComponent(Component.Identifier.Axis.RZ),
            minHeadingDot + desiredHeadingDot_Bias, maxHeadingDot + desiredHeadingDot_Bias, deadZone, signFlip));

      BooleanYoVariable walk = (BooleanYoVariable) registry.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper", "walk");
      joystickUpdater.addListener(new BooleanYoVariableJoystickEventListener(walk, joystickUpdater.findComponent(Component.Identifier.Button.TRIGGER), true));

   }

   private void setupSliderBoardForForceControl(YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel,
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
            sliderBoardConfigurationManager.setSlider(1, pdControllerBaseName + "_q_d", registry,
                  generalizedSDFRobotModel.getJointHolder(jointName).getLowerLimit(), generalizedSDFRobotModel.getJointHolder(jointName).getUpperLimit());
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

   private void setupSliderBoardForForceControlTuning(YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel,
         final SliderBoardConfigurationManager sliderBoardConfigurationManager) throws JAXBException
   {
      JAXBContext context = JAXBContext.newInstance(URDFRobotRoot.class);
      Unmarshaller um = context.createUnmarshaller();
      URDFRobotRoot urdfRoot = (URDFRobotRoot) um.unmarshal(ValkyrieConfigurationRoot.class.getResourceAsStream(ValkyrieConfigurationRoot.URDF_FILE));

      for (Transmission t : urdfRoot.getTransmissions())
      {
         for (Interface i : t.getInterfaces())
         {
            if (i.getType().equals("JointToActuatorStateInterface"))
            {
               ArrayList<String> turbos = new ArrayList<>();
               boolean isForearm = false;
               for (Interface.Actuator a : i.getActuators())
               {
                  if (a.getName().toLowerCase().contains("forearm"))
                  {
                     isForearm = true;
                  }
                  else
                  {
                     turbos.add(a.getName().replace("/", "_"));
                  }
               }

               for (Interface.Joint j : i.getJoints())
               {
                  if (!isForearm)
                  {
                     String jointName = j.getName();
                     String pdControllerBaseName = jointName + "ValkyrieJointTorqueControlTuner";

                     if (!jointName.contains("Ibeo"))
                     {
                        if (isTunableRotaryJoint(jointName))
                        {
                           assert turbos.size() == 1;

                           String turboName = turbos.get(0);
                           // knobs
                           sliderBoardConfigurationManager.setKnob(1, selectedJoint, 0, ValkyrieSliderBoardSelectableJoints.values().length - 1);
                           sliderBoardConfigurationManager.setKnob(3, turboName + "_lowLevelKp", registry, 0.0, 20.0);
                           sliderBoardConfigurationManager.setKnob(4, turboName + "_lowLevelKd", registry, 0.0, 0.5);
                           sliderBoardConfigurationManager.setKnob(5, turboName + "_forceAlpha", registry, 0.0, 1.0);
                           sliderBoardConfigurationManager.setKnob(6, turboName + "_forceDotAlpha", registry, 0.0, 1.0);
                           sliderBoardConfigurationManager.setKnob(7, turboName + "_parallelDamping", registry, -10.0, 0.0);
                           sliderBoardConfigurationManager.setKnob(8, "requestedFunctionGeneratorMode", registry, 0,
                                 YoFunctionGeneratorMode.values().length - 1);
                           sliderBoardConfigurationManager.setKnob(9, turboName + "_effortFF", registry, -0.1, 0.1);
                           // sliders
                           sliderBoardConfigurationManager.setSlider(1, pdControllerBaseName + "_q_d", registry,
                                 generalizedSDFRobotModel.getJointHolder(jointName).getLowerLimit(),
                                 generalizedSDFRobotModel.getJointHolder(jointName).getUpperLimit());
                           sliderBoardConfigurationManager.setSlider(2, "kp_" + pdControllerBaseName, registry, 0.0, 2000.0);
                           sliderBoardConfigurationManager.setSlider(3, "kd_" + pdControllerBaseName, registry, 0.0, 600.0);
                           sliderBoardConfigurationManager.setSlider(4, "ki_" + pdControllerBaseName, registry, 0.0, 600.0);
                           sliderBoardConfigurationManager.setSlider(5, pdControllerBaseName + "_transitionFactor", registry, 0.0, 1.0);

                           sliderBoardConfigurationManager.setSlider(6, pdControllerBaseName + "_functionGeneratorAmplitude", registry, 0, 200);
                           sliderBoardConfigurationManager.setSlider(7, pdControllerBaseName + "_functionGeneratorFrequency", registry, 0, 50);
                           sliderBoardConfigurationManager.setSlider(8, pdControllerBaseName + "_functionGeneratorOffset", registry, -100, 100);

                           sliderBoardConfigurationManager.saveConfiguration(jointName);
                           sliderBoardConfigurationManager.clearControls();
                        }
                        else if (isTunableLinearActuatorJoint(jointName))
                        {
                           assert turbos.size() == 2;
                           IntegerYoVariable turboIndexMonitor = new IntegerYoVariable(jointName + "_turboIndexMonitor", registry);

                           for (int count = 0; count < turbos.size(); count++)
                           {
                              String turboName = turbos.get(count);

                              // knobs
                              sliderBoardConfigurationManager.setKnob(1, selectedJoint, 0, ValkyrieSliderBoardSelectableJoints.values().length - 1);
                              sliderBoardConfigurationManager.setKnob(2, turboIndexMonitor, 0, 1);
                              sliderBoardConfigurationManager.setKnob(3, turboName + "_lowLevelKp", registry, 0.0, 0.1);
                              sliderBoardConfigurationManager.setKnob(4, turboName + "_lowLevelKd", registry, 0.0, 0.001);
                              sliderBoardConfigurationManager.setKnob(5, turboName + "_forceAlpha", registry, 0.0, 1.0);
                              sliderBoardConfigurationManager.setKnob(6, turboName + "_forceDotAlpha", registry, 0.0, 1.0);
                              sliderBoardConfigurationManager.setKnob(7, turboName + "_parallelDamping", registry, -10.0, 0.0);
                              sliderBoardConfigurationManager.setKnob(8, "requestedFunctionGeneratorMode", registry, 0,
                                    YoFunctionGeneratorMode.values().length - 1);
                              sliderBoardConfigurationManager.setKnob(9, turboName + "_effortFF", registry, -0.01, 0.01);
                              // sliders
                              sliderBoardConfigurationManager.setSlider(1, pdControllerBaseName + "_q_d", registry,
                                    generalizedSDFRobotModel.getJointHolder(jointName).getLowerLimit(),
                                    generalizedSDFRobotModel.getJointHolder(jointName).getUpperLimit());
                              sliderBoardConfigurationManager.setSlider(2, "kp_" + pdControllerBaseName, registry, 0.0, 2000.0);
                              sliderBoardConfigurationManager.setSlider(3, "kd_" + pdControllerBaseName, registry, 0.0, 600.0);
                              sliderBoardConfigurationManager.setSlider(4, "ki_" + pdControllerBaseName, registry, 0.0, 600.0);
                              sliderBoardConfigurationManager.setSlider(5, pdControllerBaseName + "_transitionFactor", registry, 0.0, 1.0);
                              // sliderBoardConfigurationManager.setSlider(5,
                              // pdControllerBaseName + "_tauDesired",
                              // registry, -100.0, 100.0);

                              // sliderBoardConfigurationManager.setButton(1,
                              // pdControllerBaseName +
                              // "_useFunctionGenerator", registry);
                              sliderBoardConfigurationManager.setSlider(6, pdControllerBaseName + "_functionGeneratorAmplitude", registry, 0, 200);
                              sliderBoardConfigurationManager.setSlider(7, pdControllerBaseName + "_functionGeneratorFrequency", registry, 0, 50);
                              sliderBoardConfigurationManager.setSlider(8, pdControllerBaseName + "_functionGeneratorOffset", registry, -100, 100);

                              sliderBoardConfigurationManager.saveConfiguration(jointName + count);
                              sliderBoardConfigurationManager.clearControls();
                           }

                           storedTurboIndex.put(jointName, turboIndexMonitor);

                           turboIndexMonitor.addVariableChangedListener(new VariableChangedListener()
                           {
                              @Override
                              public void variableChanged(YoVariable<?> v)
                              {
                                 if (isTunableRotaryJoint(selectedJoint.getEnumValue().toString()))
                                 {
                                    remoteTurboIndex.set(0);
                                    System.out.println("loading configuration " + selectedJoint.getEnumValue());
                                    sliderBoardConfigurationManager.loadConfiguration(selectedJoint.getEnumValue().toString());
                                 }

                                 if (isTunableLinearActuatorJoint(selectedJoint.getEnumValue().toString()))
                                 {
                                    int storedIndex = storedTurboIndex.get(selectedJoint.getEnumValue().toString()).getIntegerValue();
                                    remoteTurboIndex.set(storedIndex);
                                    System.out.println("loading configuration " + selectedJoint.getEnumValue() + " " + storedIndex);
                                    sliderBoardConfigurationManager.loadConfiguration(selectedJoint.getEnumValue().toString() + storedIndex);
                                 }
                              }
                           });
                        }
                     }
                  }
               }
            }
         }
      }

      selectedJoint.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (isTunableRotaryJoint(selectedJoint.getEnumValue().toString()))
            {
               remoteTurboIndex.set(0);
               System.out.println("loading configuration " + selectedJoint.getEnumValue());
               sliderBoardConfigurationManager.loadConfiguration(selectedJoint.getEnumValue().toString());
               controllerSelectedJoint.set(selectedJoint.getEnumValue());
            }

            if (isTunableLinearActuatorJoint(selectedJoint.getEnumValue().toString()))
            {
               int storedIndex = storedTurboIndex.get(selectedJoint.getEnumValue().toString()).getIntegerValue();
               remoteTurboIndex.set(storedIndex);
               System.out.println("loading configuration " + selectedJoint.getEnumValue() + " " + storedIndex);
               sliderBoardConfigurationManager.loadConfiguration(selectedJoint.getEnumValue().toString() + storedIndex);
               controllerSelectedJoint.set(selectedJoint.getEnumValue());
            }
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

   private void setupSliderBoardForOnBoardPositionControl(YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel,
         final SliderBoardConfigurationManager sliderBoardConfigurationManager)
   {
      for (ValkyrieSliderBoardSelectableJoints jointId : ValkyrieSliderBoardSelectableJoints.values())
      {
         String jointName = jointId.toString();
         System.out.println(jointName);

         // knobs

         sliderBoardConfigurationManager.setKnob(1, selectedJoint, 0, ValkyrieSliderBoardSelectableJoints.values().length - 1);

         // sliders
         sliderBoardConfigurationManager.setSlider(1, jointName + CommonNames.q_d, registry, generalizedSDFRobotModel.getJointHolder(jointName).getLowerLimit(),
               generalizedSDFRobotModel.getJointHolder(jointName).getUpperLimit());
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
