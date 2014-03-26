package us.ihmc.valkyrie.controllers;

import com.yobotics.simulationconstructionset.*;
import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.atlas.visualization.SliderBoardFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.CommonNames;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.InverseDynamicsJointController;
import us.ihmc.darpaRoboticsChallenge.valkyrie.ValkyrieJointMap;

import com.yobotics.simulationconstructionset.util.inputdevices.SliderBoardConfigurationManager;
import us.ihmc.valkyrie.configuration.ValkyrieConfigurationRoot;
import us.ihmc.valkyrie.kinematics.urdf.Interface;
import us.ihmc.valkyrie.kinematics.urdf.Transmission;
import us.ihmc.valkyrie.kinematics.urdf.URDFRobotRoot;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Unmarshaller;
import java.util.ArrayList;
import java.util.LinkedHashMap;

/**
 * Created by dstephen on 2/28/14.
 */
public class ValkyrieSliderBoard
{
   private enum ValkyrieSliderBoardType {ON_BOARD_POSITION, TORQUE_PD_CONTROL, WALKING, TUNING}

   private final EnumYoVariable<ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints> selectedJoint, remoteSelectedJoint;

   private final LinkedHashMap<String, IntegerYoVariable> storedTurboIndex = new LinkedHashMap<>();

   private final IntegerYoVariable updateIndex;

   @SuppressWarnings("unchecked")
   public ValkyrieSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel,
                              ValkyrieSliderBoardType sliderBoardType)
   {
      selectedJoint = new EnumYoVariable<>("selectedJoint", registry, ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints.class);
      selectedJoint.set(ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints.RightKneeExtensor);
      remoteSelectedJoint = (EnumYoVariable<ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints>) registry.getVariable(
         "remoteroot.ValkyrieSliderBoardController.selectedJoint");

      updateIndex = new IntegerYoVariable("updateIndex", registry);

      final SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);

      switch (sliderBoardType)
      {
         case ON_BOARD_POSITION :
            setupSliderBoardForOnBoardPositionControl(registry, generalizedSDFRobotModel, sliderBoardConfigurationManager);

            break;

         case TORQUE_PD_CONTROL :
            setupSliderBoardForForceControl(registry, generalizedSDFRobotModel, sliderBoardConfigurationManager);

            break;

         case WALKING :
            setupSliderBoardForWalking(registry, generalizedSDFRobotModel, sliderBoardConfigurationManager);

            break;

         case TUNING :
            try
            {
               setupSliderBoardForForceControlTuning(registry, generalizedSDFRobotModel, sliderBoardConfigurationManager);
            }
            catch (JAXBException e)
            {
               e.printStackTrace();
            }
      }

      sliderBoardConfigurationManager.loadConfiguration(selectedJoint.getEnumValue().toString());
   }

   private void setupSliderBoardForForceControl(YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel,
           final SliderBoardConfigurationManager sliderBoardConfigurationManager)
   {
      for (ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints jointId :
              ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints.values())
      {
         String jointName = jointId.toString();
         if (!jointName.contains("Ibeo"))
         {
            String pdControllerBaseName = jointName + "ValkyrieJointPDController";

            // knobs

            sliderBoardConfigurationManager.setKnob(1, selectedJoint, 0, ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints.values().length - 1);

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
         public void variableChanged(YoVariable v)
         {
            System.out.println("loading configuration " + selectedJoint.getEnumValue());
            sliderBoardConfigurationManager.loadConfiguration(selectedJoint.getEnumValue().toString());

            if (remoteSelectedJoint != null)
            {
               remoteSelectedJoint.set(selectedJoint.getEnumValue());
            }
         }
      });
   }

   private void setupSliderBoardForForceControlTuning(YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel,
           final SliderBoardConfigurationManager sliderBoardConfigurationManager)
           throws JAXBException
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
                  if(a.getName().toLowerCase().contains("forearm"))
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
                           sliderBoardConfigurationManager.setKnob(1, selectedJoint, 0,
                                   ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints.values().length - 1);
                           sliderBoardConfigurationManager.setKnob(3, turboName + "_lowLevelKp", registry, 0.0, 12.0);
                           sliderBoardConfigurationManager.setKnob(4, turboName + "_lowLevelKd", registry, 0.0, 12.0);
                           sliderBoardConfigurationManager.setKnob(5, turboName + "_forceAlpha", registry, 0.0, 1.0);
                           sliderBoardConfigurationManager.setKnob(6, turboName + "_forceDotAlpha", registry, 0.0, 1.0);
                           sliderBoardConfigurationManager.setKnob(7, turboName + "_parallelDamping", registry, 0.0, 10.0);

                           // sliders
                           sliderBoardConfigurationManager.setSlider(1, pdControllerBaseName + "_q_d", registry,
                                   generalizedSDFRobotModel.getJointHolder(jointName).getLowerLimit(),
                                   generalizedSDFRobotModel.getJointHolder(jointName).getUpperLimit());
                           sliderBoardConfigurationManager.setSlider(2, "kp_" + pdControllerBaseName, registry, 0.0, 2000.0);
                           sliderBoardConfigurationManager.setSlider(3, "kd_" + pdControllerBaseName, registry, 0.0, 600.0);
                           sliderBoardConfigurationManager.setSlider(4, pdControllerBaseName + "_transitionFactor", registry, 0.0, 1.0);
                           sliderBoardConfigurationManager.setSlider(5, pdControllerBaseName + "_tauDesired", registry, 0.0, 100.0);

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
                              sliderBoardConfigurationManager.setKnob(1, selectedJoint, 0,
                                      ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints.values().length - 1);
                              sliderBoardConfigurationManager.setKnob(2, turboIndexMonitor, 0, 1);
                              sliderBoardConfigurationManager.setKnob(3, turboName + "_lowLevelKp", registry, 0.0, 12.0);
                              sliderBoardConfigurationManager.setKnob(4, turboName + "_lowLevelKd", registry, 0.0, 12.0);
                              sliderBoardConfigurationManager.setKnob(5, turboName + "_forceAlpha", registry, 0.0, 1.0);
                              sliderBoardConfigurationManager.setKnob(6, turboName + "_forceDotAlpha", registry, 0.0, 1.0);
                              sliderBoardConfigurationManager.setKnob(7, turboName + "_parallelDamping", registry, 0.0, 10.0);

                              // sliders
                              sliderBoardConfigurationManager.setSlider(1, pdControllerBaseName + "_q_d", registry,
                                      generalizedSDFRobotModel.getJointHolder(jointName).getLowerLimit(),
                                      generalizedSDFRobotModel.getJointHolder(jointName).getUpperLimit());
                              sliderBoardConfigurationManager.setSlider(2, "kp_" + pdControllerBaseName, registry, 0.0, 2000.0);
                              sliderBoardConfigurationManager.setSlider(3, "kd_" + pdControllerBaseName, registry, 0.0, 600.0);
                              sliderBoardConfigurationManager.setSlider(4, pdControllerBaseName + "_transitionFactor", registry, 0.0, 1.0);
                              sliderBoardConfigurationManager.setSlider(5, pdControllerBaseName + "_tauDesired", registry, 0.0, 100.0);

                              sliderBoardConfigurationManager.saveConfiguration(jointName + count);
                              sliderBoardConfigurationManager.clearControls();
                           }

                           storedTurboIndex.put(jointName, turboIndexMonitor);

                           turboIndexMonitor.addVariableChangedListener(new VariableChangedListener()
                           {
                              @Override
                              public void variableChanged(YoVariable v)
                              {
                                 if (isTunableRotaryJoint(selectedJoint.getEnumValue().toString()))
                                 {
                                    System.out.println("loading configuration " + selectedJoint.getEnumValue());
                                    sliderBoardConfigurationManager.loadConfiguration(selectedJoint.getEnumValue().toString());
                                 }

                                 if (isTunableLinearActuatorJoint(selectedJoint.getEnumValue().toString()))
                                 {
                                    int storedIndex = storedTurboIndex.get(selectedJoint.getEnumType().toString()).getIntegerValue();
                                    System.out.println("loading configuration " + selectedJoint.getEnumValue() + " " + storedIndex);
                                    sliderBoardConfigurationManager.loadConfiguration(selectedJoint.getEnumValue().toString() + storedIndex);
                                 }

                                 if (remoteSelectedJoint != null)
                                 {
                                    remoteSelectedJoint.set(selectedJoint.getEnumValue());
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
         public void variableChanged(YoVariable v)
         {
            if (isTunableRotaryJoint(selectedJoint.getEnumValue().toString()))
            {
               System.out.println("loading configuration " + selectedJoint.getEnumValue());
               sliderBoardConfigurationManager.loadConfiguration(selectedJoint.getEnumValue().toString());
            }

            if (isTunableLinearActuatorJoint(selectedJoint.getEnumValue().toString()))
            {
               int storedIndex = storedTurboIndex.get(selectedJoint.getEnumType().toString()).getIntegerValue();
               System.out.println("loading configuration " + selectedJoint.getEnumValue() + " " + storedIndex);
               sliderBoardConfigurationManager.loadConfiguration(selectedJoint.getEnumValue().toString() + storedIndex);
            }

            if (remoteSelectedJoint != null)
            {
               remoteSelectedJoint.set(selectedJoint.getEnumValue());
            }
         }
      });
   }

   static final String[] untunableOrNonRotaryJoints = new String[]
   {
      "WaistExtensor", "WaistLateral", "Ankle", "Neck", "Forearm", "Wrist"
   };

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

   static final String[] tunableLinearActuatorJoint = new String[] {"WaistExtensor", "WaistLateral", "Ankle"};

   private boolean isTunableLinearActuatorJoint(String jointName)
   {
      boolean ret = false;

      for (String s : tunableLinearActuatorJoint)
      {
         if (jointName.contains(s))
         {
            ret = true;
         }
      }

      return ret;
   }

   private void setupSliderBoardForOnBoardPositionControl(YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel,
           final SliderBoardConfigurationManager sliderBoardConfigurationManager)
   {
      for (ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints jointId :
              ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints.values())
      {
         String jointName = jointId.toString();
         System.out.println(jointName);

         // knobs

         sliderBoardConfigurationManager.setKnob(1, selectedJoint, 0, ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints.values().length - 1);

         // sliders
         sliderBoardConfigurationManager.setSlider(1, jointName + CommonNames.q_d, registry,
                 generalizedSDFRobotModel.getJointHolder(jointName).getLowerLimit(), generalizedSDFRobotModel.getJointHolder(jointName).getUpperLimit());
         sliderBoardConfigurationManager.setSlider(2, jointName + CommonNames.qd_d, registry, -9, 9);

         sliderBoardConfigurationManager.saveConfiguration(jointId.toString());
      }


      selectedJoint.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable v)
         {
            System.out.println("loading configuration " + selectedJoint.getEnumValue());
            sliderBoardConfigurationManager.loadConfiguration(selectedJoint.getEnumValue().toString());

            if (remoteSelectedJoint != null)
            {
               remoteSelectedJoint.set(selectedJoint.getEnumValue());
            }
         }
      });
   }

   private void setupSliderBoardForWalking(YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel,
           final SliderBoardConfigurationManager sliderBoardConfigurationManager)
   {
      sliderBoardConfigurationManager.setSlider(1, "percentOfGravityCompensation", registry, 0.0, 1.0);
      sliderBoardConfigurationManager.setSlider(2, "gravityComp_gainScaling", registry, 0.0, 1.0);
      sliderBoardConfigurationManager.setSlider(8, "transitionFactor", registry, 0.0, 1.0);
   }

   private static final SliderBoardFactory turboDriverPositionControlFactory = new SliderBoardFactory()
   {
      @Override
      public void makeSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel)
      {
         new ValkyrieSliderBoard(scs, registry, generalizedSDFRobotModel, ValkyrieSliderBoardType.ON_BOARD_POSITION);
      }
   };

   public static SliderBoardFactory getTurboDriverPositionControlFactory()
   {
      return turboDriverPositionControlFactory;
   }

   private static final SliderBoardFactory forceControlFactory = new SliderBoardFactory()
   {
      @Override
      public void makeSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel)
      {
         new ValkyrieSliderBoard(scs, registry, generalizedSDFRobotModel, ValkyrieSliderBoardType.TORQUE_PD_CONTROL);
      }
   };

   public static SliderBoardFactory getForceControlFactory()
   {
      return forceControlFactory;
   }

   private static final SliderBoardFactory forceControlTuningFactory = new SliderBoardFactory()
   {
      @Override
      public void makeSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel)
      {
         new ValkyrieSliderBoard(scs, registry, generalizedSDFRobotModel, ValkyrieSliderBoardType.TUNING);
      }
   };

   public static SliderBoardFactory getForceTuningControlFactory()
   {
      return forceControlTuningFactory;
   }

   private static final SliderBoardFactory walkingFactory = new SliderBoardFactory()
   {
      @Override
      public void makeSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel)
      {
         new ValkyrieSliderBoard(scs, registry, generalizedSDFRobotModel, ValkyrieSliderBoardType.WALKING);
      }
   };

   // FIXME: Implement this
   public static SliderBoardFactory getDefaultSliderBoardFactory()
   {
      return walkingFactory;
   }

   private static final SliderBoardFactory inverseDynamicsControllerSliderBoardFactory = new SliderBoardFactory()
   {
      @Override
      public void makeSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel)
      {
         SDFFullRobotModelFactory fullRobotModelFactory = new SDFFullRobotModelFactory(generalizedSDFRobotModel, new ValkyrieJointMap());
         SDFFullRobotModel fullRobotModel = fullRobotModelFactory.create();
         new InverseDynamicsJointController.GravityCompensationSliderBoard(scs, fullRobotModel, registry, "transitionFactor", 0.0, 1.0);
      }
   };

   public static SliderBoardFactory getIDControllerSliderBoardFactory()
   {
      return inverseDynamicsControllerSliderBoardFactory;
   }
}
