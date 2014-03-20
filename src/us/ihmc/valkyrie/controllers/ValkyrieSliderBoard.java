package us.ihmc.valkyrie.controllers;

import com.yobotics.simulationconstructionset.*;
import com.yobotics.simulationconstructionset.util.inputdevices.SliderBoardConfigurationManager;
import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.atlas.visualization.SliderBoardFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.CommonNames;

/**
 * Created by dstephen on 2/28/14.
 */
public class ValkyrieSliderBoard
{
   private enum ValkyrieSliderBoardType
   {
      ON_BOARD_POSITION, TORQUE_PD_CONTROL, WALKING
   }
   private final EnumYoVariable<ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints> selectedJoint, remoteSelectedJoint;

   @SuppressWarnings("unchecked")
   public ValkyrieSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel,
                              ValkyrieSliderBoardType sliderBoardType)
   {
      selectedJoint = new EnumYoVariable<>("selectedJoint", registry, ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints.class);
      selectedJoint.set(ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints.RightKneeExtensor);
      remoteSelectedJoint = (EnumYoVariable<ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints>) registry.getVariable(
         "remoteroot.ValkyrieSliderBoardController.selectedJoint");

      final SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);

      switch(sliderBoardType)
      {
         case ON_BOARD_POSITION:
            setupSliderBoardForOnBoardPositionControl(registry, generalizedSDFRobotModel, sliderBoardConfigurationManager);
            break;
         case TORQUE_PD_CONTROL:
            setupSliderBoardForForceControl(registry, generalizedSDFRobotModel, sliderBoardConfigurationManager);
            break;
         case WALKING:
            setupSliderBoardForWalking(registry, generalizedSDFRobotModel, sliderBoardConfigurationManager);
            break;
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
         if(!jointName.contains("Ibeo"))
         {
            String pdControllerBaseName = jointName + "ValkyrieJointPDController";

            // knobs

            sliderBoardConfigurationManager.setKnob(1, selectedJoint, 0, ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints.values().length - 1);

            // sliders
            sliderBoardConfigurationManager.setSlider(1, pdControllerBaseName + "_q_d", registry,
                    generalizedSDFRobotModel.getJointHolder(jointName).getLowerLimit(), generalizedSDFRobotModel.getJointHolder(jointName).getUpperLimit());
            sliderBoardConfigurationManager.setSlider(2, "kp_" + pdControllerBaseName, registry, 0.0, 60.0);
            sliderBoardConfigurationManager.setSlider(3, "kd_" + pdControllerBaseName, registry, 0.0, 60.0);

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

   private static final SliderBoardFactory walkingFactory = new SliderBoardFactory()
   {
      @Override
      public void makeSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel)
      {
         new ValkyrieSliderBoard(scs, registry, generalizedSDFRobotModel, ValkyrieSliderBoardType.WALKING);
      }
   };

   //FIXME: Implement this
   public static SliderBoardFactory getDefaultSliderBoardFactory()
   {
      return walkingFactory;
   }
}
