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
   private final EnumYoVariable<ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints> selectedJoint, remoteSelectedJoint;

   @SuppressWarnings("unchecked")
   public ValkyrieSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel,
                              boolean forceControl)
   {
      selectedJoint = new EnumYoVariable<>("selectedJoint", registry, ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints.class);
      selectedJoint.set(ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints.RightKneeExtensor);
      remoteSelectedJoint = (EnumYoVariable<ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints>) registry.getVariable(
         "remoteroot.ValkyrieSliderBoardController.selectedJoint");

      final SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);

      if (!forceControl)
      {
         setupSliderBoardForOnBoardPositionControl(registry, generalizedSDFRobotModel, sliderBoardConfigurationManager);
      }
      else
      {
         setupSliderBoardForForceControl(registry, generalizedSDFRobotModel, sliderBoardConfigurationManager);
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
         String pdControllerBaseName = jointName + "ValkyrieJointPDController";

         // knobs

         sliderBoardConfigurationManager.setKnob(1, selectedJoint, 0, ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints.values().length - 1);

         // sliders
         sliderBoardConfigurationManager.setSlider(1, pdControllerBaseName + "_q_d", registry,
                 generalizedSDFRobotModel.getJointHolder(jointName).getLowerLimit(), generalizedSDFRobotModel.getJointHolder(jointName).getUpperLimit());
         sliderBoardConfigurationManager.setSlider(2, "kp_" + pdControllerBaseName, registry, 0.0, 200.0);
         sliderBoardConfigurationManager.setSlider(3, "kd_" + pdControllerBaseName, registry, 0.0, 200.0);

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

   private void setupSliderBoardForOnBoardPositionControl(YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel,
           final SliderBoardConfigurationManager sliderBoardConfigurationManager)
   {
      for (ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints jointId :
              ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints.values())
      {
         String jointName = jointId.toString();

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

   private static final SliderBoardFactory turboDriverPositionControlFactory = new SliderBoardFactory()
   {
      @Override
      public void makeSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel)
      {
         new ValkyrieSliderBoard(scs, registry, generalizedSDFRobotModel, false);
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
         new ValkyrieSliderBoard(scs, registry, generalizedSDFRobotModel, true);
      }
   };

   public static SliderBoardFactory getForceControlFactory()
   {
      return forceControlFactory;
   }
}
