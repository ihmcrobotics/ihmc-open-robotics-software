package us.ihmc.valkyrie.controllers;

import com.yobotics.simulationconstructionset.*;
import com.yobotics.simulationconstructionset.util.inputdevices.SliderBoardConfigurationManager;
import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.atlas.visualization.SliderBoardFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.CommonNames;

import java.util.EnumMap;
import java.util.HashMap;

/**
 * Created by dstephen on 2/28/14.
 */
public class ValkyrieSliderBoard
{
   private final EnumYoVariable<ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints> selectedJoint, remoteSelectedJoint;

   @SuppressWarnings("unchecked")
   public ValkyrieSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel)
   {
      selectedJoint = new EnumYoVariable<>("selectedJoint", registry, ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints.class);
      selectedJoint.set(ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints.RightKneeExtensor);
      remoteSelectedJoint = (EnumYoVariable<ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints>) registry.getVariable(
         "remoteroot.ValkyrieSliderBoardController.selectedJoint");

      final SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);


      for (ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints jointId :
              ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints.values())
      {
         String jointName = jointId.toString();

         // knobs

         sliderBoardConfigurationManager.setKnob(1, selectedJoint, 0, ValkyrieSliderBoardController.ValkyrieSliderBoardSelectableJoints.values().length - 1);

         // sliders
         sliderBoardConfigurationManager.setSlider(1, jointName + CommonNames.q_d, registry, generalizedSDFRobotModel.getJointHolder(jointName).getLowerLimit(), generalizedSDFRobotModel.getJointHolder(jointName).getUpperLimit());
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

      sliderBoardConfigurationManager.loadConfiguration(selectedJoint.getEnumValue().toString());
   }

   private static final SliderBoardFactory factory = new SliderBoardFactory()
   {
      @Override
      public void makeSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel)
      {
         new ValkyrieSliderBoard(scs, registry, generalizedSDFRobotModel);
      }
   };

   public static SliderBoardFactory getFactory()
   {
      return factory;
   }
}
