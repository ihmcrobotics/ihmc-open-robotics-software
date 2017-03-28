package us.ihmc.atlas;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationConstructionSetTools.util.inputdevices.SliderBoardConfigurationManager;

public class JointAngleOffsetSliderBoard
{

   public JointAngleOffsetSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry)
   {

        final SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);

        String[] armJointNames = new String[]{"shy","shx", "ely","elx","wry","wrx"};

        for(int i=0;i<armJointNames.length;i++){
         sliderBoardConfigurationManager.setSlider(i+1, "ll_l_arm_"+ armJointNames[i] + "_angleOffsetPreTransmission", registry,
               Math.toRadians(-10.0), Math.toRadians(10.0));
         sliderBoardConfigurationManager.setKnob(i+1, "ll_r_arm_" + armJointNames[i] + "_angleOffsetPreTransmission", registry,
               Math.toRadians(-10.0), Math.toRadians(10.0));
        }

        sliderBoardConfigurationManager.setSlider(8, "neck_ry" + "_angleOffsetPreTransmission", registry,
              Math.toRadians(-10.0), Math.toRadians(10.0));
   }
}

