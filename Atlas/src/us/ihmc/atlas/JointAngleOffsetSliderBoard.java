package us.ihmc.atlas;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.darpaRoboticsChallenge.visualization.SliderBoardFactory;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.inputdevices.SliderBoardConfigurationManager;

public class JointAngleOffsetSliderBoard
{

   public JointAngleOffsetSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel)
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

   private static final SliderBoardFactory factory = new SliderBoardFactory()
   {
      @Override
      public void makeSliderBoard(SimulationConstructionSet scs, YoVariableRegistry registry, GeneralizedSDFRobotModel generalizedSDFRobotModel)
      {
         new JointAngleOffsetSliderBoard(scs, registry, generalizedSDFRobotModel);
      }
   };

   public static SliderBoardFactory getFactory()
   {
      return factory;
   }

}

