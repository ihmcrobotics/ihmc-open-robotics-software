package us.ihmc.quadrupedRobotics.controller.positionDevelopment.states;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class QuadrupedPositionBasedLegJointSliderBoardController implements QuadrupedController
{
   private final YoVariableRegistry sliderBoardRegistry = new YoVariableRegistry(getClass().getSimpleName());

   private final Map<String, OneDoFJoint> jointMap;
   private final ArrayList<String> jointMapKeySet = new ArrayList<>();

   private final Map<String, AlphaFilteredYoVariable> alphaFilteredQDesiredMap = new HashMap<>();
   private final Map<String, YoDouble> QDesiredMap = new HashMap<>();

   private final JointDesiredOutputList jointDesiredOutputList;

   public QuadrupedPositionBasedLegJointSliderBoardController(QuadrupedRuntimeEnvironment runtimeEnvironment, YoVariableRegistry parentRegistry)
   {
      this.jointDesiredOutputList = runtimeEnvironment.getJointDesiredOutputList();

      jointMap = runtimeEnvironment.getFullRobotModel().getOneDoFJointsAsMap();
      jointMapKeySet.addAll(jointMap.keySet());

      for (String key : jointMap.keySet())
      {
         YoDouble qDesired = new YoDouble(key + "_q_d", sliderBoardRegistry);
         YoDouble qDesiredAlpha = new YoDouble(key + "_q_d_alpha", sliderBoardRegistry);
         qDesiredAlpha.set(0.99);
         AlphaFilteredYoVariable alphaFilteredQDesired = new AlphaFilteredYoVariable(key + "_alpha_filtered_q_d", sliderBoardRegistry, qDesiredAlpha, qDesired);
         alphaFilteredQDesiredMap.put(key, alphaFilteredQDesired);
         QDesiredMap.put(key, qDesired);

      }
      parentRegistry.addChild(sliderBoardRegistry);
   }

   @Override
   public ControllerEvent process()
   {
      for (int i = 0; i < jointMapKeySet.size(); i++)
      {
         String key = jointMapKeySet.get(i);
         OneDoFJoint oneDoFJoint = jointMap.get(key);
         AlphaFilteredYoVariable alphaFilteredYoVariable = alphaFilteredQDesiredMap.get(key);
         alphaFilteredYoVariable.update();
         jointDesiredOutputList.getJointDesiredOutput(oneDoFJoint).setDesiredPosition(alphaFilteredYoVariable.getDoubleValue());
      }
      return null;
   }

   @Override
   public void onEntry()
   {
      for (int i = 0; i < jointMapKeySet.size(); i++)
      {
         String key = jointMapKeySet.get(i);
         OneDoFJoint joint = jointMap.get(key);

         YoDouble desiredQ = QDesiredMap.get(key);
         AlphaFilteredYoVariable alphaFilteredYoVariable = alphaFilteredQDesiredMap.get(key);

         desiredQ.set(joint.getQ());
         alphaFilteredYoVariable.reset();
         alphaFilteredYoVariable.update();
         
         joint.setUnderPositionControl(true);
      }
   }

   @Override
   public void onExit()
   {

   }
}
