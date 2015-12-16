package us.ihmc.quadrupedRobotics.controller;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableBoolean;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;

public class QuadrupedLegJointSliderBoardController extends QuadrupedController
{
   private final YoVariableRegistry sliderBoardRegistry = new YoVariableRegistry(getClass().getSimpleName());

   private final SDFFullRobotModel fullRobotModel;

   private final Map<String, OneDoFJoint> jointMap;
   private final ArrayList<String> jointMapKeySet = new ArrayList<>();

   private final Map<String, AlphaFilteredYoVariable> alphaFilteredQDesiredMap = new HashMap<>();
   private final Map<String, DoubleYoVariable> QDesiredMap = new HashMap<>();
   private final Map<String, MutableBoolean> jointInitialized = new HashMap<>();

   public QuadrupedLegJointSliderBoardController(SDFFullRobotModel fullRobotModel, YoVariableRegistry registry)
   {
      super(QuadrupedControllerState.SLIDER_BOARD);
      this.fullRobotModel = fullRobotModel;
      registry.addChild(sliderBoardRegistry);

      jointMap = fullRobotModel.getOneDoFJointsAsMap();
      jointMapKeySet.addAll(jointMap.keySet());

      for(String key : jointMap.keySet())
      {
         DoubleYoVariable qDesired = new DoubleYoVariable(key + "_q_d", sliderBoardRegistry);
         DoubleYoVariable qDesiredAlpha = new DoubleYoVariable(key + "_q_d_alpha", sliderBoardRegistry);
         qDesiredAlpha.set(0.0);
         AlphaFilteredYoVariable alphaFilteredQDesired = new AlphaFilteredYoVariable(key + "_alpha_filtered_q_d", sliderBoardRegistry, qDesiredAlpha, qDesired);
         alphaFilteredQDesiredMap.put(key, alphaFilteredQDesired);
         QDesiredMap.put(key, qDesired);
         jointInitialized.put(key, new MutableBoolean(false));
      }
   }

   @Override public RobotMotionStatus getMotionStatus()
   {
      return RobotMotionStatus.STANDING;
   }

   @Override public void doAction()
   {
      for (int i = 0; i < jointMapKeySet.size(); i++)
      {
         String key = jointMapKeySet.get(i);
         MutableBoolean initialized = jointInitialized.get(key);
         OneDoFJoint oneDoFJoint = jointMap.get(key);
         AlphaFilteredYoVariable alphaFilteredYoVariable = alphaFilteredQDesiredMap.get(key);
         DoubleYoVariable yoVariable = QDesiredMap.get(key);
         if(initialized.isFalse())
         {
            if(oneDoFJoint.isOnline())
            {
               yoVariable.set(oneDoFJoint.getQ());
               alphaFilteredYoVariable.reset();
               initialized.setValue(true);
            }
         }
         
         alphaFilteredYoVariable.update();
         oneDoFJoint.setqDesired(alphaFilteredYoVariable.getDoubleValue());
      }
   }

   @Override public void doTransitionIntoAction()
   {
      for(OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         joint.setUnderPositionControl(true);
      }
   }

   @Override public void doTransitionOutOfAction()
   {

   }
}
