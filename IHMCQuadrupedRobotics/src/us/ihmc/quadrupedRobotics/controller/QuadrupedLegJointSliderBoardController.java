package us.ihmc.quadrupedRobotics.controller;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableBoolean;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.quadrupedRobotics.controller.state.QuadrupedControllerState;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
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
   private final Map<String, DoubleYoVariable> sineFrequencyMap = new HashMap<>();
   private final Map<String, DoubleYoVariable> sineAmplitudeMap = new HashMap<>();
   
   private final BooleanYoVariable startChirp = new BooleanYoVariable("startChirp", sliderBoardRegistry);
   private double chirpStartTime = -1;

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
         qDesiredAlpha.set(0.95);
         AlphaFilteredYoVariable alphaFilteredQDesired = new AlphaFilteredYoVariable(key + "_alpha_filtered_q_d", sliderBoardRegistry, qDesiredAlpha, qDesired);
         alphaFilteredQDesiredMap.put(key, alphaFilteredQDesired);
         QDesiredMap.put(key, qDesired);
         jointInitialized.put(key, new MutableBoolean(false));
         
         DoubleYoVariable sineFrequency = new DoubleYoVariable(key + "_freq", registry);
         DoubleYoVariable sineAmplitude = new DoubleYoVariable(key + "_amp", registry);
         
         sineFrequencyMap.put(key, sineFrequency);
         sineAmplitudeMap.put(key, sineAmplitude);
      }
   }

   @Override public RobotMotionStatus getMotionStatus()
   {
      return RobotMotionStatus.STANDING;
   }

   @Override public void doAction()
   {
      if(startChirp.getBooleanValue())
      {
         chirpStartTime = getTimeInCurrentState();
         startChirp.set(false);
      }
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

         double sine = 0.0;
         
         if(chirpStartTime > 1.0)
         {
            double timeInChirp = getTimeInCurrentState() - chirpStartTime;
            if(timeInChirp > 10.0)
            {
               chirpStartTime = -1;
            }
            
            sine = sineAmplitudeMap.get(key).getDoubleValue() * Math.sin(Math.PI * sineFrequencyMap.get(key).getDoubleValue() * timeInChirp * timeInChirp);
         } 
         oneDoFJoint.setqDesired(alphaFilteredYoVariable.getDoubleValue() + sine);
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
