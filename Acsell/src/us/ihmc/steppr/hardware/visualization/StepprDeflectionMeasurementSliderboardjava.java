package us.ihmc.steppr.hardware.visualization;

import java.util.EnumMap;

import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.IndexChangedListener;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationConstructionSetTools.util.inputdevices.SliderBoardConfigurationManager;
import us.ihmc.steppr.hardware.StepprDashboard;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.steppr.hardware.controllers.StepprStandPrepSetpoints;

public class StepprDeflectionMeasurementSliderboardjava extends SCSVisualizer implements IndexChangedListener
{
   private final YoVariableRegistry sliderBoardRegistry = new YoVariableRegistry("StepprStandPrepSliderBoard");
   private final EnumYoVariable<StepprStandPrepSetpoints> selectedJointPair = new EnumYoVariable<>("selectedJointPair", sliderBoardRegistry,
         StepprStandPrepSetpoints.class);

   private final DoubleYoVariable selectedJoint_tau_d = new DoubleYoVariable("selectedJoint_tau_d", sliderBoardRegistry);
   private final DoubleYoVariable selectedJoint_damping = new DoubleYoVariable("selectedJoint_damping", sliderBoardRegistry);

   private final EnumMap<StepprStandPrepSetpoints, StandPrepVariables> allSetpoints = new EnumMap<>(StepprStandPrepSetpoints.class);

   public StepprDeflectionMeasurementSliderboardjava(int bufferSize)
   {
      super(bufferSize);
   }

   @Override
   public void starting(SimulationConstructionSet scs, Robot robot, YoVariableRegistry registry)
   {


      registry.addChild(sliderBoardRegistry);
      final SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);

     
      for (StepprStandPrepSetpoints setpoint : StepprStandPrepSetpoints.values)
      {
         StandPrepVariables variables = new StandPrepVariables(setpoint, registry);

         StepprJoint aJoint = setpoint.getJoints()[0];
         sliderBoardConfigurationManager.setKnob(1, selectedJointPair, 0, StepprJoint.values.length);
         sliderBoardConfigurationManager.setSlider(1, variables.tau_d, -100.0, 100.0);
         sliderBoardConfigurationManager.setSlider(3, variables.damping, 0, 5 * aJoint.getRatio() * aJoint.getRatio());
         
         sliderBoardConfigurationManager.saveConfiguration(setpoint.toString());

         allSetpoints.put(setpoint, variables);
      }

      selectedJointPair.addVariableChangedListener(new VariableChangedListener()
      {

         @Override
         public void variableChanged(YoVariable<?> v)
         {
            sliderBoardConfigurationManager.loadConfiguration(selectedJointPair.getEnumValue().toString());
         }
      });

      selectedJointPair.set(StepprStandPrepSetpoints.HIP_Y);

      StepprDashboard.createDashboard(scs, registry);
      scs.getDataBuffer().attachIndexChangedListener(this);
   }

   private class StandPrepVariables
   {
      private final DoubleYoVariable tau_d;
      private final DoubleYoVariable damping;

      public StandPrepVariables(StepprStandPrepSetpoints setpoint, YoVariableHolder variableHolder)
      {
         String prefix = setpoint.getName();
         tau_d = (DoubleYoVariable) variableHolder.getVariable("StepprStandPrep", prefix + "_tau_d");
         damping = (DoubleYoVariable) variableHolder.getVariable("StepprStandPrep", prefix + "_damping");
      }

      public void update()
      {
         selectedJoint_tau_d.set(tau_d.getDoubleValue());
         
         selectedJoint_damping.set(damping.getDoubleValue());
      }
   }

   @Override
   public void indexChanged(int newIndex, double newTime)
   {
      StepprStandPrepSetpoints joint = selectedJointPair.getEnumValue();
      allSetpoints.get(joint).update();
   }

   public static void main(String[] args)
   {
      SCSVisualizer scsYoVariablesUpdatedListener = new StepprDeflectionMeasurementSliderboardjava(64000);
      scsYoVariablesUpdatedListener.setShowOverheadView(false);

      YoVariableClient client = new YoVariableClient(scsYoVariablesUpdatedListener, "remote");
      client.start();
   }
}
