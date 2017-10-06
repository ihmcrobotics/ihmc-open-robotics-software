package us.ihmc.steppr.hardware.visualization;

import java.util.EnumMap;

import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.yoVariables.dataBuffer.IndexChangedListener;
import us.ihmc.yoVariables.dataBuffer.YoVariableHolder;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationConstructionSetTools.util.inputdevices.SliderBoardConfigurationManager;
import us.ihmc.steppr.hardware.StepprDashboard;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.steppr.hardware.controllers.StepprStandPrepSetpoints;

public class StepprDeflectionMeasurementSliderboardjava extends SCSVisualizer implements IndexChangedListener
{
   private final YoVariableRegistry sliderBoardRegistry = new YoVariableRegistry("StepprStandPrepSliderBoard");
   private final YoEnum<StepprStandPrepSetpoints> selectedJointPair = new YoEnum<>("selectedJointPair", sliderBoardRegistry,
         StepprStandPrepSetpoints.class);

   private final YoDouble selectedJoint_tau_d = new YoDouble("selectedJoint_tau_d", sliderBoardRegistry);
   private final YoDouble selectedJoint_damping = new YoDouble("selectedJoint_damping", sliderBoardRegistry);

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
         public void notifyOfVariableChange(YoVariable<?> v)
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
      private final YoDouble tau_d;
      private final YoDouble damping;

      public StandPrepVariables(StepprStandPrepSetpoints setpoint, YoVariableHolder variableHolder)
      {
         String prefix = setpoint.getName();
         tau_d = (YoDouble) variableHolder.getVariable("StepprStandPrep", prefix + "_tau_d");
         damping = (YoDouble) variableHolder.getVariable("StepprStandPrep", prefix + "_damping");
      }

      public void update()
      {
         selectedJoint_tau_d.set(tau_d.getDoubleValue());
         
         selectedJoint_damping.set(damping.getDoubleValue());
      }
   }

   @Override
   public void notifyOfIndexChange(int newIndex)
   {
      StepprStandPrepSetpoints joint = selectedJointPair.getEnumValue();
      allSetpoints.get(joint).update();
   }

   public static void main(String[] args)
   {
      SCSVisualizer scsYoVariablesUpdatedListener = new StepprDeflectionMeasurementSliderboardjava(64000);
      scsYoVariablesUpdatedListener.setShowOverheadView(false);

      YoVariableClient client = new YoVariableClient(scsYoVariablesUpdatedListener);
      client.start();
   }
}
