package us.ihmc.valkyrie.visualizer;

import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizerStateListener;
import us.ihmc.simulationConstructionSetTools.util.inputdevices.SliderBoardConfigurationManager;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.valkyrieRosControl.impedance.ValkyrieJointList;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

public class ValkyrieImpedanceTuningVisualizer implements SCSVisualizerStateListener
{
   public static final int BUFFER_SIZE = 16384;
   public static final boolean SETUP_SLIDER_BOARD = true;

   public ValkyrieImpedanceTuningVisualizer()
   {
      String host = NetworkParameters.getHost(NetworkParameterKeys.logger);
      System.out.println("Connecting to host " + host);

      SCSVisualizer scsVisualizer = new SCSVisualizer(BUFFER_SIZE);
      scsVisualizer.setVariableUpdateRate(8);
      scsVisualizer.addSCSVisualizerStateListener(this);
      scsVisualizer.setShowOverheadView(true);

      YoVariableClient client = new YoVariableClient(scsVisualizer);
      client.startWithHostSelector();
   }

   @Override
   public void starting(final SimulationConstructionSet scs, Robot robot, YoRegistry registry)
   {
      if (SETUP_SLIDER_BOARD)
      {
         createSliderBoard(scs, registry);
      }
   }

   private void createSliderBoard(SimulationConstructionSet scs, YoRegistry registry)
   {
      YoEnum<ValkyrieJointList.ValkyrieImpedanceJoint> selectedJoint = new YoEnum<>("selectedJoint", registry, ValkyrieJointList.ValkyrieImpedanceJoint.class);

      SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);
      String suffixString = "VITC";

      for (ValkyrieJointList.ValkyrieImpedanceJoint impedanceJoint : ValkyrieJointList.ValkyrieImpedanceJoint.values())
      {
         sliderBoardConfigurationManager.setSlider(1, "masterGain", registry, 0.0, 1.0);
         sliderBoardConfigurationManager.setSlider(3, impedanceJoint.name() + "Stiffness" + suffixString, registry, 0.0, 300.0);
         sliderBoardConfigurationManager.setSlider(4, impedanceJoint.name() + "Damping" + suffixString, registry, 0.0, 50.0);

         sliderBoardConfigurationManager.setSlider(5, impedanceJoint.name() + "Offset" + suffixString, registry, -0.5, 0.5);
         sliderBoardConfigurationManager.setSlider(6, impedanceJoint.name() + "Amp" + suffixString, registry, 0.0, 0.5);
         sliderBoardConfigurationManager.setSlider(7, impedanceJoint.name() + "Freq" + suffixString, registry, 0.2, 2.0);

         sliderBoardConfigurationManager.saveConfiguration(impedanceJoint.name());
         sliderBoardConfigurationManager.clearControls();
      }

      selectedJoint.set(ValkyrieJointList.ValkyrieImpedanceJoint.rightElbowPitch);

      YoVariableChangedListener listener = v ->
      {
         LogTools.info("SliderBoardMode: " + selectedJoint.getEnumValue().toString());
         sliderBoardConfigurationManager.loadConfiguration(selectedJoint.getEnumValue().toString());
      };

      selectedJoint.addListener(listener);
   }

   public static void main(String[] args)
   {
      new ValkyrieImpedanceTuningVisualizer();
   }
}
