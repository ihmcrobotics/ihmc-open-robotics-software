package us.ihmc.valkyrie.visualizer;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commons.FormattingTools;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizerStateListener;
import us.ihmc.simulationConstructionSetTools.util.inputdevices.SliderBoardConfigurationManager;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

import javax.swing.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.HashMap;
import java.util.Map;

public class ValkyrieImpedanceVisualizer implements SCSVisualizerStateListener
{
   public static final int BUFFER_SIZE = 16384;
   public static final boolean SETUP_SLIDER_BOARD = false;

   public ValkyrieImpedanceVisualizer()
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
         // TODO The sliderboard throws an NPE when scrubbing, at least in Sim. If this is okay on the real robot then feel free to uncomment. -- Doug
         createSliderBoard(scs, registry);
      }

      final YoEnum<?> requestHighLevelControlMode = (YoEnum<?>) scs.findVariable(HighLevelHumanoidControllerFactory.class.getSimpleName(),
                                                                                "requestedHighLevelControllerState");

      HighLevelControllerName[] valuesToDisplay = {HighLevelControllerName.CALIBRATION, HighLevelControllerName.STAND_TRANSITION_STATE, HighLevelControllerName.EXIT_WALKING};
      Map<HighLevelControllerName, String> displayNames = new HashMap<>();
      displayNames.put(HighLevelControllerName.STAND_TRANSITION_STATE, "Go Walking Val!");

      final String[] comboBoxValues = new String[valuesToDisplay.length + 1];
      comboBoxValues[0] = "High-Level Control Mode";

      Map<Integer, Integer> fromComboBoxIndexToEnumOrdinalMap = new HashMap<>();
      fromComboBoxIndexToEnumOrdinalMap.put(0, -1);
      Map<Integer, Integer> fromEnumOrdinalToComboBoxIndexMap = new HashMap<>();
      fromEnumOrdinalToComboBoxIndexMap.put(-1, 0);
      int comboBoxIndex = 1; // Leave the first for a default item
      for (HighLevelControllerName valueToDisplay : valuesToDisplay)
      {
         if (displayNames.containsKey(valueToDisplay))
         {
            comboBoxValues[comboBoxIndex] = displayNames.get(valueToDisplay);
         }
         else
         {
            comboBoxValues[comboBoxIndex] = valueToDisplay.name();
            comboBoxValues[comboBoxIndex] = comboBoxValues[comboBoxIndex].replaceAll("_", " _");
            comboBoxValues[comboBoxIndex] = FormattingTools.underscoredToCamelCase(comboBoxValues[comboBoxIndex], true);
         }

         fromComboBoxIndexToEnumOrdinalMap.put(comboBoxIndex, valueToDisplay.ordinal());
         fromEnumOrdinalToComboBoxIndexMap.put(valueToDisplay.ordinal(), comboBoxIndex);
         comboBoxIndex++;
      }

      final JComboBox<String> requestControlModeComboBox = new JComboBox<>(comboBoxValues);
      requestControlModeComboBox.setSelectedIndex(0);
      requestControlModeComboBox.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            int newOrdinal = fromComboBoxIndexToEnumOrdinalMap.get(requestControlModeComboBox.getSelectedIndex());
            int currentOrdinal = requestHighLevelControlMode.getOrdinal();
            if (newOrdinal != currentOrdinal)
            {
               requestHighLevelControlMode.set(newOrdinal);
            }
         }
      });

      requestHighLevelControlMode.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            try
            {
               int currentIndex = requestControlModeComboBox.getSelectedIndex();
               int newIndex = fromEnumOrdinalToComboBoxIndexMap.get(requestHighLevelControlMode.getOrdinal());
               if (currentIndex != newIndex)
                  requestControlModeComboBox.setSelectedIndex(newIndex);
            }
            catch (NullPointerException e)
            {
            }
         }
      });

      scs.addComboBox(requestControlModeComboBox);
   }

   private void createSliderBoard(final SimulationConstructionSet scs, YoRegistry registry)
   {
      SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);

      sliderBoardConfigurationManager.setSlider(1, "masterGain", registry, 0.0, 1.0);
      sliderBoardConfigurationManager.setSlider(2, "desiredStiffness", registry, 0.0, 140.0);
      sliderBoardConfigurationManager.setSlider(3, "desiredDamping", registry, 0.0, 20.0);

      sliderBoardConfigurationManager.setSlider(5, "qDesired_leftShoulderPitch", registry, -0.7, 0.7);
      sliderBoardConfigurationManager.setSlider(6, "qDesired_leftShoulderRoll", registry, -1.45, -0.9);
      sliderBoardConfigurationManager.setSlider(7, "qDesired_leftShoulderYaw", registry, -1.0, 1.0);
      sliderBoardConfigurationManager.setSlider(8, "qDesired_leftElbowPitch", registry, -0.8, 0.2);
   }

   public static void main(String[] args)
   {
      new ValkyrieImpedanceVisualizer();
   }
}
