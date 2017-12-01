package us.ihmc.valkyrie.visualizer;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Arrays;

import javax.swing.JComboBox;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commons.FormattingTools;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizerStateListener;
import us.ihmc.simulationConstructionSetTools.util.inputdevices.SliderBoardConfigurationManager;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

public class RemoteValkyrieVisualizer implements SCSVisualizerStateListener
{
   public static final int BUFFER_SIZE = 16384;

   public RemoteValkyrieVisualizer()
   {
      String host = NetworkParameters.getHost(NetworkParameterKeys.logger);
      System.out.println("Connecting to host " + host);

      SCSVisualizer scsVisualizer = new SCSVisualizer(BUFFER_SIZE);
      scsVisualizer.setDisplayOneInNPackets(5);
      scsVisualizer.addSCSVisualizerStateListener(this);
      scsVisualizer.setShowOverheadView(true);

      YoVariableClient client = new YoVariableClient(scsVisualizer);
      client.start();
   }

   @Override
   public void starting(final SimulationConstructionSet scs, Robot robot, YoVariableRegistry registry)
   {
      // TODO The sliderboard throws an NPE when scrubbing, at least in Sim. If this is okay on the real robot then feel free to uncomment. -- Doug
      createSliderBoard(scs, registry);

      final YoEnum<?> requestHighLevelControlMode = (YoEnum<?>) scs.getVariable(HighLevelHumanoidControllerFactory.class.getSimpleName(),
                                                                                "requestedHighLevelControllerState");

      final String[] enumValuesAsString = requestHighLevelControlMode.getEnumValuesAsString();
      for (int i = 0; i < enumValuesAsString.length; i++)
      {
         enumValuesAsString[i] = enumValuesAsString[i].replaceAll("_", " _");
         enumValuesAsString[i] = FormattingTools.underscoredToCamelCase(enumValuesAsString[i], true);
      }
      final String[] comboBoxValues = new String[enumValuesAsString.length + 1];
      System.arraycopy(enumValuesAsString, 0, comboBoxValues, 1, enumValuesAsString.length);
      comboBoxValues[0] = "High-Level Control Mode";
      System.out.println(Arrays.deepToString(enumValuesAsString));
      final JComboBox<String> requestControlModeComboBox = new JComboBox<>(comboBoxValues);
      requestControlModeComboBox.setSelectedIndex(requestHighLevelControlMode.getOrdinal() % enumValuesAsString.length);
      requestControlModeComboBox.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            int selectedIndex = requestControlModeComboBox.getSelectedIndex() - 1;
            int ordinal = requestHighLevelControlMode.getOrdinal();
            if (selectedIndex != ordinal)
            {
               requestHighLevelControlMode.set(selectedIndex);
            }
         }
      });

      requestHighLevelControlMode.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            int selectedIndex = requestControlModeComboBox.getSelectedIndex();
            int ordinal = requestHighLevelControlMode.getOrdinal() + 1;
            if (selectedIndex != ordinal)
               requestControlModeComboBox.setSelectedIndex(ordinal);
         }
      });

      scs.addComboBox(requestControlModeComboBox);
   }

   private void createSliderBoard(final SimulationConstructionSet scs, YoVariableRegistry registry)
   {
      SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);
      sliderBoardConfigurationManager.setButton(1, registry.getVariable("PelvisICPBasedTranslationManager", "manualModeICPOffset"));
      sliderBoardConfigurationManager.setSlider(1, "desiredICPOffsetX", registry, -0.3, 0.3);
      sliderBoardConfigurationManager.setSlider(2, "desiredICPOffsetY", registry, -0.3, 0.3);
      sliderBoardConfigurationManager.setSlider(8, "offsetHeightAboveGround", registry, 0.0, 0.20);
   }

   public static void main(String[] args)
   {
      new RemoteValkyrieVisualizer();
   }
}
