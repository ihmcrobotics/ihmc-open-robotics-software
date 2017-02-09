package us.ihmc.valkyrie.visualizer;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Arrays;

import javax.swing.JComboBox;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizerStateListener;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.FormattingTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.controllers.ValkyrieSliderBoard;
import us.ihmc.valkyrie.controllers.ValkyrieSliderBoard.ValkyrieSliderBoardType;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlLowLevelController;

public class RemoteValkyrieVisualizer implements SCSVisualizerStateListener
{
   public static final int BUFFER_SIZE = 16384;

   private final ValkyrieSliderBoardType valkyrieSliderBoardType;
   private final ValkyrieRobotModel valkyrieRobotModel;

   public RemoteValkyrieVisualizer(ValkyrieSliderBoardType valkyrieSliderBoardType)
   {
      this.valkyrieSliderBoardType = valkyrieSliderBoardType;

      String host = NetworkParameters.getHost(NetworkParameterKeys.logger);
      System.out.println("Connecting to host " + host);
      valkyrieRobotModel = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.REAL_ROBOT, false);

      SCSVisualizer scsVisualizer = new SCSVisualizer(BUFFER_SIZE);
      scsVisualizer.setDisplayOneInNPackets(3);
      scsVisualizer.addSCSVisualizerStateListener(this);
      scsVisualizer.setShowOverheadView(false);

      YoVariableClient client = new YoVariableClient(scsVisualizer, "remote", new ValkyrieIpToNiceNameRemapper());
      client.start();
   }

   @Override
   public void starting(final SimulationConstructionSet scs, Robot robot, YoVariableRegistry registry)
   {
      // TODO The sliderboard throws an NPE when scrubbing, at least in Sim. If this is okay on the real robot then feel free to uncomment. -- Doug
      new ValkyrieSliderBoard(scs, registry, valkyrieRobotModel, valkyrieSliderBoardType);

      final EnumYoVariable<?> requestLowlLevelControlMode = (EnumYoVariable<?>) scs.getVariable(ValkyrieRosControlLowLevelController.class.getSimpleName(), "requestedLowLevelControlMode");

      final String[] enumValuesAsString = requestLowlLevelControlMode.getEnumValuesAsString();
      for (int i =0; i < enumValuesAsString.length; i++)
      {
         enumValuesAsString[i] = enumValuesAsString[i].replaceAll("_", " _");
         enumValuesAsString[i] = FormattingTools.underscoredToCamelCase(enumValuesAsString[i], true);
      }
      final String[] comboBoxValues = new String[enumValuesAsString.length  + 1];
      System.arraycopy(enumValuesAsString, 0, comboBoxValues, 1, enumValuesAsString.length);
      comboBoxValues[0] = "Low-Level Control Mode";
      System.out.println(Arrays.deepToString(enumValuesAsString));
      final JComboBox<String> requestControlModeComboBox = new JComboBox<>(comboBoxValues);
      requestControlModeComboBox.setSelectedIndex(requestLowlLevelControlMode.getOrdinal() % enumValuesAsString.length);
      requestControlModeComboBox.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            int selectedIndex = requestControlModeComboBox.getSelectedIndex() - 1;
            int ordinal = requestLowlLevelControlMode.getOrdinal();
            if (selectedIndex != ordinal)
            {
               requestLowlLevelControlMode.set(selectedIndex);
            }
         }
      });

      requestLowlLevelControlMode.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            int selectedIndex = requestControlModeComboBox.getSelectedIndex();
            int ordinal = requestLowlLevelControlMode.getOrdinal() + 1;
            if (selectedIndex != ordinal)
               requestControlModeComboBox.setSelectedIndex(ordinal);
         }
      });

      scs.addComboBox(requestControlModeComboBox);
   }

   public static void main(String[] args)
   {
      new RemoteValkyrieWalkingVisualizer();
   }
}
