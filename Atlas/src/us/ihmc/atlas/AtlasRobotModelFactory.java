package us.ihmc.atlas;

import java.awt.BorderLayout;
import java.util.Arrays;

import javax.swing.JComboBox;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

public class AtlasRobotModelFactory
{
   private static String[] AvailableRobotModels = { "ATLAS_NO_HANDS_ADDED_MASS", "ATLAS_SANDIA_HANDS",
         "ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS", "DRC_NO_HANDS", "DRC_HANDS", "DRC_EXTENDED_HANDS", "DRC_HOOKS", "DRC_TASK_HOSE", "DRC_EXTENDED_HOOKS" };

   public static AtlasRobotModel createDRCRobotModel(String robotModelAsString, boolean runningOnRealRobot, boolean headless)
   {
      robotModelAsString = robotModelAsString.toUpperCase().trim();
      try
      {
         AtlasRobotVersion atlasRobotVersion = AtlasRobotVersion.valueOf(robotModelAsString);
         if (atlasRobotVersion != null)
         {
            return new AtlasRobotModel(atlasRobotVersion, runningOnRealRobot, headless);
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
      throw new IllegalArgumentException(robotModelAsString + " Not a valid robot model");
   }

   public static String robotModelsToString()
   {
      return Arrays.toString(AvailableRobotModels);
   }

   public static String[] getAvailableRobotModels()
   {
      return AvailableRobotModels;
   }

   public static int getOrdinalOfModel(String st)
   {
      st = st.toUpperCase();
      for (int i = 0; i < AvailableRobotModels.length; i++)
      {
         if (st == AvailableRobotModels[i].toUpperCase())
         {
            return i;
         }
      }
      return -1;
   }

   public static AtlasRobotModel selectModelFromGraphicSelector(DRCRobotModel defaultOption)
   {
      JPanel userPromptPanel = new JPanel(new BorderLayout());
      JPanel comboBoxPanel = new JPanel(new BorderLayout());

      JLabel userMessageLabel = new JLabel("What robot?");

      @SuppressWarnings({ "rawtypes", "unchecked" })
      JComboBox robotTypeComboBox = new JComboBox(AvailableRobotModels);
      robotTypeComboBox.setSelectedItem(defaultOption);

      comboBoxPanel.add(robotTypeComboBox, BorderLayout.NORTH);

      userPromptPanel.add(userMessageLabel, BorderLayout.NORTH);
      userPromptPanel.add(comboBoxPanel, BorderLayout.CENTER);

      int selectedOption = JOptionPane.showOptionDialog(null, userPromptPanel, "Select", JOptionPane.OK_CANCEL_OPTION, JOptionPane.QUESTION_MESSAGE, null,
            null, null);

      if (selectedOption == JOptionPane.CANCEL_OPTION)
      {
         System.err.println("Operation canceled by the user.");
         return null;
      }
      else if (selectedOption == JOptionPane.OK_OPTION)
      {
         String groundTypeString = robotTypeComboBox.getSelectedItem().toString();
         AtlasRobotModel model = createDRCRobotModel(groundTypeString, false, false);
         return model;
      }
      else
      {
         throw new RuntimeException("Ooops!");
      }
   }

   public static AtlasRobotModel selectModelFromFlag(String[] args, boolean runningOnRealRobot, boolean headless)
   {
      // Add flag to set robot model
      JSAP jsap = new JSAP();
      FlaggedOption robotModel = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true).setStringParser(JSAP.STRING_PARSER);
      robotModel.setHelp("Robot models: " + Arrays.toString(AvailableRobotModels));
      try
      {
         jsap.registerParameter(robotModel);

         JSAPResult config = jsap.parse(args);

         if (config.success())
            return createDRCRobotModel(config.getString("robotModel"), runningOnRealRobot, headless);
      }
      catch (JSAPException e)
      {
         e.printStackTrace();
      }
      return null;
   }
}
