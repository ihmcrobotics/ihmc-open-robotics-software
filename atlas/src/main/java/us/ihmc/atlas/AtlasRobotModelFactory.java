package us.ihmc.atlas;

import java.awt.BorderLayout;
import java.util.Arrays;

import javax.swing.JComboBox;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.FootContactPoints;

public class AtlasRobotModelFactory
{
   private static final String[] AVAILABLE_ROBOT_MODELS = new String[AtlasRobotVersion.values().length];
   static
   {
      for (AtlasRobotVersion version : AtlasRobotVersion.values())
      {
         AVAILABLE_ROBOT_MODELS[version.ordinal()] = version.toString();
      }
   }

   public static AtlasRobotModel createDefaultRobotModel()
   {
      return createDRCRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ.toString(), RobotTarget.SCS, false, null);
   }

   public static AtlasRobotModel createDefaultRobotModel(FootContactPoints<RobotSide> simulationContactPoints)
   {
      return createDRCRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ.toString(), RobotTarget.SCS, false, simulationContactPoints);
   }

   public static AtlasRobotModel createDefaultRobotModel(AtlasRobotVersion robotVersion)
   {
      return createDRCRobotModel(robotVersion.toString(), RobotTarget.SCS, false, null);
   }

   public static AtlasRobotModel createDefaultRobotModel(AtlasRobotVersion robotVersion, FootContactPoints<RobotSide> simulationContactPoints)
   {
      return createDRCRobotModel(robotVersion.toString(), RobotTarget.SCS, false, simulationContactPoints);
   }

   public static AtlasRobotModel createDRCRobotModel(String robotModelAsString, RobotTarget runningOnRealRobot, boolean headless)
   {
      return createDRCRobotModel(robotModelAsString, runningOnRealRobot, headless, null);
   }

   public static AtlasRobotModel createDRCRobotModel(String robotModelAsString, RobotTarget runningOnRealRobot, boolean headless,
         FootContactPoints<RobotSide> simulationContactPoints)
   {
      robotModelAsString = robotModelAsString.toUpperCase().trim();
      try
      {
         AtlasRobotVersion atlasRobotVersion = AtlasRobotVersion.valueOf(robotModelAsString);
         if (atlasRobotVersion != null)
         {
            return new AtlasRobotModel(atlasRobotVersion, runningOnRealRobot, headless, simulationContactPoints);
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
      return Arrays.toString(AVAILABLE_ROBOT_MODELS);
   }

   public static String[] getAvailableRobotModels()
   {
      return AVAILABLE_ROBOT_MODELS;
   }

   public static int getOrdinalOfModel(String st)
   {
      st = st.toUpperCase();
      for (int i = 0; i < AVAILABLE_ROBOT_MODELS.length; i++)
      {
         if (st.equals( AVAILABLE_ROBOT_MODELS[i].toUpperCase() ) )
         {
            return i;
         }
      }
      return -1;
   }

   public static AtlasRobotModel selectModelFromGraphicSelector()
   {
      return selectModelFromGraphicSelector(new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false));
   }

   public static AtlasRobotModel selectModelFromGraphicSelector(DRCRobotModel defaultOption)
   {
      JPanel userPromptPanel = new JPanel(new BorderLayout());
      JPanel comboBoxPanel = new JPanel(new BorderLayout());

      JLabel userMessageLabel = new JLabel("What robot?");

      JComboBox<String> robotTypeComboBox = new JComboBox<>(AVAILABLE_ROBOT_MODELS);
      robotTypeComboBox.setSelectedItem(defaultOption.toString());

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
         AtlasRobotModel model = createDRCRobotModel(groundTypeString, RobotTarget.SCS, false);
         return model;
      }
      else
      {
         throw new RuntimeException("Ooops!");
      }
   }

   public static AtlasRobotModel selectSimulationModelFromFlag(String[] args)
   {
      // Add flag to set robot model
      JSAP jsap = new JSAP();
      FlaggedOption robotModel = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true).setStringParser(JSAP.STRING_PARSER);
      robotModel.setHelp("Robot models: " + Arrays.toString(AVAILABLE_ROBOT_MODELS));
      try
      {
         jsap.registerParameter(robotModel);

         JSAPResult config = jsap.parse(args);

         if (config.success())
            return createDRCRobotModel(config.getString("robotModel"), RobotTarget.SCS, false);
      }
      catch (JSAPException e)
      {
         e.printStackTrace();
      }
      return null;
   }
}
