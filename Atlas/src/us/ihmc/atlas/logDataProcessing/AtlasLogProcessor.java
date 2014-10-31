package us.ihmc.atlas.logDataProcessing;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.io.IOException;

import javax.swing.JButton;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.robotDataCommunication.logger.YoVariableLogVisualizer;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;

public class AtlasLogProcessor
{
   private FootRotationProcessor footRotationProcessor;
   private SimulationConstructionSet scs;
   
   public AtlasLogProcessor(String[] args)
   {
      final AtlasRobotVersion ATLAS_ROBOT_VERSION = AtlasRobotVersion.DRC_NO_HANDS;
      String[] vars = null;

      DRCRobotModel model = new AtlasRobotModel(ATLAS_ROBOT_VERSION, AtlasRobotModel.AtlasTarget.SIM, false);
      String timeVariableName = "estimatorTime";
      boolean showOverheadView = true;

      File logFile = null;
      if (args.length > 0)
      {
         System.out.println("Arg0:" + args[0]);
         logFile = (new File(args[0]));
         if (!logFile.exists())
         {
            System.err.println("File/folder doesn't exist, quitting");
            return;
         }
         if (logFile.isFile())
            logFile = logFile.getParentFile();

      }

      YoVariableLogVisualizer atlasLogVisualizer = null;
      try
      {
         atlasLogVisualizer = new YoVariableLogVisualizer(model.getGeneralizedRobotModel(), model.getJointMap(), timeVariableName, 32768, showOverheadView, logFile);
      }
      catch (IOException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      
      scs = atlasLogVisualizer.getSimulationConstructionSet();
      footRotationProcessor = new FootRotationProcessor(model, scs);
      footRotationProcessor.getYoGraphicsListRegistry().addArtifactListsToPlotter(atlasLogVisualizer.getPlotter().getPlotter());
      scs.addButton(new UpdateFootRotationCalculatorButton());
      
      int numberOfTicksBeforeUpdatingGraphs = 500;
      scs.setFastSimulate(true, numberOfTicksBeforeUpdatingGraphs);
      
      
      atlasLogVisualizer.run();
   }
   
   public static void main(String[] args) throws IOException
   {
      new AtlasLogProcessor(args);
   }

   private class UpdateFootRotationCalculatorButton extends JButton implements ActionListener
   {
      private static final long serialVersionUID = -112620995090732618L;

      public UpdateFootRotationCalculatorButton()
      {
         super("UpdateDiagnostics");
         this.addActionListener(this);
      }

      @Override
      public void actionPerformed(ActionEvent e)
      {
         scs.applyDataProcessingFunction(footRotationProcessor);
      }
   }
   
}
