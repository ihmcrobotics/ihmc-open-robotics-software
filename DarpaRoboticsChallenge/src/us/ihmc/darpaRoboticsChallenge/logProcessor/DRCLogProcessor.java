package us.ihmc.darpaRoboticsChallenge.logProcessor;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;

import javax.swing.JButton;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.plotting.Plotter;
import us.ihmc.robotDataCommunication.logger.YoVariableLogVisualizer;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.DataProcessingFunction;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public abstract class DRCLogProcessor
{
   private final YoVariableLogVisualizer logVisualizer;
   private final SimulationConstructionSet scs;
   private final Plotter plotter;

   private final DRCRobotModel drcRobotModel;
   private final LogDataProcessorWrapper logDataProcessorWrapper;

   protected final LogDataProcessorHelper logDataProcessorHelper;

   public DRCLogProcessor() throws IOException
   {
      String timeVariableName = "estimatorTime";

      drcRobotModel = createDRCRobotModel();
      GeneralizedSDFRobotModel generalizedRobotModel = drcRobotModel.getGeneralizedRobotModel();
      DRCRobotJointMap jointMap = drcRobotModel.getJointMap();
      logVisualizer = new YoVariableLogVisualizer(generalizedRobotModel, jointMap, timeVariableName);

      scs = logVisualizer.getSimulationConstructionSet();
      plotter = logVisualizer.getPlotter();

      int numberOfTicksBeforeUpdatingGraphs = 500;
      scs.setFastSimulate(true, numberOfTicksBeforeUpdatingGraphs);

      logDataProcessorWrapper = new LogDataProcessorWrapper(scs);
      SDFRobot sdfRobot = logVisualizer.getSDFRobot();
      logDataProcessorHelper = new LogDataProcessorHelper(drcRobotModel, scs, sdfRobot);

      scs.addButton(new UpdateLogDataProcessorButton(logDataProcessorWrapper));
   }

   public abstract DRCRobotModel createDRCRobotModel();

   public final void setLogDataProcessor(LogDataProcessorFunction logDataProcessor)
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = logDataProcessor.getYoGraphicsListRegistry();
      if (yoGraphicsListRegistry != null)
      {
         yoGraphicsListRegistry.addArtifactListsToPlotter(plotter);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      }

      logDataProcessorWrapper.addLogDataProcessor(logDataProcessor);
   }

   public final void startLogger()
   {
      logVisualizer.run();
   }

   private class UpdateLogDataProcessorButton extends JButton implements ActionListener
   {
      private static final long serialVersionUID = -112620995090732618L;
      private final DataProcessingFunction dataProcessingFunction;

      public UpdateLogDataProcessorButton(DataProcessingFunction dataProcessingFunction)
      {
         super("Rerun log data processors");
         this.addActionListener(this);
         this.dataProcessingFunction = dataProcessingFunction;
      }

      @Override
      public void actionPerformed(ActionEvent e)
      {
         scs.applyDataProcessingFunction(dataProcessingFunction);
      }
   }
}
