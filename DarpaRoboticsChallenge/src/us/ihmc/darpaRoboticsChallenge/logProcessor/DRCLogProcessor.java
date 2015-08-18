package us.ihmc.darpaRoboticsChallenge.logProcessor;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;

import javax.swing.JButton;

import us.ihmc.SdfLoader.SDFBaseRobot;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.plotting.Plotter;
import us.ihmc.robotDataCommunication.logger.LogVisualizer;
import us.ihmc.simulationconstructionset.DataProcessingFunction;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public abstract class DRCLogProcessor
{
   private final LogVisualizer logVisualizer;
   private final Plotter plotter;

   private final LogDataProcessorWrapper logDataProcessorWrapper;
   private final SDFBaseRobot sdfRobot;

   protected final SimulationConstructionSet scs;
   protected final DRCRobotModel drcRobotModel;

   public DRCLogProcessor() throws IOException
   {
      drcRobotModel = createDRCRobotModel();
      logVisualizer = new LogVisualizer();

      scs = logVisualizer.getSimulationConstructionSet();
      plotter = logVisualizer.getPlotter();

      int numberOfTicksBeforeUpdatingGraphs = 500;
      scs.setFastSimulate(true, numberOfTicksBeforeUpdatingGraphs);

      logDataProcessorWrapper = new LogDataProcessorWrapper(scs);
      sdfRobot = logVisualizer.getSDFRobot();

      scs.addButton(new UpdateLogDataProcessorButton(logDataProcessorWrapper));
   }

   public LogDataProcessorHelper createLogDataProcessorHelper()
   {
      return new LogDataProcessorHelper(drcRobotModel, scs, sdfRobot);
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
