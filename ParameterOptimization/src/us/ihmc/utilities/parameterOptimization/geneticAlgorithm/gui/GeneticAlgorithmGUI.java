package us.ihmc.utilities.parameterOptimization.geneticAlgorithm.gui;

import java.awt.Container;
import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.Toolkit;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSplitPane;
import javax.swing.border.TitledBorder;

import us.ihmc.utilities.parameterOptimization.geneticAlgorithm.GeneticAlgorithm;
import us.ihmc.utilities.parameterOptimization.geneticAlgorithm.GeneticAlgorithmChangedListener;
import us.ihmc.utilities.parameterOptimization.geneticAlgorithm.GeneticAlgorithmIndividualToEvaluate;
import us.ihmc.utilities.parameterOptimization.geneticAlgorithm.Population;


/**
 * <p>Title: Genetic Algorithm Library </p>
 *
 * <p>Description: General Purpose Genetic Algorithm Library </p>
 *
 * <p>Copyright: Copyright (c) 2003-2005 Jerry Pratt, IHMC </p>
 *
 * <p>Company: Institute for Human and Machine Cognition.
 * 40 South Alcaniz Street
 * Pensacola, FL 32502 </p>
 *
 * @author Jerry Pratt and Jim Warrenfeltz, jpratt@ihmc.us
 * @version 1.0
 */


public class GeneticAlgorithmGUI implements GeneticAlgorithmChangedListener, SelectedPopulationHolder, SelectedIndividualHolder
{
   private JFrame frame;

   private Population selectedPopulation;
   private GeneticAlgorithmIndividualToEvaluate selectedIndividual;

   private Container contentPane;

   private GeneticAlgorithmSummaryPanel gaSummaryPanel;
   private JScrollPane gaSummaryScrollPane;

   private PopulationSummaryPanel populationSummaryPanel;
   private JScrollPane populationSummaryScrollPane;

   private GeneticAlgorithmStatisticsPanel gaStatisticsPanel;
   private JScrollPane gaStatisticsScrollPane;

   private GeneticAlgorithmProgressPanel geneticAlgorithmProgressPanel;

   private IndividualSummaryPanel individualSummaryPanel;
   private JScrollPane individualSummaryScrollPane;

// private JPanel numericContentPane;
   protected JPanel buttonPanel;

   // protected EntryBoxArrayPanel myEntryBoxArrayPanel;
   // protected GraphArrayPanel myGraphArrayPanel;


   // private ExportDataAction exportAction;
   // private ImportDataAction importDataAction;
   // private ExportSnapshotAction exportSnapshotAction;
   // private ExportVideoAction exportVideoAction;
// private JMenuItem exitMenuItem;

   // private SimulateAction simulateAction;
// private GeneticAlgorithm ga;
   public GeneticAlgorithmGUI(GeneticAlgorithm ga)
   {
//    this.ga = ga;
      frame = new JFrame("Genetic Algorithm");
      contentPane = frame.getContentPane();

      // contentPane.setLayout(new BorderLayout());
      contentPane.setLayout(new GridLayout(2, 2));

      geneticAlgorithmProgressPanel = new GeneticAlgorithmProgressPanel(ga);
      contentPane.add(geneticAlgorithmProgressPanel);

      individualSummaryPanel = new IndividualSummaryPanel(this);
      individualSummaryScrollPane = new JScrollPane(individualSummaryPanel, JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED,
              JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
      individualSummaryScrollPane.setBorder(new TitledBorder("Individual Summary"));

      gaStatisticsPanel = new GeneticAlgorithmStatisticsPanel(ga, this, this);
      gaStatisticsScrollPane = new JScrollPane(gaStatisticsPanel, JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED, JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
      gaStatisticsScrollPane.setBorder(new TitledBorder("Statistics"));

      JSplitPane panel = new JSplitPane(JSplitPane.VERTICAL_SPLIT, gaStatisticsPanel, individualSummaryPanel);
      contentPane.add(panel);


      gaSummaryPanel = new GeneticAlgorithmSummaryPanel(ga, this);
      gaSummaryScrollPane = new JScrollPane(gaSummaryPanel, JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED, JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);

      // gaSummaryScrollPane.getVerticalScrollBar().setUnitIncrement(SCROLL_PANE_INCREMENT);
      // gaSummaryScrollPane.getVerticalScrollBar().setBlockIncrement(SCROLL_PANE_INCREMENT);
      gaSummaryScrollPane.setBorder(new TitledBorder(ga.getName() + " Populations"));

      // contentPane.add("West", gaSummaryScrollPane);
      contentPane.add(gaSummaryScrollPane);

      populationSummaryPanel = new PopulationSummaryPanel(ga, this, this);
      populationSummaryScrollPane = new JScrollPane(populationSummaryPanel, JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED,
              JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
      populationSummaryScrollPane.setBorder(new TitledBorder("Individuals"));

      // contentPane.add("Center", populationSummaryScrollPane);
      contentPane.add(populationSummaryScrollPane);


      gaSummaryPanel.resetSelected();

      ga.addGeneticAlgorithmChangedListener(this);

      // frame.pack();
      // frame.show();

      Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();

      frame.setSize(screenSize.width * 7 / 8, screenSize.height * 7 / 8);

      // frame.setSize(screenSize.width*1/8, screenSize.height*1/8);
      frame.setLocation(screenSize.width / 16, screenSize.height / 16);
      frame.validate();

      // frame.pack();
      frame.setVisible(true);

      // frame.show();
   }

   public void geneticAlgorithmChanged()
   {
      // gaSummaryPanel.updateUI();
      // gaSummaryScrollPane.updateUI();

      gaSummaryPanel.updatePanelSize();
      gaSummaryPanel.repaint();

      populationSummaryPanel.repaint();
      gaStatisticsPanel.repaint();
      geneticAlgorithmProgressPanel.repaint();

      individualSummaryPanel.individualChanged();

      // System.out.println("GeneticAlgorithm Changed");
   }

   public Population getSelectedPopulation()
   {
      return selectedPopulation;
   }

   public void setSelectedPopulation(Population pop)
   {
      this.selectedPopulation = pop;
      populationSummaryPanel.resetSelected();

      individualSummaryPanel.individualChanged();

      populationSummaryPanel.repaint();
      gaStatisticsPanel.repaint();
   }

   public GeneticAlgorithmIndividualToEvaluate getSelectedIndividual()
   {
      return selectedIndividual;
   }

   public void setSelectedIndividual(GeneticAlgorithmIndividualToEvaluate ind)
   {
      this.selectedIndividual = ind;
      individualSummaryPanel.individualChanged();
      gaStatisticsPanel.repaint();
   }


}
