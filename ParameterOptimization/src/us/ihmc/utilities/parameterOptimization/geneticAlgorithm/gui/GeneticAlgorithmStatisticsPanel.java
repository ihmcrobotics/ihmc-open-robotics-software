package us.ihmc.utilities.parameterOptimization.geneticAlgorithm.gui;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;

import javax.swing.JPanel;

import us.ihmc.utilities.parameterOptimization.geneticAlgorithm.GeneticAlgorithm;
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

public class GeneticAlgorithmStatisticsPanel extends JPanel
{
   private static final long serialVersionUID = -1988659103401089927L;    // Since Serializable
   private final static int WIDTH = 200;
   private final static int FONT_SIZE = 12;

// private int selected = -1;

   public final static java.text.NumberFormat format = new java.text.DecimalFormat(" 0.00000;-0.00000");
   Dimension sz = new Dimension(WIDTH, 12);

// private Color viewcolor = new Color(0xd0, 0xff, 0xff);
// private Color selectcolor = new Color(0x60, 0x00, 0x00);
// private Thread anim;

// private GeneticAlgorithm ga;
   private SelectedPopulationHolder selectedPopulationHolder;
   private SelectedIndividualHolder selectedIndividualHolder;

   public GeneticAlgorithmStatisticsPanel(GeneticAlgorithm ga, SelectedPopulationHolder popHolder, SelectedIndividualHolder indHolder)
   {
      super();

//    this.ga = ga;
      this.selectedPopulationHolder = popHolder;
      this.selectedIndividualHolder = indHolder;

      updatePanelSize();
   }

   public void updatePanelSize()
   {
      sz.setSize(WIDTH, FONT_SIZE * 20);
      this.setPreferredSize(sz);
      this.updateUI();
   }

   public void paintComponent(Graphics g)
   {
      super.paintComponent(g);

      Population pop = selectedPopulationHolder.getSelectedPopulation();
      if (pop == null)
         return;

      g.setColor(Color.black);

      if (pop.allIndividualsEvaluated())
      {
         double maxFitness = pop.getMaximumFitness();
         double aveFitness = pop.getAverageFitness();
         double minFitness = pop.getMinimumFitness();

         g.drawString("Selected Population: ", 10, 3 * FONT_SIZE + 10);
         g.drawString("  Max Fitness: ", 10, 4 * FONT_SIZE + 10);
         g.drawString(Double.toString(maxFitness), 200, 4 * FONT_SIZE + 10);

         g.drawString("  Average Fitness: ", 10, 5 * FONT_SIZE + 10);
         g.drawString(Double.toString(aveFitness), 200, 5 * FONT_SIZE + 10);

         g.drawString("  Min Fitness: ", 10, 6 * FONT_SIZE + 10);
         g.drawString(Double.toString(minFitness), 200, 6 * FONT_SIZE + 10);
      }

      GeneticAlgorithmIndividualToEvaluate ind = selectedIndividualHolder.getSelectedIndividual();
      if (ind == null)
         return;

      if (ind.isEvaluationDone())
      {
         double fitness = ind.getFitness();

         g.drawString("Selected Individual: ", 10, 100 + 3 * FONT_SIZE + 10);
         g.drawString("  Fitness: " + Double.toString(fitness), 10, 100 + 4 * FONT_SIZE + 10);

         // g.drawString(, 10, 150 + 4*FONT_SIZE+10);
      }
   }
}
