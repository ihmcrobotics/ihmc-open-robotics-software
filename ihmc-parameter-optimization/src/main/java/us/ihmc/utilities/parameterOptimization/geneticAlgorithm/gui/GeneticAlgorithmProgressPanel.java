package us.ihmc.utilities.parameterOptimization.geneticAlgorithm.gui;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;

import javax.swing.JPanel;

import us.ihmc.utilities.parameterOptimization.geneticAlgorithm.GeneticAlgorithm;
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

public class GeneticAlgorithmProgressPanel extends JPanel
{
   private static final long serialVersionUID = -8173031386267589793L;    // Since Serializable.

   private GeneticAlgorithm ga;

   private final static int
      DESIRED_WIDTH = 800, DESIRED_HEIGHT = 200;

   public GeneticAlgorithmProgressPanel(GeneticAlgorithm ga)
   {
      this.ga = ga;

      // updatePanelSize();

      this.setLayout(new BorderLayout());

      // this.validate();
   }

   public void updatePanelSize()
   {
      // size.setSize(this.getWidth(), DESIRED_HEIGHT);
      Dimension size = new Dimension();
      size.setSize(DESIRED_WIDTH, DESIRED_HEIGHT);
      this.setPreferredSize(size);
      this.updateUI();
   }


   public void paintComponent(Graphics g)
   {
      super.paintComponent(g);

      // Dimension size = this.getSize();
      // System.out.println(size);

      Population[] populations = ga.getPopulations();
      double maxFitness = Double.NEGATIVE_INFINITY;
      double minFitness = Double.POSITIVE_INFINITY;
      double minPopNumber = Integer.MAX_VALUE;
      double maxPopNumber = Integer.MIN_VALUE;

      // Find ranges of plot.
      for (int i = 0; i < populations.length; i++)
      {
         Population population = populations[i];
         if (population.allIndividualsEvaluated())
         {
            if (population.getMaximumFitness() > maxFitness)
               maxFitness = population.getMaximumFitness();
            if (population.getMinimumFitness() < minFitness)
               minFitness = population.getMinimumFitness();
            if (population.getPopulationNumber() > maxPopNumber)
               maxPopNumber = population.getPopulationNumber();
            if (population.getPopulationNumber() < minPopNumber)
               minPopNumber = population.getPopulationNumber();
         }
      }

      for (int i = 0; i < populations.length; i++)
      {
         Population population = populations[i];
         if (population.allIndividualsEvaluated())
         {
            double max = population.getMaximumFitness();
            double min = population.getMinimumFitness();
            double ave = population.getAverageFitness();

            int popNumber = population.getPopulationNumber();

            g.setColor(Color.RED);
            double x = (popNumber - minPopNumber) / (maxPopNumber - minPopNumber) * getWidth();
            double y = getHeight() - (max - minFitness) / (maxFitness - minFitness) * getHeight();
            g.fillOval(((int) x), ((int) y), 4, 4);

            g.setColor(Color.BLUE);
            y = getHeight() - (ave - minFitness) / (maxFitness - minFitness) * getHeight();
            g.fillOval(((int) x), ((int) y), 4, 4);

            g.setColor(Color.BLACK);
            y = getHeight() - (min - minFitness) / (maxFitness - minFitness) * getHeight();
            g.fillOval(((int) x), ((int) y), 4, 4);


         }
      }
   }


}
