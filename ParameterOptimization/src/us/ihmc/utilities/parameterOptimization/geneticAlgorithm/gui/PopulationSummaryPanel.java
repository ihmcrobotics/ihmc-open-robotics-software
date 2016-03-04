package us.ihmc.utilities.parameterOptimization.geneticAlgorithm.gui;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Rectangle;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;

import javax.swing.JPanel;

import us.ihmc.utilities.parameterOptimization.geneticAlgorithm.GeneticAlgorithm;
import us.ihmc.utilities.parameterOptimization.geneticAlgorithm.GeneticAlgorithmIndividualToEvaluate;
import us.ihmc.utilities.parameterOptimization.geneticAlgorithm.Population;

//import javax.swing.border.*;

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

public class PopulationSummaryPanel extends JPanel implements KeyListener, MouseListener
{
   private static final long serialVersionUID = -2961498723532438093L;    // Since Serializable

   private final static int WIDTH = 200;
   private final static int FONT_SIZE = 12;

   private int selected = 0;

   public final static java.text.NumberFormat format = new java.text.DecimalFormat(" 0.00000;-0.00000");
   Dimension sz = new Dimension(WIDTH, 12);

   // private Color viewcolor = new Color(0xd0, 0xff, 0xff);
   // private Color selectcolor = new Color(0x60, 0x00, 0x00);
   // private Thread anim;

   private GeneticAlgorithm ga;
   private SelectedPopulationHolder selectedPopulationHolder;
   private SelectedIndividualHolder selectedIndividualHolder;

   public PopulationSummaryPanel(GeneticAlgorithm ga, SelectedPopulationHolder popHolder, SelectedIndividualHolder individualHolder)
   {
      super();
      this.ga = ga;
      this.selectedPopulationHolder = popHolder;
      this.selectedIndividualHolder = individualHolder;

      this.addMouseListener(this);
      this.addKeyListener(this);
      this.setRequestFocusEnabled(true);

      updatePanelSize();
   }

   public void updatePanelSize()
   {
      sz.setSize(WIDTH, FONT_SIZE * ga.getPopulationSize());
      this.setPreferredSize(sz);
      this.updateUI();
   }

   public void paintComponent(Graphics g)
   {
      super.paintComponent(g);

      // g.drawString("Hello", 100, 100);

      Population population = selectedPopulationHolder.getSelectedPopulation();

      if (population == null)
         return;

      GeneticAlgorithmIndividualToEvaluate[] individuals = population.getAllIndividuals();

      Rectangle rectangle = this.getVisibleRect();
      int i_min = (rectangle.y) / FONT_SIZE - 1;
      int i_max = (rectangle.y + rectangle.height) / FONT_SIZE + 1;
      int size = individuals.length;

      i_min = Math.max(i_min, 0);
      i_min = Math.min(i_min, size);
      i_max = Math.max(i_max, 0);
      i_max = Math.min(i_max, size);

      // for(int i=0; i<individuals.length; i++)
      // {
      for (int i = i_min; i < i_max; i++)
      {
         GeneticAlgorithmIndividualToEvaluate ind = individuals[i];

         if (selectedIndividualHolder.getSelectedIndividual() == ind)
            g.setColor(Color.red);
         else
            g.setColor(Color.black);

         g.drawString(ind.getName(), 2, i * FONT_SIZE + 10);

         if (ind.isEvaluationDone())
         {
            double fitness = ind.getFitness();
            g.drawString(Double.toString(fitness), 200, i * FONT_SIZE + 10);
         }
      }
   }

   public void keyTyped(KeyEvent evt)
   {
   }

   public void keyReleased(KeyEvent evt)
   {
   }

   public void keyPressed(KeyEvent evt)
   {
      // System.out.println("Key Pressed");
      int code = evt.getKeyCode();

      Population population = selectedPopulationHolder.getSelectedPopulation();
      if (population == null)
         return;

      // Individual individual = selectedIndividualHolder.getSelectedIndividual();

      if (code == KeyEvent.VK_UP)
      {
         if (selected > 0)
            selected--;
      }

      else if (code == KeyEvent.VK_DOWN)
      {
         if (selected < population.getNumberOfIndividuals() - 1)
            selected++;
      }

      updateSelected(population);

      // if ((selected < 0) || (selected > population.getNumberOfIndividuals())) selectedIndividualHolder.setSelectedIndividual(null);
      // else selectedIndividualHolder.setSelectedIndividual(population.getIndividual(selected));

      this.repaint();
   }

   public void mousePressed(MouseEvent evt)
   {
   }

   public void mouseReleased(MouseEvent evt)
   {
   }

   public void mouseEntered(MouseEvent evt)
   {
   }

   public void mouseExited(MouseEvent evt)
   {
   }

   public void mouseClicked(MouseEvent evt)
   {
      Population population = selectedPopulationHolder.getSelectedPopulation();
      if (population == null)
         return;

      int y = evt.getY();

      // if (y<0) return;
      selected = y / FONT_SIZE;

      updateSelected(population);

      this.requestFocus();
      this.repaint();
   }

   public void resetSelected()
   {
      selected = 0;

      Population population = selectedPopulationHolder.getSelectedPopulation();
      if (population == null)
         return;

      updateSelected(population);
   }

   private void updateSelected(Population pop)
   {
      int popSize = pop.getNumberOfIndividuals();

      if (selected > popSize)
         selected = popSize - 1;
      if (selected < 0)
         selected = 0;

      if ((selected >= 0) && (selected < popSize))    // selectedVariable = (YoVariable) variables.getVariables().get(selected);
      {
         selectedIndividualHolder.setSelectedIndividual(pop.getIndividual(selected));
      }

   }

}
