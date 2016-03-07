package us.ihmc.utilities.parameterOptimization.geneticAlgorithm.gui;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;

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

public class GeneticAlgorithmSummaryPanel extends JPanel implements KeyListener, MouseListener
{
   private static final long serialVersionUID = 2161678567990103733L;    // Since Serializable

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

   public GeneticAlgorithmSummaryPanel(GeneticAlgorithm ga, SelectedPopulationHolder holder)
   {
      super();
      this.ga = ga;
      this.selectedPopulationHolder = holder;

      this.addMouseListener(this);
      this.addKeyListener(this);
      this.setRequestFocusEnabled(true);

      // selected = 0;
      // updateSelected();

      updatePanelSize();
   }

   public void updatePanelSize()
   {
      sz.setSize(WIDTH, FONT_SIZE * ga.getNumberOfPopulations());
      this.setPreferredSize(sz);
      this.updateUI();
   }

   public void paintComponent(Graphics g)
   {
      super.paintComponent(g);

      // g.drawString("Hello", 100, 100);

      Population[] populations = ga.getPopulations();
      for (int i = 0; i < populations.length; i++)
      {
         Population pop = populations[i];

         if (selectedPopulationHolder.getSelectedPopulation() == pop)
            g.setColor(Color.red);
         else
            g.setColor(Color.black);

         g.drawString(pop.getName() + "  " + pop.getPopulationNumber(), 2, i * FONT_SIZE + 10);

         if (pop.allIndividualsEvaluated())
         {
            double maxFitness = pop.getMaximumFitness();
            g.drawString(Double.toString(maxFitness), 160, i * FONT_SIZE + 10);
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

      if (code == KeyEvent.VK_UP)
      {
         if (selected > 0)
            selected--;
      }

      else if (code == KeyEvent.VK_DOWN)
      {
         if (selected < ga.getNumberOfPopulations() - 1)
            selected++;
      }

      updateSelected();

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
      int y = evt.getY();
      if (y < 0)
         return;
      selected = y / FONT_SIZE;

      updateSelected();

      this.requestFocus();
      this.repaint();
   }

   private void updateSelected()
   {
      if (ga == null)
         selected = -1;
      else if (selected >= ga.getNumberOfPopulations())
         selected = -1;

      if ((selected >= 0) && (selected < ga.getNumberOfPopulations()))    // selectedVariable = (YoVariable) variables.getVariables().get(selected);
      {
         Population selectedPopulation = ga.getPopulation(selected);
         selectedPopulationHolder.setSelectedPopulation(selectedPopulation);
      }
   }

   public void resetSelected()
   {
      selected = 0;

      updateSelected();
   }

}
