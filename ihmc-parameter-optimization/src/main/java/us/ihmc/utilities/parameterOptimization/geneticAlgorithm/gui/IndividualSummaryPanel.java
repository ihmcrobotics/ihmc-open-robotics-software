package us.ihmc.utilities.parameterOptimization.geneticAlgorithm.gui;

import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.border.TitledBorder;

import us.ihmc.utilities.parameterOptimization.geneticAlgorithm.GeneticAlgorithmIndividualToEvaluate;

/**
 * <p>
 * Title: Genetic Algorithm Library
 * </p>
 * <p>
 * Description: General Purpose Genetic Algorithm Library
 * </p>
 * <p>
 * Copyright: Copyright (c) 2003-2005 Jerry Pratt, IHMC
 * </p>
 * <p>
 * Company: Institute for Human and Machine Cognition. 40 South Alcaniz Street Pensacola, FL 32502
 * </p>
 *
 * @author Jerry Pratt and Jim Warrenfeltz, jpratt@ihmc.us
 * @version 1.0
 */

public class IndividualSummaryPanel extends JPanel implements ActionListener
{
   private static final long serialVersionUID = 3983892175336480563L; // Since Serializable

   private SelectedIndividualHolder selectedIndividualHolder;

   private JTextArea textArea;

   public IndividualSummaryPanel(SelectedIndividualHolder holder)
   {
      super(new BorderLayout());
      this.selectedIndividualHolder = holder;

      textArea = new JTextArea();
      JScrollPane individualSummaryScrollPane = new JScrollPane(textArea, JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED, JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
      individualSummaryScrollPane.setBorder(new TitledBorder("Individual Summary"));

      JButton replayButton = new JButton("Replay");
      replayButton.addActionListener(this);
      this.add(individualSummaryScrollPane, BorderLayout.CENTER);
      this.add(replayButton, BorderLayout.NORTH);
      this.setVisible(true);
   }

   public void individualChanged()
   {
      GeneticAlgorithmIndividualToEvaluate individual = selectedIndividualHolder.getSelectedIndividual();
      if (individual == null)
         textArea.setText("");

      else if (!individual.isEvaluationDone())
         textArea.setText("Waiting to eval!");

      else
         textArea.setText(individual.toString());
   }

   @Override
   public void actionPerformed(ActionEvent e)
   {
      GeneticAlgorithmIndividualToEvaluate individual = selectedIndividualHolder.getSelectedIndividual();

      if (individual == null) 
         return;
      
      else if (!individual.isEvaluationDone())
         return;
      
      else
      {
         individual.replay();
      }
   }

}
