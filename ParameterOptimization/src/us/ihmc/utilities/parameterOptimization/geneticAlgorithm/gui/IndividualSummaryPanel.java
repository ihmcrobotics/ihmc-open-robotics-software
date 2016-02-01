package us.ihmc.utilities.parameterOptimization.geneticAlgorithm.gui;

import javax.swing.JTextArea;

import us.ihmc.utilities.parameterOptimization.geneticAlgorithm.GeneticAlgorithmIndividualToEvaluate;

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

public class IndividualSummaryPanel extends JTextArea
{
   private static final long serialVersionUID = 3983892175336480563L;    // Since Serializable

   private SelectedIndividualHolder selectedIndividualHolder;

   public IndividualSummaryPanel(SelectedIndividualHolder holder)
   {
      this.selectedIndividualHolder = holder;
   }

   public void individualChanged()
   {
      GeneticAlgorithmIndividualToEvaluate ind = selectedIndividualHolder.getSelectedIndividual();
      if (ind == null)
         this.setText("");

      else if (!ind.isEvaluationDone())
         setText("Waiting to eval!");

      else
         this.setText(ind.toString());
   }

}
