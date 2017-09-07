package us.ihmc.plotting;

import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;

import javax.swing.JPanel;

public class PlotterPanel extends JPanel
{
   private static final long serialVersionUID = 4697277324924209439L;
   protected Plotter plotter;

   public PlotterPanel()
   {
      plotter = new Plotter();

      GridBagConstraints gridBagConstraints = new GridBagConstraints();
      setLayout(new GridBagLayout());

      gridBagConstraints.gridx = 0;
      gridBagConstraints.gridy = 0;
      gridBagConstraints.fill = GridBagConstraints.BOTH;
      gridBagConstraints.gridwidth = 5;
      gridBagConstraints.gridheight = 5;
      gridBagConstraints.weightx = 1;
      gridBagConstraints.weighty = 1;
      add(plotter.getJPanel(), gridBagConstraints);
   }

   public Plotter getPlotter()
   {
      return plotter;
   }
}
