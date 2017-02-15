package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.awt.Dimension;

import javax.swing.JDialog;
import javax.swing.JEditorPane;
import javax.swing.JFrame;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class AboutDialogGenerator implements AboutDialogConstructor
{
   private JFrame parentJFrame;

   public AboutDialogGenerator(JFrame parentJFrame)
   {
      this.parentJFrame = parentJFrame;
   }

   @Override
   public void constructDialog()
   {
      String scsVersionNumber = SimulationConstructionSet.getVersion();

      JEditorPane jEditorPane = new JEditorPane();
      jEditorPane.setText("SimulationConstructionSet.\n\n Originally developed at Yobotics, Inc. from 2000-2010. \n\n Now developed at IHMC from 2002 to the present. \n\n Website: http://ihmc.us/groups/scs/");
      JDialog aboutDialog = new JDialog(parentJFrame, "Simulation Construction Set Version: " + scsVersionNumber, false);

      // JScrollPane aboutScrollPane = new JScrollPane(aboutEditorPane, JScrollPane.VERTICAL_SCROLLBAR_ALWAYS, JScrollPane.HORIZONTAL_SCROLLBAR_ALWAYS);

      // aboutDialog.getContentPane().add(aboutScrollPane);
      aboutDialog.getContentPane().add(jEditorPane);

      aboutDialog.setSize(new Dimension(450, 300));
      aboutDialog.validate();
      aboutDialog.setVisible(true);
   }

   public void closeAndDispose()
   {
      parentJFrame = null; 
   }

}
