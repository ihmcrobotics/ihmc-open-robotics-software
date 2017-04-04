package us.ihmc.simulationConstructionSetTools.dataExporter;

import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JTextField;

class DataExporterOptionsDialog
{
   private String tag;
   private final JCheckBox saveReadMeCheckBox = new JCheckBox("Save ReadMe");
   private final JCheckBox saveDataCheckBox = new JCheckBox("Save Data");
   private final JCheckBox createSpreadSheetCheckBox = new JCheckBox("Create Torque & Speed Spreadsheet");
   private final JCheckBox createGraphsCheckBoxJPG = new JCheckBox("Create Torque & Speed Graph JPG");
   private final JCheckBox createGraphsCheckBoxPDF = new JCheckBox("Create Torque & Speed Graph PDF");
   private final JCheckBox createVideoCheckBox = new JCheckBox("Create Video");
   private final JCheckBox tagCodeCheckBox = new JCheckBox("Auto Tag Code");
   private final JTextField tagTextField = new JTextField();
   private final JTextField filenameTextField = new JTextField();

   private boolean isCancelled = false;
   private final JDialog dialog;

   private final String simulationDataAndVideoDirectory;

   public DataExporterOptionsDialog(String tag, String simulationDataAndVideoDirectory)
   {
      this.tag = tag;
      JFrame frame = new JFrame();
      dialog = new JDialog(frame, "What would you like to do?", true);
      dialog.addWindowListener(new WindowAdapter()
      {
         @Override
         public void windowClosing(WindowEvent e)
         {
            isCancelled = true;
            dialog.setVisible(false);
         }
      });
      dialog.add(buildOptionsPanel());

      this.simulationDataAndVideoDirectory = simulationDataAndVideoDirectory;
      dialog.setTitle("Saving to " + simulationDataAndVideoDirectory);
      dialog.pack();
      dialog.setVisible(true);
   }

   private JPanel buildOptionsPanel()
   {
      GridBagConstraints gridBagConstraints = new GridBagConstraints();

      // check box panel
      JPanel panel = new JPanel();
      panel.setLayout(new GridBagLayout());

      // layout components
      gridBagConstraints.gridx = 0;
      gridBagConstraints.gridy = 0;
      gridBagConstraints.fill = GridBagConstraints.BOTH;
      gridBagConstraints.gridwidth = 2;
      panel.add(saveReadMeCheckBox, gridBagConstraints);
      saveReadMeCheckBox.setSelected(true);
      gridBagConstraints.gridy++;
      gridBagConstraints.gridwidth = 2;
      panel.add(saveDataCheckBox, gridBagConstraints);
      saveDataCheckBox.setSelected(true);

      gridBagConstraints.gridy++;
      gridBagConstraints.gridwidth = 2;
      panel.add(createSpreadSheetCheckBox, gridBagConstraints);
      createSpreadSheetCheckBox.setSelected(true);

      gridBagConstraints.gridy++;
      gridBagConstraints.gridwidth = 2;
      panel.add(createGraphsCheckBoxJPG, gridBagConstraints);
      createGraphsCheckBoxJPG.setSelected(true);

      gridBagConstraints.gridy++;
      gridBagConstraints.gridwidth = 2;
      panel.add(createGraphsCheckBoxPDF, gridBagConstraints);
      createGraphsCheckBoxPDF.setSelected(true);

      gridBagConstraints.gridy++;
      gridBagConstraints.gridwidth = 2;
      panel.add(createVideoCheckBox, gridBagConstraints);
      createVideoCheckBox.setSelected(true);


      gridBagConstraints.gridy++;
      gridBagConstraints.gridwidth = 1;
      panel.add(tagCodeCheckBox, gridBagConstraints);
      tagCodeCheckBox.setSelected(false);

      gridBagConstraints.gridx++;
      gridBagConstraints.gridwidth = 1;
      panel.add(tagTextField, gridBagConstraints);
      tagTextField.setText(tag);

//    gridBagConstraints.gridy++;
//    gridBagConstraints.gridwidth = 1;
////    gridBagConstraints.gridx++;
////    gridBagConstraints.gridwidth = 1;
//    filenameTextField.setText(simulationDataAndVideoDirectory);
//    panel.add(filenameTextField, gridBagConstraints);

      JButton okButton = new JButton("OK");
      okButton.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent ae)
         {
            isCancelled = false;
            dialog.setVisible(false);
         }
      });
      gridBagConstraints.gridx = 0;
      gridBagConstraints.gridy++;
      gridBagConstraints.gridwidth = 1;
      panel.add(okButton, gridBagConstraints);

      JButton cancelButton = new JButton("CANCEL");
      cancelButton.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent ae)
         {
            isCancelled = true;
            dialog.setVisible(false);
         }
      });
      gridBagConstraints.gridx++;
      gridBagConstraints.gridwidth = 1;
      panel.add(cancelButton, gridBagConstraints);

      return panel;
   }

   public boolean saveReadMe()
   {
      return saveReadMeCheckBox.isSelected();
   }

   public boolean saveData()
   {
      return saveDataCheckBox.isSelected();
   }

   public boolean createSpreadSheet()
   {
      return createSpreadSheetCheckBox.isSelected();
   }

   public boolean createGraphsJPG()
   {
      return createGraphsCheckBoxJPG.isSelected();
   }

   public boolean createGraphsPDF()
   {
      return createGraphsCheckBoxPDF.isSelected();
   }

   public boolean createVideo()
   {
      return createVideoCheckBox.isSelected();
   }

   public boolean tagCode()
   {
      return tagCodeCheckBox.isSelected();
   }

   public String tagName()
   {
      return tagTextField.getText();
   }

   public boolean isCancelled()
   {
      return isCancelled;
   }
}
