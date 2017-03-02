package us.ihmc.simulationconstructionset.gui.dialogs;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.Point;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.border.Border;

import us.ihmc.simulationconstructionset.gui.ExportDataDialogListener;
import us.ihmc.simulationconstructionset.gui.config.VarGroupList;


public class ExportDataDialog extends JDialog implements ActionListener
{
   private static final long serialVersionUID = 8567689788428234128L;

   public static final int
      STATE = 0, DATA = 1, COMPRESS = 5, NO_COMPRESS = 6;

   private JButton exportButton, cancelButton;
   private ExportDataPanel exportPanel;
   private JFrame parentFrame;
   private ExportDataDialogListener listener;
   private VarGroupList varGroupList;

   public ExportDataDialog(JFrame frame, VarGroupList varGroupList, ExportDataDialogListener listener)
   {
      super(frame, "Export Data", false);
      this.parentFrame = frame;
      this.listener = listener;
      this.varGroupList = varGroupList;

      Container contentPane = this.getContentPane();

      exportPanel = new ExportDataPanel();
      contentPane.add(exportPanel);

      // Bottom Buttons:

      exportButton = new JButton("Export");
      exportButton.addActionListener(this);
      cancelButton = new JButton("Cancel");
      cancelButton.addActionListener(this);
      JPanel buttonPanel = new JPanel();
      buttonPanel.add(exportButton);
      buttonPanel.add(cancelButton);

      contentPane.add(buttonPanel, BorderLayout.SOUTH);

      Point point = frame.getLocation();
      Dimension frameSize = frame.getSize();

      point.translate(frameSize.width / 2, frameSize.height / 4);
      this.setLocation(point);

      this.setResizable(false);

      // Enlarge Text Fields:

      /*
       * size = currentTextField.getSize(); size.width = size.width*5/4;
       * currentTextField.setSize(size);
       * currentTextField.setPreferredSize(size);
       * currentTextField.setMinimumSize(size);
       */

      this.pack();
      this.setVisible(true);

      parentFrame.repaint();    // This is a horrible way to get the graphs to repaint...

   }

   @Override
   public void actionPerformed(ActionEvent event)
   {
      if (event.getSource() == cancelButton)
      {
         this.setVisible(false);
      }

      if (event.getSource() == exportButton)
      {
         this.setVisible(false);
         exportPanel.export();
      }

      parentFrame.repaint();    // This is a horrible way to get the graphs to repaint...
   }

   private class ExportDataPanel extends JPanel implements ActionListener
   {
      /**
       *
       */
      private static final long serialVersionUID = 2066572350748134411L;
      private JLabel varGroupJLabel = new JLabel();
      private GridBagLayout gridBagLayout1 = new GridBagLayout();
      private JComboBox varGroupComboBox = new JComboBox();
      private ButtonGroup stateDataButtonGroup = new ButtonGroup();
      private JRadioButton stateRadioButton = new JRadioButton();
      private JRadioButton dataRadioButton = new JRadioButton();
      private JRadioButton asciiRadioButton = new JRadioButton();
      private JRadioButton matlabRadioButton = new JRadioButton();
      private JRadioButton spreadsheetRadioButton = new JRadioButton();
      private JRadioButton binaryRadioButton = new JRadioButton();
      private ButtonGroup dataFormatButtonGroup = new ButtonGroup();
      private JRadioButton compressRadioButton = new JRadioButton();
      private JRadioButton noCompressRadioButton = new JRadioButton();
      private ButtonGroup compressButtonGroup = new ButtonGroup();

      private Component horizontalStrut1;
      private Component verticalStrut1;

      public ExportDataPanel()
      {
         horizontalStrut1 = Box.createHorizontalStrut(60);
         verticalStrut1 = Box.createVerticalStrut(4);
         varGroupJLabel.setToolTipText("");
         varGroupJLabel.setText("VarGroup:   ");
         this.setLayout(gridBagLayout1);

         stateRadioButton.setText("State");
         stateRadioButton.setSelected(false);
         stateRadioButton.addActionListener(this);
         dataRadioButton.setText("Data");
         dataRadioButton.setSelected(true);
         dataRadioButton.addActionListener(this);

         asciiRadioButton.setText("ASCII (Similar to Matlab/Octave Script)");
         asciiRadioButton.setSelected(false);
         asciiRadioButton.addActionListener(this);
         binaryRadioButton.setText("Binary");
         binaryRadioButton.setSelected(true);
         binaryRadioButton.addActionListener(this);
         matlabRadioButton.setText("Matlab/Octave (*.mat)");
         matlabRadioButton.setSelected(false);
         matlabRadioButton.addActionListener(this);
         spreadsheetRadioButton.setText("Comma-Separated-Values (CSV)");
         spreadsheetRadioButton.setSelected(false);
         spreadsheetRadioButton.addActionListener(this);

         compressRadioButton.setText("Compress");
         compressRadioButton.setSelected(true);
         compressRadioButton.addActionListener(this);
         noCompressRadioButton.setText("No Compression");
         noCompressRadioButton.setSelected(false);
         noCompressRadioButton.addActionListener(this);

         stateDataButtonGroup.add(stateRadioButton);
         stateDataButtonGroup.add(dataRadioButton);

         dataFormatButtonGroup.add(asciiRadioButton);
         dataFormatButtonGroup.add(binaryRadioButton);
         dataFormatButtonGroup.add(matlabRadioButton);
         dataFormatButtonGroup.add(spreadsheetRadioButton);
         

         compressButtonGroup.add(compressRadioButton);
         compressButtonGroup.add(noCompressRadioButton);

         varGroupComboBox.setMaximumSize(new Dimension(125, 21));
         varGroupComboBox.setMinimumSize(new Dimension(125, 21));
         varGroupComboBox.addActionListener(new java.awt.event.ActionListener()
         {
            @Override
            public void actionPerformed(ActionEvent e)
            {
               // varGroupComboBox_actionPerformed(e);
            }
         });

         // Add the VarGroups to the varGroupComboBox:
         varGroupComboBox.addItem("all");
         String[] names = varGroupList.getVarGroupNames();
         for (int i = 0; i < names.length; i++)
         {
            varGroupComboBox.addItem(names[i]);
         }

         add(varGroupComboBox, new GridBagConstraints(1, 0, 2, 1, 0.0, 0.0, GridBagConstraints.CENTER, GridBagConstraints.NONE, new Insets(0, 0, 0, 0), 36, 0));
         add(varGroupJLabel, new GridBagConstraints(0, 0, 1, 1, 0.0, 0.0, GridBagConstraints.WEST, GridBagConstraints.NONE, new Insets(0, 10, 0, 0), 0, 0));
         add(horizontalStrut1, new GridBagConstraints(1, 1, 1, 1, 0.0, 0.0, GridBagConstraints.CENTER, GridBagConstraints.NONE, new Insets(0, 0, 0, 0), 0, 0));

         add(stateRadioButton, new GridBagConstraints(0, 3, 1, 1, 0.0, 0.0, GridBagConstraints.WEST, GridBagConstraints.NONE, new Insets(0, 10, 0, 0), 0, 0));
         add(dataRadioButton, new GridBagConstraints(0, 2, 1, 1, 0.0, 0.0, GridBagConstraints.WEST, GridBagConstraints.NONE, new Insets(0, 10, 0, 0), 0, 0));

         add(verticalStrut1, new GridBagConstraints(0, 1, 1, 1, 0.0, 0.0, GridBagConstraints.CENTER, GridBagConstraints.NONE, new Insets(0, 0, 0, 0), 0, 0));

         add(binaryRadioButton, new GridBagConstraints(1, 2, 1, 1, 0.0, 0.0, GridBagConstraints.WEST, GridBagConstraints.NONE, new Insets(0, 0, 0, 0), 0, 0));
         add(asciiRadioButton, new GridBagConstraints(1, 3, 1, 1, 0.0, 0.0, GridBagConstraints.WEST, GridBagConstraints.NONE, new Insets(0, 0, 0, 0), 0, 0));
         add(matlabRadioButton, new GridBagConstraints(1, 4, 1, 1, 0.0, 0.0, GridBagConstraints.WEST, GridBagConstraints.NONE, new Insets(0, 0, 0, 0), 0, 0));
         add(spreadsheetRadioButton,
             new GridBagConstraints(1, 5, 1, 1, 0.0, 0.0, GridBagConstraints.WEST, GridBagConstraints.NONE, new Insets(0, 0, 0, 0), 0, 0));

         add(verticalStrut1, new GridBagConstraints(0, 1, 1, 1, 0.0, 0.0, GridBagConstraints.CENTER, GridBagConstraints.NONE, new Insets(0, 0, 0, 0), 0, 0));

         add(compressRadioButton, new GridBagConstraints(2, 2, 1, 1, 0.0, 0.0, GridBagConstraints.WEST, GridBagConstraints.NONE, new Insets(0, 0, 0, 0), 0, 0));
         add(noCompressRadioButton,
             new GridBagConstraints(2, 3, 1, 1, 0.0, 0.0, GridBagConstraints.WEST, GridBagConstraints.NONE, new Insets(0, 0, 0, 0), 0, 0));

         add(verticalStrut1, new GridBagConstraints(0, 1, 1, 1, 0.0, 0.0, GridBagConstraints.CENTER, GridBagConstraints.NONE, new Insets(0, 0, 0, 0), 0, 0));

         Border blackLine = BorderFactory.createLineBorder(Color.black);

         // TitledBorder title = BorderFactory.createTitledBorder(blackLine,selectedVariable.getName());
         // this.setBorder(title);
         this.setBorder(blackLine);

      }

      @Override
      public void actionPerformed(ActionEvent event)
      {
         if (event.getSource() == stateRadioButton)
         {
            asciiRadioButton.setSelected(true);
            binaryRadioButton.setEnabled(false);
            matlabRadioButton.setEnabled(false);
            spreadsheetRadioButton.setEnabled(false);
            noCompressRadioButton.setSelected(true);
         }

         if (event.getSource() == dataRadioButton)
         {
            binaryRadioButton.setEnabled(true);
            binaryRadioButton.setSelected(true);
            matlabRadioButton.setEnabled(true);
            spreadsheetRadioButton.setEnabled(true);            
            compressRadioButton.setSelected(true);
         }

         if (event.getSource() == asciiRadioButton)
         {
            noCompressRadioButton.setSelected(true);
         }

         if (event.getSource() == binaryRadioButton)
         {
            compressRadioButton.setSelected(true);
         }


         if (event.getSource() == matlabRadioButton || event.getSource() == spreadsheetRadioButton)
         {
            compressRadioButton.setEnabled(false);
            noCompressRadioButton.setEnabled(false);
         }
         else
         {
            compressRadioButton.setEnabled(true);
            noCompressRadioButton.setEnabled(true);
         }

      }

      public void export()
      {
         String varGroup = varGroupComboBox.getSelectedItem().toString();
         int dataType = stateRadioButton.isSelected() ? STATE : DATA;
         SCSExportDataFormat dataFormat;
         if (asciiRadioButton.isSelected())
         {
            dataFormat = SCSExportDataFormat.ASCII;
         }
         else if (binaryRadioButton.isSelected())
         {
            dataFormat = SCSExportDataFormat.BINARY;
         }
         else if(matlabRadioButton.isSelected())
         {
            dataFormat = SCSExportDataFormat.MATLAB;
         }
         else if(spreadsheetRadioButton.isSelected())
         {
            dataFormat=SCSExportDataFormat.SPREADSHEET;
         }else
            throw new RuntimeException("unknown data format");

         int dataCompression = compressRadioButton.isSelected() ? COMPRESS : NO_COMPRESS;

         listener.export(varGroup, dataType, dataFormat, dataCompression);
      }
   }
}
