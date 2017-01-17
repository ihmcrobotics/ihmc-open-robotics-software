package us.ihmc.simulationconstructionset.gui.dialogs;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.Point;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.text.NumberFormat;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.border.Border;

import us.ihmc.jMonkeyEngineToolkit.camera.CameraPropertiesHolder;
import us.ihmc.simulationconstructionset.gui.ActiveCameraHolder;


public class CameraPropertiesDialog extends JDialog implements ActionListener
{
   private static final long serialVersionUID = 859815534934501986L;

   private NumberFormat numFormat;

   private JCheckBox trackCheckBox, trackXCheckBox, trackYCheckBox, trackZCheckBox;
   private JCheckBox dollyCheckBox, dollyXCheckBox, dollyYCheckBox, dollyZCheckBox;

   private JTextField trackXTextField, trackYTextField, trackZTextField;
   private JTextField dollyXTextField, dollyYTextField, dollyZTextField;

   private JTextField fixXTextField, fixYTextField, fixZTextField;
   private JTextField camXTextField, camYTextField, camZTextField;

   private double newTrackDx, newTrackDy, newTrackDz, newDollyDx, newDollyDy, newDollyDz;
   private double newFixX, newFixY, newFixZ, newCamX, newCamY, newCamZ;

   private JButton trackCurrentButton, dollyCurrentButton;


   // private final static java.text.NumberFormat numFormat = new java.text.DecimalFormat(" 0.00000;-0.00000");

   private JButton okButton, applyButton, cancelButton;
   private CameraPropertiesPanel cameraPropertiesPanel;
   private Container parentContainer;
   @SuppressWarnings("unused")
   private JFrame parentFrame;

   // private CameraPropertiesDialogListener listener;
   private ActiveCameraHolder cameraHolder;
   private JCheckBox GUITrackCheckBox, GUIDollyCheckBox;

   public CameraPropertiesDialog(Container parentContainer, JFrame frame, JCheckBox GUITrackCheckBox, JCheckBox GUIDollyCheckBox,
                                 ActiveCameraHolder cameraHolder)    // CameraPropertiesDialogListener listener)
   {
      super(frame, "Camera Properties", false);
      this.parentContainer = parentContainer;
      this.parentFrame = frame;

      this.cameraHolder = cameraHolder;

      // this.listener = listener;
      this.GUITrackCheckBox = GUITrackCheckBox;
      this.GUIDollyCheckBox = GUIDollyCheckBox;

      this.numFormat = NumberFormat.getInstance();
      this.numFormat.setMaximumFractionDigits(4);
      this.numFormat.setMinimumFractionDigits(1);
      this.numFormat.setGroupingUsed(false);

      Container contentPane = this.getContentPane();

      cameraPropertiesPanel = new CameraPropertiesPanel();
      contentPane.add(cameraPropertiesPanel);

      // Bottom Buttons:

      okButton = new JButton("OK");
      okButton.addActionListener(this);
      applyButton = new JButton("Apply");
      applyButton.addActionListener(this);
      cancelButton = new JButton("Cancel");
      cancelButton.addActionListener(this);
      JPanel buttonPanel = new JPanel();
      buttonPanel.add(okButton);
      buttonPanel.add(applyButton);
      buttonPanel.add(cancelButton);

      contentPane.add(buttonPanel, BorderLayout.SOUTH);

      Point point = parentContainer.getLocation();
      Dimension frameSize = parentContainer.getSize();

      point.translate(frameSize.width / 2, frameSize.height / 4);
      this.setLocation(point);

      this.setResizable(false);
      this.pack();

      // Enlarge Text Fields:

      int width = trackXTextField.getSize().width;

      width = Math.max(width, trackYTextField.getSize().width);
      width = Math.max(width, trackZTextField.getSize().width);
      width = Math.max(width, dollyXTextField.getSize().width);
      width = Math.max(width, dollyYTextField.getSize().width);
      width = Math.max(width, dollyZTextField.getSize().width);
      width = Math.max(width, fixXTextField.getSize().width);
      width = Math.max(width, fixYTextField.getSize().width);
      width = Math.max(width, fixZTextField.getSize().width);
      width = Math.max(width, camXTextField.getSize().width);
      width = Math.max(width, camYTextField.getSize().width);
      width = Math.max(width, camZTextField.getSize().width);

      width = width * 5 / 4;

      Dimension size = trackXTextField.getSize();
      size.width = width;

      trackXTextField.setSize(size);
      trackXTextField.setPreferredSize(size);
      trackXTextField.setMinimumSize(size);
      trackYTextField.setSize(size);
      trackYTextField.setPreferredSize(size);
      trackYTextField.setMinimumSize(size);
      trackZTextField.setSize(size);
      trackZTextField.setPreferredSize(size);
      trackZTextField.setMinimumSize(size);

      dollyXTextField.setSize(size);
      dollyXTextField.setPreferredSize(size);
      dollyXTextField.setMinimumSize(size);
      dollyYTextField.setSize(size);
      dollyYTextField.setPreferredSize(size);
      dollyYTextField.setMinimumSize(size);
      dollyZTextField.setSize(size);
      dollyZTextField.setPreferredSize(size);
      dollyZTextField.setMinimumSize(size);

      fixXTextField.setSize(size);
      fixXTextField.setPreferredSize(size);
      fixXTextField.setMinimumSize(size);
      fixYTextField.setSize(size);
      fixYTextField.setPreferredSize(size);
      fixYTextField.setMinimumSize(size);
      fixZTextField.setSize(size);
      fixZTextField.setPreferredSize(size);
      fixZTextField.setMinimumSize(size);

      camXTextField.setSize(size);
      camXTextField.setPreferredSize(size);
      camXTextField.setMinimumSize(size);
      camYTextField.setSize(size);
      camYTextField.setPreferredSize(size);
      camYTextField.setMinimumSize(size);
      camZTextField.setSize(size);
      camZTextField.setPreferredSize(size);
      camZTextField.setMinimumSize(size);

      /*
       * size = currentTextField.getSize();
       * size.width = size.width*5/4;
       * currentTextField.setSize(size);
       * currentTextField.setPreferredSize(size);
       * currentTextField.setMinimumSize(size);
       */

      this.pack();
      this.setVisible(true);
      GUITrackCheckBox.setEnabled(false);
      GUIDollyCheckBox.setEnabled(false);

      parentContainer.repaint();    // This is a horrible way to get the graphs to repaint...
   }

   public void actionPerformed(ActionEvent event)
   {
      if (event.getSource() == cancelButton)
      {
         this.setVisible(false);
         GUITrackCheckBox.setEnabled(true);
         GUIDollyCheckBox.setEnabled(true);
      }

      if (event.getSource() == applyButton)
      {
         cameraPropertiesPanel.commitChanges();
      }

      if (event.getSource() == okButton)
      {
         cameraPropertiesPanel.commitChanges();
         this.setVisible(false);
         GUITrackCheckBox.setEnabled(true);
         GUIDollyCheckBox.setEnabled(true);
      }

      parentContainer.repaint();    // This is a horrible way to get the graphs to repaint...
   }

   @SuppressWarnings("serial")
   public class CameraPropertiesPanel extends JPanel implements ActionListener
   {
      public CameraPropertiesPanel()
      {
         super();

         JPanel trackPanel = new JPanel();
         JPanel dollyPanel = new JPanel();

         CameraPropertiesHolder listener = cameraHolder.getCameraPropertiesForActiveCamera();

         newTrackDx = listener.getTrackingXOffset();
         newTrackDy = listener.getTrackingYOffset();
         newTrackDz = listener.getTrackingZOffset();

         newDollyDx = listener.getDollyXOffset();
         newDollyDy = listener.getDollyYOffset();
         newDollyDz = listener.getDollyZOffset();

         newFixX = listener.getFixX();
         newFixY = listener.getFixY();
         newFixZ = listener.getFixZ();

         newCamX = listener.getCamX();
         newCamY = listener.getCamY();
         newCamZ = listener.getCamZ();

         GridBagLayout trackGridbag = new GridBagLayout();
         trackPanel.setLayout(trackGridbag);
         Border blackLine = BorderFactory.createLineBorder(Color.black);

         // TitledBorder title = BorderFactory.createTitledBorder(blackLine,selectedVariable.getName());
         // this.setBorder(title);
         trackPanel.setBorder(blackLine);

         GridBagLayout dollyGridbag = new GridBagLayout();
         dollyPanel.setLayout(dollyGridbag);
         blackLine = BorderFactory.createLineBorder(Color.black);

         // TitledBorder title = BorderFactory.createTitledBorder(blackLine,selectedVariable.getName());
         // this.setBorder(title);
         dollyPanel.setBorder(blackLine);


         GridBagConstraints constraints = new GridBagConstraints();

         // Row 0:

         JLabel fixLabel = new JLabel("       Fix Position:       ");
         setConstraints(constraints, 0, 0, 2, GridBagConstraints.WEST);
         trackGridbag.setConstraints(fixLabel, constraints);
         trackPanel.add(fixLabel);

         // JLabel policyLabel = new JLabel("   Filled Policy:  ");
         trackCheckBox = new JCheckBox("Track", listener.isTracking());
         trackCheckBox.setName("Track");
         trackCheckBox.addActionListener(this);
         setConstraints(constraints, 2, 0, 1, GridBagConstraints.EAST);
         trackGridbag.setConstraints(trackCheckBox, constraints);
         trackPanel.add(trackCheckBox);

         trackCurrentButton = new TrackCurrentButton();
         setConstraints(constraints, 3, 0, 2, GridBagConstraints.WEST);
         trackGridbag.setConstraints(trackCurrentButton, constraints);
         trackPanel.add(trackCurrentButton);

         // Row 1:

         JLabel fixXLabel = new JLabel("  X:  ");
         setConstraints(constraints, 0, 1, 1, GridBagConstraints.WEST);
         trackGridbag.setConstraints(fixXLabel, constraints);
         trackPanel.add(fixXLabel);

         String fixXVal = numFormat.format(newFixX);
         fixXTextField = new JTextField(fixXVal);
         fixXTextField.addActionListener(this);
         setConstraints(constraints, 1, 1, 1, GridBagConstraints.WEST);
         trackGridbag.setConstraints(fixXTextField, constraints);
         trackPanel.add(fixXTextField);

         trackXCheckBox = new JCheckBox("Track X", listener.isTrackingX());
         trackXCheckBox.addActionListener(this);
         setConstraints(constraints, 2, 1, 1, GridBagConstraints.EAST);
         trackGridbag.setConstraints(trackXCheckBox, constraints);
         trackPanel.add(trackXCheckBox);

         JLabel trackDXLabel = new JLabel("  X Offset:  ");
         setConstraints(constraints, 3, 1, 1, GridBagConstraints.EAST);
         trackGridbag.setConstraints(trackDXLabel, constraints);
         trackPanel.add(trackDXLabel);

         String trackDXVal = numFormat.format(newTrackDx);
         trackXTextField = new JTextField(trackDXVal);
         trackXTextField.addActionListener(this);
         setConstraints(constraints, 4, 1, 1, GridBagConstraints.WEST);
         trackGridbag.setConstraints(trackXTextField, constraints);
         trackPanel.add(trackXTextField);


         // Row 2:

         JLabel fixYLabel = new JLabel("  Y:  ");
         setConstraints(constraints, 0, 2, 1, GridBagConstraints.WEST);
         trackGridbag.setConstraints(fixYLabel, constraints);
         trackPanel.add(fixYLabel);

         String fixYVal = numFormat.format(newFixY);
         fixYTextField = new JTextField(fixYVal);
         fixYTextField.addActionListener(this);
         setConstraints(constraints, 1, 2, 1, GridBagConstraints.WEST);
         trackGridbag.setConstraints(fixYTextField, constraints);
         trackPanel.add(fixYTextField);

         trackYCheckBox = new JCheckBox("Track Y", listener.isTrackingY());
         trackYCheckBox.addActionListener(this);
         setConstraints(constraints, 2, 2, 1, GridBagConstraints.EAST);
         trackGridbag.setConstraints(trackYCheckBox, constraints);
         trackPanel.add(trackYCheckBox);

         JLabel trackDYLabel = new JLabel("  Y Offset:  ");
         setConstraints(constraints, 3, 2, 1, GridBagConstraints.EAST);
         trackGridbag.setConstraints(trackDYLabel, constraints);
         trackPanel.add(trackDYLabel);

         String trackDYVal = numFormat.format(newTrackDy);
         trackYTextField = new JTextField(trackDYVal);
         trackYTextField.addActionListener(this);
         setConstraints(constraints, 4, 2, 1, GridBagConstraints.WEST);
         trackGridbag.setConstraints(trackYTextField, constraints);
         trackPanel.add(trackYTextField);


         // Row 3:

         JLabel fixZLabel = new JLabel("  Z:  ");
         setConstraints(constraints, 0, 3, 1, GridBagConstraints.WEST);
         trackGridbag.setConstraints(fixZLabel, constraints);
         trackPanel.add(fixZLabel);

         String fixZVal = numFormat.format(newFixZ);
         fixZTextField = new JTextField(fixZVal);
         fixZTextField.addActionListener(this);
         setConstraints(constraints, 1, 3, 1, GridBagConstraints.WEST);
         trackGridbag.setConstraints(fixZTextField, constraints);
         trackPanel.add(fixZTextField);

         trackZCheckBox = new JCheckBox("Track Z", listener.isTrackingZ());
         trackZCheckBox.addActionListener(this);
         setConstraints(constraints, 2, 3, 1, GridBagConstraints.EAST);
         trackGridbag.setConstraints(trackZCheckBox, constraints);
         trackPanel.add(trackZCheckBox);

         JLabel trackDZLabel = new JLabel("  Z Offset:  ");
         setConstraints(constraints, 3, 3, 1, GridBagConstraints.EAST);
         trackGridbag.setConstraints(trackDZLabel, constraints);
         trackPanel.add(trackDZLabel);

         String trackDZVal = numFormat.format(newTrackDz);
         trackZTextField = new JTextField(trackDZVal);
         trackZTextField.addActionListener(this);
         setConstraints(constraints, 4, 3, 1, GridBagConstraints.WEST);
         trackGridbag.setConstraints(trackZTextField, constraints);
         trackPanel.add(trackZTextField);



         // Dolly:
         // Row 0:


         JLabel cameraLabel = new JLabel("  Camera Position:  ");
         setConstraints(constraints, 0, 0, 2, GridBagConstraints.WEST);
         dollyGridbag.setConstraints(cameraLabel, constraints);
         dollyPanel.add(cameraLabel);

         dollyCheckBox = new JCheckBox("Dolly", listener.isDolly());
         dollyCheckBox.setName("Dolly");
         dollyCheckBox.addActionListener(this);
         setConstraints(constraints, 2, 0, 1, GridBagConstraints.EAST);
         dollyGridbag.setConstraints(dollyCheckBox, constraints);
         dollyPanel.add(dollyCheckBox);

         dollyCurrentButton = new DollyCurrentButton();
         setConstraints(constraints, 3, 0, 2, GridBagConstraints.WEST);
         dollyGridbag.setConstraints(dollyCurrentButton, constraints);
         dollyPanel.add(dollyCurrentButton);

         // Row 1:

         JLabel camXLabel = new JLabel("  X:  ");
         setConstraints(constraints, 0, 1, 1, GridBagConstraints.WEST);
         dollyGridbag.setConstraints(camXLabel, constraints);
         dollyPanel.add(camXLabel);

         String camXVal = numFormat.format(newCamX);
         camXTextField = new JTextField(camXVal);
         camXTextField.addActionListener(this);
         setConstraints(constraints, 1, 1, 1, GridBagConstraints.WEST);
         dollyGridbag.setConstraints(camXTextField, constraints);
         dollyPanel.add(camXTextField);

         dollyXCheckBox = new JCheckBox("Dolly X", listener.isDollyX());
         dollyXCheckBox.addActionListener(this);
         setConstraints(constraints, 2, 1, 1, GridBagConstraints.EAST);
         dollyGridbag.setConstraints(dollyXCheckBox, constraints);
         dollyPanel.add(dollyXCheckBox);

         JLabel dollyDXLabel = new JLabel("  X Offset:  ");
         setConstraints(constraints, 3, 1, 1, GridBagConstraints.EAST);
         dollyGridbag.setConstraints(dollyDXLabel, constraints);
         dollyPanel.add(dollyDXLabel);

         String dollyDXVal = numFormat.format(newDollyDx);
         dollyXTextField = new JTextField(dollyDXVal);
         dollyXTextField.addActionListener(this);
         setConstraints(constraints, 4, 1, 1, GridBagConstraints.WEST);
         dollyGridbag.setConstraints(dollyXTextField, constraints);
         dollyPanel.add(dollyXTextField);


         // Row 2:

         JLabel camYLabel = new JLabel("  Y:  ");
         setConstraints(constraints, 0, 2, 1, GridBagConstraints.WEST);
         dollyGridbag.setConstraints(camYLabel, constraints);
         dollyPanel.add(camYLabel);

         String camYVal = numFormat.format(newCamY);
         camYTextField = new JTextField(camYVal);
         camYTextField.addActionListener(this);
         setConstraints(constraints, 1, 2, 1, GridBagConstraints.WEST);
         dollyGridbag.setConstraints(camYTextField, constraints);
         dollyPanel.add(camYTextField);

         dollyYCheckBox = new JCheckBox("Dolly Y", listener.isDollyY());
         dollyYCheckBox.addActionListener(this);
         setConstraints(constraints, 2, 2, 1, GridBagConstraints.EAST);
         dollyGridbag.setConstraints(dollyYCheckBox, constraints);
         dollyPanel.add(dollyYCheckBox);

         JLabel dollyDYLabel = new JLabel("  Y Offset:  ");
         setConstraints(constraints, 3, 2, 1, GridBagConstraints.EAST);
         dollyGridbag.setConstraints(dollyDYLabel, constraints);
         dollyPanel.add(dollyDYLabel);

         String dollyDYVal = numFormat.format(newDollyDy);
         dollyYTextField = new JTextField(dollyDYVal);
         dollyYTextField.addActionListener(this);
         setConstraints(constraints, 4, 2, 1, GridBagConstraints.WEST);
         dollyGridbag.setConstraints(dollyYTextField, constraints);
         dollyPanel.add(dollyYTextField);


         // Row 3:

         JLabel camZLabel = new JLabel("  Z:  ");
         setConstraints(constraints, 0, 3, 1, GridBagConstraints.WEST);
         dollyGridbag.setConstraints(camZLabel, constraints);
         dollyPanel.add(camZLabel);

         String camZVal = numFormat.format(newCamZ);
         camZTextField = new JTextField(camZVal);
         camZTextField.addActionListener(this);
         setConstraints(constraints, 1, 3, 1, GridBagConstraints.WEST);
         dollyGridbag.setConstraints(camZTextField, constraints);
         dollyPanel.add(camZTextField);

         dollyZCheckBox = new JCheckBox("Dolly Z", listener.isDollyZ());
         dollyZCheckBox.addActionListener(this);
         setConstraints(constraints, 2, 3, 1, GridBagConstraints.EAST);
         dollyGridbag.setConstraints(dollyZCheckBox, constraints);
         dollyPanel.add(dollyZCheckBox);

         JLabel dollyDZLabel = new JLabel("  Z Offset:  ");
         setConstraints(constraints, 3, 3, 1, GridBagConstraints.EAST);
         dollyGridbag.setConstraints(dollyDZLabel, constraints);
         dollyPanel.add(dollyDZLabel);

         String dollyDZVal = numFormat.format(newDollyDz);
         dollyZTextField = new JTextField(dollyDZVal);
         dollyZTextField.addActionListener(this);
         setConstraints(constraints, 4, 3, 1, GridBagConstraints.WEST);
         dollyGridbag.setConstraints(dollyZTextField, constraints);
         dollyPanel.add(dollyZTextField);


         this.setLayout(new GridLayout(2, 1));
         this.add(trackPanel);
         this.add(dollyPanel);

         this.updateTracking();
         this.updateEnabling();
      }




      private void setConstraints(GridBagConstraints constraints, int gridx, int gridy, int gridwidth, int anchor)
      {
         constraints.gridx = gridx;
         constraints.gridy = gridy;
         constraints.gridwidth = gridwidth;
         constraints.anchor = anchor;
      }

      public void commitChanges()
      {
         updateTracking();
         updateEnabling();

         CameraPropertiesHolder listener = cameraHolder.getCameraPropertiesForActiveCamera();

         listener.setFixX(newFixX);
         listener.setFixY(newFixY);
         listener.setFixZ(newFixZ);

         listener.setCamX(newCamX);
         listener.setCamY(newCamY);
         listener.setCamZ(newCamZ);

         listener.setTracking(trackCheckBox.isSelected());
         listener.setTrackingX(trackXCheckBox.isSelected());
         listener.setTrackingY(trackYCheckBox.isSelected());
         listener.setTrackingZ(trackZCheckBox.isSelected());

         listener.setDolly(dollyCheckBox.isSelected());
         listener.setDollyX(dollyXCheckBox.isSelected());
         listener.setDollyY(dollyYCheckBox.isSelected());
         listener.setDollyZ(dollyZCheckBox.isSelected());

         listener.setTrackingXOffset(newTrackDx);
         listener.setTrackingYOffset(newTrackDy);
         listener.setTrackingZOffset(newTrackDz);

         listener.setDollyXOffset(newDollyDx);
         listener.setDollyYOffset(newDollyDy);
         listener.setDollyZOffset(newDollyDz);

         listener.update();

         GUITrackCheckBox.setSelected(trackCheckBox.isSelected());
         GUIDollyCheckBox.setSelected(dollyCheckBox.isSelected());

         fixXTextField.setText(numFormat.format(listener.getFixX()));
         fixYTextField.setText(numFormat.format(listener.getFixY()));
         fixZTextField.setText(numFormat.format(listener.getFixZ()));

         camXTextField.setText(numFormat.format(listener.getCamX()));
         camYTextField.setText(numFormat.format(listener.getCamY()));
         camZTextField.setText(numFormat.format(listener.getCamZ()));

      }

      public void actionPerformed(ActionEvent event)
      {
         updateTracking();
         updateEnabling();
      }

      private void updateEnabling()
      {
         if (trackCheckBox.isSelected())
         {
            trackXCheckBox.setEnabled(true);
            trackYCheckBox.setEnabled(true);
            trackZCheckBox.setEnabled(true);
            if (trackXCheckBox.isSelected())
               trackXTextField.setEnabled(true);
            else
               trackXTextField.setEnabled(false);
            if (trackYCheckBox.isSelected())
               trackYTextField.setEnabled(true);
            else
               trackYTextField.setEnabled(false);
            if (trackZCheckBox.isSelected())
               trackZTextField.setEnabled(true);
            else
               trackZTextField.setEnabled(false);
         }

         else
         {
            trackXCheckBox.setEnabled(false);
            trackYCheckBox.setEnabled(false);
            trackZCheckBox.setEnabled(false);
            trackXTextField.setEnabled(false);
            trackYTextField.setEnabled(false);
            trackZTextField.setEnabled(false);
         }

         if (dollyCheckBox.isSelected())
         {
            dollyXCheckBox.setEnabled(true);
            dollyYCheckBox.setEnabled(true);
            dollyZCheckBox.setEnabled(true);
            if (dollyXCheckBox.isSelected())
               dollyXTextField.setEnabled(true);
            else
               dollyXTextField.setEnabled(false);
            if (dollyYCheckBox.isSelected())
               dollyYTextField.setEnabled(true);
            else
               dollyYTextField.setEnabled(false);
            if (dollyZCheckBox.isSelected())
               dollyZTextField.setEnabled(true);
            else
               dollyZTextField.setEnabled(false);
         }

         else
         {
            dollyXCheckBox.setEnabled(false);
            dollyYCheckBox.setEnabled(false);
            dollyZCheckBox.setEnabled(false);
            dollyXTextField.setEnabled(false);
            dollyYTextField.setEnabled(false);
            dollyZTextField.setEnabled(false);
         }
      }


      private void updateTracking()
      {
         updateTrackXOffset();
         updateTrackYOffset();
         updateTrackZOffset();

         updateDollyXOffset();
         updateDollyYOffset();
         updateDollyZOffset();

         updateFixX();
         updateFixY();
         updateFixZ();

         updateCamX();
         updateCamY();
         updateCamZ();
      }

      private void updateTrackXOffset()
      {
         newTrackDx = updateTrackDolly(trackXTextField, newTrackDx);
      }

      private void updateTrackYOffset()
      {
         newTrackDy = updateTrackDolly(trackYTextField, newTrackDy);
      }

      private void updateTrackZOffset()
      {
         newTrackDz = updateTrackDolly(trackZTextField, newTrackDz);
      }

      private void updateDollyXOffset()
      {
         newDollyDx = updateTrackDolly(dollyXTextField, newDollyDx);
      }

      private void updateDollyYOffset()
      {
         newDollyDy = updateTrackDolly(dollyYTextField, newDollyDy);
      }

      private void updateDollyZOffset()
      {
         newDollyDz = updateTrackDolly(dollyZTextField, newDollyDz);
      }

      private void updateFixX()
      {
         newFixX = updateTrackDolly(fixXTextField, newFixX);
      }

      private void updateFixY()
      {
         newFixY = updateTrackDolly(fixYTextField, newFixY);
      }

      private void updateFixZ()
      {
         newFixZ = updateTrackDolly(fixZTextField, newFixZ);
      }

      private void updateCamX()
      {
         newCamX = updateTrackDolly(camXTextField, newCamX);
      }

      private void updateCamY()
      {
         newCamY = updateTrackDolly(camYTextField, newCamY);
      }

      private void updateCamZ()
      {
         newCamZ = updateTrackDolly(camZTextField, newCamZ);
      }


      private double updateTrackDolly(JTextField textField, double oldVal)
      {
         double newVal = oldVal;
         String text = textField.getText();

         try
         {
            double val = Double.parseDouble(text);
            newVal = val;
         }
         catch (NumberFormatException e)
         {
            textField.setText(numFormat.format(newVal));
         }

         return newVal;
      }

   }


   public class TrackCurrentButton extends JButton implements ActionListener
   {
      private static final long serialVersionUID = -4680530982540091563L;

      public TrackCurrentButton()
      {
         super("Track Current");
         this.addActionListener(this);
      }

      public void actionPerformed(ActionEvent actionEvent)
      {
         CameraPropertiesHolder cameraPropertiesHolder = cameraHolder.getCameraPropertiesForActiveCamera();

         newTrackDx = -cameraPropertiesHolder.getTrackXVar() + newFixX;
         trackXTextField.setText(numFormat.format(newTrackDx));
         newTrackDy = -cameraPropertiesHolder.getTrackYVar() + newFixY;
         trackYTextField.setText(numFormat.format(newTrackDy));
         newTrackDz = -cameraPropertiesHolder.getTrackZVar() + newFixZ;
         trackZTextField.setText(numFormat.format(newTrackDz));

      }
   }


   public class DollyCurrentButton extends JButton implements ActionListener
   {
      private static final long serialVersionUID = 6700937880902001238L;

      public DollyCurrentButton()
      {
         super("Dolly Current");
         this.addActionListener(this);
      }

      public void actionPerformed(ActionEvent actionEvent)
      {
         CameraPropertiesHolder cameraPropertiesHolder = cameraHolder.getCameraPropertiesForActiveCamera();

         newDollyDx = -cameraPropertiesHolder.getDollyXVar() + newCamX;
         dollyXTextField.setText(numFormat.format(newDollyDx));
         newDollyDy = -cameraPropertiesHolder.getDollyYVar() + newCamY;
         dollyYTextField.setText(numFormat.format(newDollyDy));
         newDollyDz = -cameraPropertiesHolder.getDollyZVar() + newCamZ;
         dollyZTextField.setText(numFormat.format(newDollyDz));
      }
   }
}
