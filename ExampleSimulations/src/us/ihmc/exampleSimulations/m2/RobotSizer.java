package us.ihmc.exampleSimulations.m2;

//import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.Toolkit;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.FocusAdapter;
import java.awt.event.FocusEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Properties;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;

public class RobotSizer extends JFrame
{
   private static final long serialVersionUID = 4412795235391101513L;

   private ImagePanel panel;

   private final JTextField upperArmWeight = new JTextField();
   private final JTextField lowerArmWeight = new JTextField();
   private final JTextField upperLegWeight = new JTextField();
   private final JTextField lowerLegLength = new JTextField();
   private final JTextField headWeight = new JTextField();
   private final JTextField bodyWeight = new JTextField();
   private final JTextField pelvisWeight = new JTextField();
   private final JTextField footWeight = new JTextField();
   private final JTextField shoulderSpacing = new JTextField();
   private final JTextField bodyHeight = new JTextField();
   private final JTextField headHeight = new JTextField();
   private final JTextField upperArmLength = new JTextField();
   private final JTextField lowerArmLength = new JTextField();
   private final JTextField pelvisHeight = new JTextField();
   private final JTextField pelviswidth = new JTextField();
   private final JTextField lowerLeLength = new JTextField();
   private final JTextField upperLegLength = new JTextField();
   private final JTextField footWidth = new JTextField();
   private final JTextField footLength = new JTextField();

   Properties props = new Properties();
   File propertiesFile = new File("properties/robotProperties.properties");

   private final JLabel totalWeightLabel = new JLabel();
   private final JTextField totalWeight = new JTextField();
   private final JLabel upperBodyLabel = new JLabel();
   private final JLabel lowerBodyLabel = new JLabel();
   private final JTextField upperBody = new JTextField();
   private final JTextField lowerBody = new JTextField();
   private final JLabel totalHeightLabel = new JLabel();
   private final JLabel upperHeightLabel = new JLabel();
   private final JLabel lowerHeightLabel = new JLabel();
   private final JTextField totalHeight = new JTextField();
   private final JTextField upperHeight = new JTextField();
   private final JTextField lowerHeight = new JTextField();

   public RobotSizer()
   {
      this.setTitle("Robot Sizer");
      this.setSize(1224 + 6, 816 + 25);
      this.setResizable(false);

      try
      {
         jbInit();
      }
      catch (Throwable e)
      {
         e.printStackTrace();
      }

      fillValues();
      updateTotals();
      this.setVisible(true);
      Toolkit toolkit = Toolkit.getDefaultToolkit();


      Dimension scrnsize = toolkit.getScreenSize();

      setLocation(scrnsize.width / 2 - this.getWidth() / 2, scrnsize.height / 2 - this.getHeight() / 2);
      this.setDefaultCloseOperation(EXIT_ON_CLOSE);
      this.addWindowListener(new WindowAdapter()
      {
         public void windowClosing(WindowEvent e)
         {
            System.out.println("Saving Values");
            saveValues();
         }

      });

   }

   private void fillValues()
   {
      try
      {
         File propertiesDir = new File("properties");
         if (!propertiesDir.exists())
            propertiesDir.mkdir();

         props.load(new FileInputStream(propertiesFile));
         upperArmWeight.setText(props.getProperty("upperArmWeight"));
         lowerArmWeight.setText(props.getProperty("lowerArmWeight"));
         upperLegWeight.setText(props.getProperty("upperLegWeight"));
         lowerLegLength.setText(props.getProperty("lowerLegLength"));
         headWeight.setText(props.getProperty("headWeight"));
         bodyWeight.setText(props.getProperty("bodyWeight"));
         pelvisWeight.setText(props.getProperty("pelvisWeight"));
         footWeight.setText(props.getProperty("footWeight"));
         shoulderSpacing.setText(props.getProperty("shoulderSpacing"));
         bodyHeight.setText(props.getProperty("bodyHeight"));
         headHeight.setText(props.getProperty("headHeight"));
         upperArmLength.setText(props.getProperty("upperArmLength"));
         lowerArmLength.setText(props.getProperty("lowerArmLength"));
         pelvisHeight.setText(props.getProperty("pelvisHeight"));
         pelviswidth.setText(props.getProperty("pelviswidth"));
         lowerLeLength.setText(props.getProperty("lowerLeLength"));
         upperLegLength.setText(props.getProperty("upperLegLength"));
         footWidth.setText(props.getProperty("footWidth"));
         footLength.setText(props.getProperty("footLength"));

      }
      catch (Exception e)
      {
      }

   }

   private void saveValues()
   {
      try
      {
         props.setProperty("upperArmWeight", upperArmWeight.getText());
         props.setProperty("lowerArmWeight", lowerArmWeight.getText());
         props.setProperty("upperLegWeight", upperLegWeight.getText());
         props.setProperty("lowerLegLength", lowerLegLength.getText());
         props.setProperty("headWeight", headWeight.getText());
         props.setProperty("bodyWeight", bodyWeight.getText());
         props.setProperty("pelvisWeight", pelvisWeight.getText());
         props.setProperty("footWeight", footWeight.getText());
         props.setProperty("shoulderSpacing", shoulderSpacing.getText());
         props.setProperty("bodyHeight", bodyHeight.getText());
         props.setProperty("headHeight", headHeight.getText());
         props.setProperty("upperArmLength", upperArmLength.getText());
         props.setProperty("lowerArmLength", lowerArmLength.getText());
         props.setProperty("pelvisHeight", pelvisHeight.getText());
         props.setProperty("pelviswidth", pelviswidth.getText());
         props.setProperty("lowerLeLength", lowerLeLength.getText());
         props.setProperty("upperLegLength", upperLegLength.getText());
         props.setProperty("footWidth", footWidth.getText());
         props.setProperty("footLength", footLength.getText());
         props.store(new FileOutputStream(propertiesFile), "This file contains teh properties of the biped robot");

      }
      catch (FileNotFoundException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      catch (IOException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
   }

   private void jbInit() throws Exception
   {
      File tmp = new File("images\\robot2.jpg");

      // tmp.createNewFile();
      System.out.println(tmp.exists());
      panel = new ImagePanel(new ImageIcon(tmp.getPath()).getImage());
      getContentPane().add(panel);

      panel.setLayout(null);

      int lowerAmmount = 30;
      panel.add(totalWeightLabel);
      totalWeightLabel.setText("Total Weight");
      totalWeightLabel.setBounds(340, 701 + lowerAmmount, 71, 16);

      panel.add(totalWeight);
      totalWeight.setBounds(417, 699 + lowerAmmount, 87, 20);

      panel.add(upperBodyLabel);
      upperBodyLabel.setText("Upper Body");
      upperBodyLabel.setBounds(340, 723 + lowerAmmount, 71, 16);

      panel.add(lowerBodyLabel);
      lowerBodyLabel.setText("Lower Body");
      lowerBodyLabel.setBounds(340, 745 + lowerAmmount, 71, 16);

      panel.add(upperBody);
      upperBody.setBounds(417, 721 + lowerAmmount, 87, 20);

      panel.add(lowerBody);
      lowerBody.setBounds(417, 743 + lowerAmmount, 87, 20);

      panel.add(totalHeightLabel);
      totalHeightLabel.setText("Total Height");
      totalHeightLabel.setBounds(578, 701 + lowerAmmount, 80, 16);

      panel.add(upperHeightLabel);
      upperHeightLabel.setText("Upper Height");
      upperHeightLabel.setBounds(578, 723 + lowerAmmount, 80, 16);

      panel.add(lowerHeightLabel);
      lowerHeightLabel.setText("Lower Height");
      lowerHeightLabel.setBounds(578, 745 + lowerAmmount, 80, 16);

      panel.add(totalHeight);

      totalHeight.setBounds(664, 699 + lowerAmmount, 87, 20);

      panel.add(upperHeight);
      upperHeight.setBounds(664, 721 + lowerAmmount, 87, 20);

      panel.add(lowerHeight);
      lowerHeight.setBounds(664, 743 + lowerAmmount, 87, 20);
      panel.setLayout(null);

      panel.add(upperArmWeight);
      upperArmWeight.setName("upperArmWeight");
      upperArmWeight.addFocusListener(new TextFieldFocusListener());
      upperArmWeight.addActionListener(new ValueUpdateListener());
      upperArmWeight.setText("000.00");
      upperArmWeight.setBounds(70, 115, 87, 20);

      panel.add(lowerArmWeight);
      lowerArmWeight.setName("lowerArmWeight");
      lowerArmWeight.addActionListener(new ValueUpdateListener());
      lowerArmWeight.addFocusListener(new TextFieldFocusListener());
      lowerArmWeight.setText("000.00");
      lowerArmWeight.setBounds(120, 355, 87, 20);

      panel.add(upperLegWeight);
      upperLegWeight.setName("upperLegWeight");
      upperLegWeight.addActionListener(new ValueUpdateListener());
      upperLegWeight.addFocusListener(new TextFieldFocusListener());
      upperLegWeight.setText("000.00");
      upperLegWeight.setBounds(75, 515, 87, 20);

      panel.add(lowerLegLength);
      lowerLegLength.setName("lowerLegLength");
      lowerLegLength.addActionListener(new ValueUpdateListener());
      lowerLegLength.addFocusListener(new TextFieldFocusListener());
      lowerLegLength.setText("000.00");
      lowerLegLength.setBounds(70, 635, 87, 20);

      panel.add(headWeight);
      headWeight.setName("headWeight");
      headWeight.addActionListener(new ValueUpdateListener());
      headWeight.addFocusListener(new TextFieldFocusListener());
      headWeight.setText("000.00");
      headWeight.setBounds(355, 80, 87, 20);

      panel.add(bodyWeight);
      bodyWeight.setName("bodyWeight");
      bodyWeight.addActionListener(new ValueUpdateListener());
      bodyWeight.addFocusListener(new TextFieldFocusListener());
      bodyWeight.setText("000.00");
      bodyWeight.setBounds(365, 255, 87, 20);

      panel.add(pelvisWeight);
      pelvisWeight.setName("pelvisWeight");
      pelvisWeight.addActionListener(new ValueUpdateListener());
      pelvisWeight.addFocusListener(new TextFieldFocusListener());
      pelvisWeight.setText("000.00");
      pelvisWeight.setBounds(400, 425, 87, 20);

      panel.add(footWeight);
      footWeight.setName("footWeight");
      footWeight.addActionListener(new ValueUpdateListener());
      footWeight.addFocusListener(new TextFieldFocusListener());
      footWeight.setText("000.00");
      footWeight.setBounds(290, 679, 87, 20);

      panel.add(shoulderSpacing);
      shoulderSpacing.setName("shoulderSpacing");
      shoulderSpacing.addActionListener(new ValueUpdateListener());
      shoulderSpacing.addFocusListener(new TextFieldFocusListener());
      shoulderSpacing.setText("000.00");
      shoulderSpacing.setBounds(690, 45, 87, 20);

      panel.add(bodyHeight);
      bodyHeight.setName("bodyHeight");
      bodyHeight.addActionListener(new ValueUpdateListener());
      bodyHeight.addFocusListener(new TextFieldFocusListener());
      bodyHeight.setText("000.00");
      bodyHeight.setBounds(694, 202, 87, 20);

      panel.add(headHeight);
      headHeight.setName("headHeight");
      headHeight.addActionListener(new ValueUpdateListener());
      headHeight.addFocusListener(new TextFieldFocusListener());
      headHeight.setText("000.00");
      headHeight.setBounds(1004, 60, 87, 20);

      panel.add(upperArmLength);
      upperArmLength.setName("upperArmLength");
      upperArmLength.addActionListener(new ValueUpdateListener());
      upperArmLength.addFocusListener(new TextFieldFocusListener());
      upperArmLength.setText("000.00");
      upperArmLength.setBounds(1073, 127, 87, 20);

      panel.add(lowerArmLength);
      lowerArmLength.setName("lowerArmLength");
      lowerArmLength.addActionListener(new ValueUpdateListener());
      lowerArmLength.addFocusListener(new TextFieldFocusListener());
      lowerArmLength.setText("000.00");
      lowerArmLength.setBounds(1020, 335, 87, 20);

      panel.add(pelvisHeight);
      pelvisHeight.setName("pelvisHeight");
      pelvisHeight.addActionListener(new ValueUpdateListener());
      pelvisHeight.addFocusListener(new TextFieldFocusListener());
      pelvisHeight.setText("000.00");
      pelvisHeight.setBounds(716, 393, 87, 20);

      panel.add(pelviswidth);
      pelviswidth.setName("pelviswidth");
      pelviswidth.addActionListener(new ValueUpdateListener());
      pelviswidth.addFocusListener(new TextFieldFocusListener());
      pelviswidth.setText("000.00");
      pelviswidth.setBounds(756, 486, 87, 20);

      panel.add(lowerLeLength);
      lowerLeLength.setName("lowerLeLength");
      lowerLeLength.addActionListener(new ValueUpdateListener());
      lowerLeLength.addFocusListener(new TextFieldFocusListener());
      lowerLeLength.setText("000.00");
      lowerLeLength.setBounds(755, 655, 87, 20);

      panel.add(upperLegLength);
      upperLegLength.setName("upperLegLength");
      upperLegLength.addActionListener(new ValueUpdateListener());
      upperLegLength.addFocusListener(new TextFieldFocusListener());
      upperLegLength.setText("000.00");
      upperLegLength.setBounds(1073, 474, 81, 16);

      panel.add(footWidth);
      footWidth.setName("footWidth");
      footWidth.addActionListener(new ValueUpdateListener());
      footWidth.addFocusListener(new TextFieldFocusListener());
      footWidth.setText("000.00");
      footWidth.setBounds(1100, 670, 81, 16);

      panel.add(footLength);
      footLength.setName("footLength");
      footLength.addActionListener(new ValueUpdateListener());
      footLength.addFocusListener(new TextFieldFocusListener());
      footLength.setText("000.00");
      footLength.setBounds(1100, 775, 87, 20);
   }

   class ImagePanel extends JPanel
   {
	  private static final long serialVersionUID = 8048037125805015251L;
	  private Image img;

      public ImagePanel(String img)
      {
         this(new ImageIcon(img).getImage());
      }

      public ImagePanel(Image img)
      {
         this.img = img;
         Dimension size = new Dimension(img.getWidth(null), img.getHeight(null));
         setPreferredSize(size);
         setMinimumSize(size);
         setMaximumSize(size);
         setSize(size);
         setLayout(null);
      }

      public void paintComponent(Graphics g)
      {
         g.drawImage(img, 0, 0, null);
      }

   }


   public static void main(String[] args)
   {
      new RobotSizer();
   }

   private class ValueUpdateListener implements ActionListener
   {
      public void actionPerformed(ActionEvent arg0)
      {
         boolean validInput = false;
         try
         {
            new Double(((JTextField) arg0.getSource()).getText());
            validInput = true;

         }
         catch (Exception e)
         {
            System.out.println("invalid Value " + "-" + ((JTextField) arg0.getSource()).getName() + "-");

            ((JTextField) arg0.getSource()).setText(props.getProperty(((JTextField) arg0.getSource()).getName()));
         }

         if (validInput)
         {
            updateTotals();

         }
      }
   }


   private class TextFieldFocusListener extends FocusAdapter
   {
      public void focusLost(FocusEvent arg0)
      {
         boolean validInput = false;
         try
         {
            new Double(((JTextField) arg0.getSource()).getText());
            validInput = true;

         }
         catch (Exception e)
         {
            System.out.println("invalid Value " + "-" + ((JTextField) arg0.getSource()).getName() + "-");

            ((JTextField) arg0.getSource()).setText(props.getProperty(((JTextField) arg0.getSource()).getName()));
         }

         if (validInput)
         {
            updateTotals();

         }
      }
   }


   private void updateTotals()
   {
      double totalWeightVal = 0, upperWeightVal = 0, lowerWeightVal = 0;
      double totalHeightVal = 0, upperHeightVal = 0, lowerHeightVal = 0;

      upperWeightVal += new Double(upperArmWeight.getText()) * 2;

      upperWeightVal += new Double(lowerArmWeight.getText()) * 2;

      lowerWeightVal += new Double(upperLegWeight.getText()) * 2;

      lowerWeightVal += new Double(lowerLegLength.getText()) * 2;

      upperWeightVal += new Double(headWeight.getText());

      upperWeightVal += new Double(bodyWeight.getText());

      upperWeightVal += new Double(pelvisWeight.getText());

      lowerWeightVal += new Double(footWeight.getText()) * 2;

      upperHeightVal += new Double(bodyHeight.getText());

      upperHeightVal += new Double(headHeight.getText());

      upperHeightVal += new Double(pelvisHeight.getText());

      lowerHeightVal += new Double(lowerLeLength.getText());

      lowerHeightVal += new Double(upperLegLength.getText());

      totalWeightVal = upperWeightVal + lowerWeightVal;
      totalHeightVal = upperHeightVal + lowerHeightVal;

      totalHeight.setText(totalHeightVal + "");
      upperHeight.setText(upperHeightVal + "");
      lowerHeight.setText(lowerHeightVal + "");

      totalWeight.setText(totalWeightVal + "");
      upperBody.setText(upperWeightVal + "");
      lowerBody.setText(lowerWeightVal + "");
   }

   protected void textField_focusLost(FocusEvent arg0)
   {
   }
}
