package us.ihmc.simulationConstructionSetTools.util.gui;


import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.text.DecimalFormat;
import java.util.ArrayList;

import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.JTabbedPane;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationConstructionSetTools.util.globalParameters.BooleanGlobalParameter;
import us.ihmc.simulationConstructionSetTools.util.globalParameters.DoubleGlobalParameter;
import us.ihmc.simulationConstructionSetTools.util.globalParameters.EnumGlobalParameter;
import us.ihmc.simulationConstructionSetTools.util.globalParameters.GlobalParameter;
import us.ihmc.simulationConstructionSetTools.util.globalParameters.GlobalParameterChangedListener;
import us.ihmc.simulationConstructionSetTools.util.globalParameters.IntGlobalParameter;

public class YoGUIMessageFrame implements GlobalParameterChangedListener
{
   private static final DecimalFormat decimal3DPrintFormatter = new DecimalFormat("0.000");

   private static final int CHARACTERS_IN_VALUE_STRING = 12;
   public static boolean showMessagePanel = true;
   public static String messageWindowName = "GUI Message Frame";

   private static JFrame parentFrame;
   private static boolean initialized = false;

// private static final JFrame messageWindow = new JFrame("LittleDog Messages");
   private static JFrame messageWindow;
   private static final JTabbedPane jTabbedPane = new JTabbedPane();


// private static int trialMessagePanelIndex;
   private static int errorMessagePanelIndex;

// private static int terrainMessagePanelIndex;
   private int parameterMessagePanelIndex;
   private boolean createdParameterMessagePanel = false;

   private static JCheckBox lockBox = new JCheckBox("Lock Focus");
   private static boolean locked = false;
   private static int lastMessagePanel = 0;

   private static ArrayList<Integer> listOfPanels = new ArrayList<Integer>();

   private static YoGUIMessageFrame elvis;

   private DoubleYoVariable time;

   private YoGUIMessageFrame()
   {
   }

   public static YoGUIMessageFrame getInstance()
   {
      initialize();

      return elvis;
   }

   public static void setTimeVariable(DoubleYoVariable time)
   {
      initialize();

      elvis.time = time;
   }


   public void createChangeListener(String tabName)
   {
      if (createdParameterMessagePanel)
      {
         System.out.println("Already created change listener");

         return;
      }

      YoGUIMessagePanel messagePanel;
      messagePanel = new YoGUIMessagePanel(tabName);
      jTabbedPane.add(tabName, messagePanel);
      parameterMessagePanelIndex = jTabbedPane.indexOfTab(tabName);
      listOfPanels.add(parameterMessagePanelIndex);

      createdParameterMessagePanel = true;
   }


   private static void initialize()
   {
      if (initialized)
         return;

      if (elvis != null)
      {
         throw new RuntimeException("Should never get here!");
      }

      elvis = new YoGUIMessageFrame();

//    LittleDogOutput.parentFrame = parentFrame;

      messageWindow = new JFrame(messageWindowName);
      messageWindow.getContentPane().setLayout(new BorderLayout());
      messageWindow.getContentPane().add(jTabbedPane);    // ,BorderLayout.CENTER);
      messageWindow.getContentPane().add(lockBox, BorderLayout.SOUTH);

      // jTabbedPane.setSize(100, 100);
      lockBox.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            if (locked)
            {
               locked = false;
               jTabbedPane.setSelectedIndex(lastMessagePanel);
            }
            else
            {
               locked = true;
            }

         }
      });

      YoGUIMessagePanel messagePanel;


//    String trialStatusString = "Trial Status";
//    messagePanel = new GUIMessagePanel(trialStatusString);
//    jTabbedPane.add(trialStatusString, messagePanel);
//    trialMessagePanelIndex = jTabbedPane.indexOfTab(trialStatusString);
//    listOfPanels.add(trialMessagePanelIndex);

      String errorStatusString = "Error";
      messagePanel = new YoGUIMessagePanel(errorStatusString);
      jTabbedPane.add(errorStatusString, messagePanel);
      errorMessagePanelIndex = jTabbedPane.indexOfTab(errorStatusString);
      listOfPanels.add(errorMessagePanelIndex);

//    String terrainStatusString = "Terrain";
//    messagePanel = new GUIMessagePanel(terrainStatusString);
//    jTabbedPane.add(terrainStatusString, messagePanel);
//    terrainMessagePanelIndex = jTabbedPane.indexOfTab(terrainStatusString);
//    listOfPanels.add(terrainMessagePanelIndex);




      messageWindow.setSize(new Dimension(900, 800));
      messageWindow.setLocation(250, 0);

      if (showMessagePanel)
      {
         messageWindow.setVisible(true);
      }
      else
         messageWindow.setVisible(false);

      initialized = true;
   }


// public static void appendTrialMessage(String message)
// {
//    if (!initialized)
//       initialize();
//
//    appendMessageToPanel(trialMessagePanelIndex, message);
// }
//
// public static void appendErrorMessage(String error)
// {
//    if (!initialized)
//       initialize();
//
//    appendMessageToPanel(errorMessagePanelIndex, error);
// }
//
// public static void appendTerrainMessage(String error)
// {
//    if (!initialized)
//       initialize();
//
//    appendMessageToPanel(terrainMessagePanelIndex, error);
// }
//
//
   private void appendParameterMessage(String message)
   {
      if (!createdParameterMessagePanel)
         createChangeListener("Parameter");

      appendMessageToPanel(parameterMessagePanelIndex, message);
   }


// public static void popupErrorMessage(String error)
// {
//    if (!initialized)
//       initialize();
//    GUIMessagePanel gUIMessagePanel = (GUIMessagePanel) jTabbedPane.getComponentAt(trialMessagePanelIndex);
//    gUIMessagePanel.appendMessage("Error: " + error);
//
//    JOptionPane.showMessageDialog(parentFrame, error, "Error", JOptionPane.ERROR_MESSAGE);
// }

   public static void popupWarningMessage(String warning)
   {
      if (!initialized)
         initialize();

      JOptionPane.showMessageDialog(parentFrame, warning, "Warning", JOptionPane.WARNING_MESSAGE);
   }

   public static void popupInformationMessage(String message)
   {
      if (!initialized)
         initialize();

      JOptionPane.showMessageDialog(parentFrame, message, "Message", JOptionPane.INFORMATION_MESSAGE);
   }


   public static void appendMessageToPanel(int panelIndex, String message)
   {
      appendMessageToPanel(panelIndex, message, Color.BLACK);
   }

   public static void appendMessageToPanel(int panelIndex, String message, Color color)
   {
      if (!initialized)
         initialize();

      // make sure that panel index is valid
      // IndexOutOfBoundsException
      YoGUIMessagePanel gUIMessagePanel = null;

//    String time;
//    if (LittleDogGlobalParameters.hasInstanceBeenCreated())
//       time = SpecialFunctions.getRobotTime();
//    else
//       time = "";
//    message = time + ": " + message;

      if (elvis.time != null)
      {
         message = decimal3DPrintFormatter.format(elvis.time.getDoubleValue()) + ": " + message;
      }

      try
      {
         gUIMessagePanel = (YoGUIMessagePanel) jTabbedPane.getComponentAt(panelIndex);
         gUIMessagePanel.appendMessage(message, color);
         if (!locked)
            jTabbedPane.setSelectedIndex(panelIndex);
         lastMessagePanel = panelIndex;
      }
      catch (IndexOutOfBoundsException indexOutOfBoundsException)
      {
         throw indexOutOfBoundsException;
      }
   }

   public static int createGUIMessagePanel(String name)
   {
      if (!initialized)
         initialize();

      YoGUIMessagePanel ret = new YoGUIMessagePanel(name);


      int checkIndex = jTabbedPane.indexOfTab(name);
      if (checkIndex != -1)
      {
         throw new RuntimeException("This name is already taken: " + name);
      }
      else
         jTabbedPane.addTab(name, ret);


      int integerValueToReturn = jTabbedPane.indexOfTab(name);

      listOfPanels.add(integerValueToReturn);

      return integerValueToReturn;
   }

   public static void reset()
   {
      for (Integer panelIndexValue : listOfPanels)
      {
         YoGUIMessagePanel gUIMessagePanel = null;

         gUIMessagePanel = (YoGUIMessagePanel) jTabbedPane.getComponentAt(panelIndexValue);

         gUIMessagePanel.clear();
      }
   }

   @Override
   public void globalParameterCreated(GlobalParameter globalParameter)
   {
      appendParameterMessage(globalParameter.getShortName() + "= " + padWithSpacesOrTrim(globalParameter.getValueInStringFormat(), CHARACTERS_IN_VALUE_STRING)
                             + " created");
   }


   @Override
   public void booleanValueChanged(GlobalParameter globalParameter, String comment, boolean previousValue, boolean newValue)
   {
      appendParameterMessage(globalParameter.getShortName() + " GlobalParameter changed from   "
                             + padWithSpacesOrTrim(Boolean.toString(previousValue), CHARACTERS_IN_VALUE_STRING) + " to   "
                             + padWithSpacesOrTrim(Boolean.toString(newValue), CHARACTERS_IN_VALUE_STRING) + " " + comment);
   }

   @Override
   public void doubleValueChanged(GlobalParameter globalParameter, String comment, double previousValue, double newValue)
   {
      appendParameterMessage(globalParameter.getShortName() + " GlobalParameter changed from   "
                             + padWithSpacesOrTrim(Double.toString(previousValue), CHARACTERS_IN_VALUE_STRING) + " to   "
                             + padWithSpacesOrTrim(Double.toString(newValue), CHARACTERS_IN_VALUE_STRING) + " " + comment);
   }

   @Override
   public void integerValueChanged(GlobalParameter globalParameter, String comment, int previousValue, int newValue)
   {
      appendParameterMessage(globalParameter.getShortName() + " GlobalParameter changed from   "
                             + padWithSpacesOrTrim(Integer.toString(previousValue), CHARACTERS_IN_VALUE_STRING) + " to   "
                             + padWithSpacesOrTrim(Integer.toString(newValue), CHARACTERS_IN_VALUE_STRING) + " " + comment);
   }

   @Override
   public void enumValueChanged(GlobalParameter globalParameter, String comment, Enum<?> previousValue, Enum<?> newValue)
   {
      appendParameterMessage(globalParameter.getShortName() + " GlobalParameter changed from   "
                             + padWithSpacesOrTrim(previousValue.toString(), CHARACTERS_IN_VALUE_STRING) + " to   "
                             + padWithSpacesOrTrim(newValue.toString(), CHARACTERS_IN_VALUE_STRING) + " " + comment);
   }


// CHARACTERS_IN_VALUE_STRING

   private String padWithSpacesOrTrim(String string, int sizeToBe)
   {
      if (string.length() == sizeToBe)
         return string;

      if (string.length() > sizeToBe)
         return string.substring(0, sizeToBe);

      StringBuffer padded = new StringBuffer(string);
      while (padded.length() < sizeToBe)
      {
         padded.append(" ");
      }

      return padded.toString();
   }


   public static void main(String[] args)
   {
      int sleepTime = 100;

      YoGUIMessageFrame yoGUIMessageFrame = YoGUIMessageFrame.getInstance();

//    YoGUIMessageFrame.appendMessageToPanel("01 This is an error");

      int junkPanelIndex = YoGUIMessageFrame.createGUIMessagePanel("Junk Panel");

      try
      {
         Thread.sleep(sleepTime);
      }
      catch (InterruptedException ex)
      {
      }


      YoGUIMessageFrame.appendMessageToPanel(junkPanelIndex, "01 new message");

      try
      {
         Thread.sleep(sleepTime);
      }
      catch (InterruptedException ex)
      {
      }

//    YoGUIMessageFrame.appendTrialMessage("02 This is an ");

      try
      {
         Thread.sleep(sleepTime);
      }
      catch (InterruptedException ex)
      {
      }

      for (int i = 0; i < 100; i++)
      {
         YoGUIMessageFrame.appendMessageToPanel(junkPanelIndex, i + " new message", Color.red);
      }



      GlobalParameter globalParameterDouble = new DoubleGlobalParameter("testParametwerwer wer werer wer werweerDouble", "", -10.0, yoGUIMessageFrame);
      GlobalParameter globalParameterBoolean = new BooleanGlobalParameter("testParameterBoolean", "", true, yoGUIMessageFrame);
      GlobalParameter globalParameterInteger = new IntGlobalParameter("testParameterInteger", "", 2, yoGUIMessageFrame);
      GlobalParameter globalParameterEnum = new EnumGlobalParameter("testParameterEnum", "", TestEnum.askdjaskdjasdjadajsdajsdjkasdlkj, yoGUIMessageFrame);

      System.out.println("testParameterDouble" + " max number for display = " + globalParameterDouble.getMaximumNumberOfCharactersInValue());
      System.out.println("testParameterBoolean" + " max number for display = " + globalParameterBoolean.getMaximumNumberOfCharactersInValue());
      System.out.println("testParameterInteger" + " max number for display = " + globalParameterInteger.getMaximumNumberOfCharactersInValue());
      System.out.println("testParameterEnum" + " max number for display = " + globalParameterEnum.getMaximumNumberOfCharactersInValue());

      for (int i = 0; i < 10; i++)
      {
         ((DoubleGlobalParameter) globalParameterDouble).set(i,
                 " changing, i=" + i + " a whole bunch of stuff to print here so that we go over the edge of the window.");
      }

      ((BooleanGlobalParameter) globalParameterBoolean).set(false, "hi");

//    try
//    {
//       Thread.sleep(10000);
//    }
//    catch (InterruptedException ex1)
//    {
//    }
//
//    System.out.println("Clearing");
//
//
//    YoGUIMessageFrame.reset();

   }

   private enum TestEnum {A, B, C, WEWER, askdjaskdjasdjadajsdajsdjkasdlkj}

}
