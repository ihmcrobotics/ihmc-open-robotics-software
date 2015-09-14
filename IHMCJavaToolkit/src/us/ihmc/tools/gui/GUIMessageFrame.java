package us.ihmc.tools.gui;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.JTabbedPane;

public class GUIMessageFrame
{
   private static final boolean SHOW_GUI_MESSAGE_FRAME = false;

   private final SimpleDateFormat simpleDateFormat = new SimpleDateFormat();

   private final JFrame messageWindow;
   private final JTabbedPane jTabbedPane;

   private final int errorMessagePanelIndex, outMessagePanelIndex, parameterMessagePanelIndex;

   private final JCheckBox lockBox = new JCheckBox("Lock Focus");
   private boolean locked = false;
   private int lastMessagePanel = 0;

   private ArrayList<Integer> listOfPanels = new ArrayList<Integer>();

   private static GUIMessageFrame elvis;

   public GUIMessageFrame(String messageWindowName)
   {      

      if (SHOW_GUI_MESSAGE_FRAME)
      {
         messageWindow = new JFrame(messageWindowName);
         jTabbedPane = new JTabbedPane();

         messageWindow.getContentPane().setLayout(new BorderLayout());
         messageWindow.getContentPane().add(jTabbedPane); // ,BorderLayout.CENTER);
         messageWindow.getContentPane().add(lockBox, BorderLayout.SOUTH);

         // jTabbedPane.setSize(100, 100);
         lockBox.addActionListener(new ActionListener()
         {
            public void actionPerformed(ActionEvent e)
            {
               if (locked)
               {
                  locked = false;
                  jTabbedPane.setSelectedIndex(lastMessagePanel);
               } else
               {
                  locked = true;
               }

            }
         });

         GUIMessagePanel messagePanel;

         String outStatusString = "Out";
         messagePanel = new GUIMessagePanel(outStatusString);
         jTabbedPane.add(outStatusString, messagePanel);
         outMessagePanelIndex = jTabbedPane.indexOfTab(outStatusString);
         listOfPanels.add(outMessagePanelIndex);

         String errorStatusString = "Error";
         messagePanel = new GUIMessagePanel(errorStatusString);
         jTabbedPane.add(errorStatusString, messagePanel);
         errorMessagePanelIndex = jTabbedPane.indexOfTab(errorStatusString);
         listOfPanels.add(errorMessagePanelIndex);

         String parameterStatusString = "Parameter";
         messagePanel = new GUIMessagePanel(parameterStatusString);
         jTabbedPane.add(parameterStatusString, messagePanel);
         parameterMessagePanelIndex = jTabbedPane.indexOfTab(parameterStatusString);
         listOfPanels.add(parameterMessagePanelIndex);

         messageWindow.setSize(new Dimension(900, 800));
         messageWindow.setLocation(250, 0);

         if (SHOW_GUI_MESSAGE_FRAME)
            messageWindow.setVisible(true);

      }
      
      else
      {
         this.parameterMessagePanelIndex = 0;
         this.outMessagePanelIndex = 0;
         this.errorMessagePanelIndex = 0;
         
         this.messageWindow = null;
         this.jTabbedPane = null;
      }
   }

   public static GUIMessageFrame getInstance()
   {
      if (elvis == null)
      {
         elvis = new GUIMessageFrame("GUI Message Frame");
      }

      return elvis;
   }


   public void appendOutMessage(String outMessage)
   {
      appendMessageToPanel(outMessagePanelIndex, outMessage);
   }

   public void appendErrorMessage(String error)
   {
      appendMessageToPanel(errorMessagePanelIndex, error);
   }

   public void appendParameterMessage(String message)
   {
      appendMessageToPanel(parameterMessagePanelIndex, message);
   }

   public void popupErrorMessage(String error)
   {
      JOptionPane.showMessageDialog(messageWindow, error, "Error", JOptionPane.ERROR_MESSAGE);
   }
   
   public void popupWarningMessage(String warning)
   {
      JOptionPane.showMessageDialog(messageWindow, warning, "Warning", JOptionPane.WARNING_MESSAGE);
   }

   public void popupInformationMessage(String message)
   {
      JOptionPane.showMessageDialog(messageWindow, message, "Message", JOptionPane.INFORMATION_MESSAGE);
   }

   public void appendMessageToPanel(int panelIndex, String message)
   {
      appendMessageToPanel(panelIndex, message, Color.BLACK);
   }

   private String prependTimeToMessage(String message)
   {
      String time = computeTime();
      message = time + message;

      return message;
   }

   public String computeTime()
   {
      return simpleDateFormat.format(new Date()) + ": ";
   }

   public void appendMessageToPanel(int panelIndex, String message, Color color)
   {
      if (SHOW_GUI_MESSAGE_FRAME)
      {
         message = prependTimeToMessage(message);

         GUIMessagePanel gUIMessagePanel = (GUIMessagePanel) jTabbedPane.getComponentAt(panelIndex);
         gUIMessagePanel.appendMessage(message, color);
         if (!locked)
            jTabbedPane.setSelectedIndex(panelIndex);
         lastMessagePanel = panelIndex;
      }
   }

   public int createGUIMessagePanel(String name)
   {
      if (SHOW_GUI_MESSAGE_FRAME)
      {
         GUIMessagePanel ret = new GUIMessagePanel(name);

         int checkIndex = jTabbedPane.indexOfTab(name);
         if (checkIndex != -1)
         {
            throw new RuntimeException("This name is already taken: " + name);
         } else
            jTabbedPane.addTab(name, ret);

         int integerValueToReturn = jTabbedPane.indexOfTab(name);

         listOfPanels.add(integerValueToReturn);

         return integerValueToReturn;
      }
      else
      {
         return 0;
      }
   }

   public void reset()
   {
      for (Integer panelIndexValue : listOfPanels)
      {
         GUIMessagePanel gUIMessagePanel = null;

         gUIMessagePanel = (GUIMessagePanel) jTabbedPane.getComponentAt(panelIndexValue);

         gUIMessagePanel.clear();
      }
   }
   
   public void save(String filename)
   {
      File file = new File(filename);
      save(file);
   }
   
   
   public void save(File file)
   {
      PrintWriter printWriter;
      
      try
      {
         printWriter = new PrintWriter(file);
      } 
      catch (FileNotFoundException e)
      {
         System.err.println("File not found : " + file);
         return;
      }
      
//      printWriter.println(listOfPanels.size());
      
      for (int panelIndex = 0; panelIndex < listOfPanels.size(); panelIndex++)
      {
         GUIMessagePanel guiMessagePanel = (GUIMessagePanel) jTabbedPane.getComponentAt(panelIndex);
         
         String name = guiMessagePanel.getName();
         
         printWriter.println(name);
         printWriter.println("{\\StartGUIMessagePanelText}");
         String text = guiMessagePanel.getText();
         printWriter.write(text);
         printWriter.println("{\\EndGUIMessagePanelText}");

      }
      
      printWriter.flush();
      printWriter.close();
   }

   
   public static void main(String[] args)
   {
      int sleepTime = 1000;

      GUIMessageFrame guiMessageFrame = GUIMessageFrame.getInstance();

      guiMessageFrame.appendErrorMessage("This is an error message");
      guiMessageFrame.appendOutMessage("This is an out message.");
      guiMessageFrame.appendParameterMessage("01 This is a parameter message.");

      int testPanelIndex = guiMessageFrame.createGUIMessagePanel("Test Panel");

      sleep(sleepTime);

    
      for (int i = 0; i < 10; i++)
      {
         guiMessageFrame.appendMessageToPanel(testPanelIndex, i + " new message", Color.red);
         
         sleep(sleepTime);
      }

      //    GlobalParameter globalParameterDouble = new GlobalParameter("testParametwerwer wer werer wer werweerDouble", -10.0, gUIMessageFrame);
      //    GlobalParameter globalParameterBoolean = new GlobalParameter("testParameterBoolean", YoVariableType.BOOLEAN, true, gUIMessageFrame);
      //    GlobalParameter globalParameterInteger = new GlobalParameter("testParameterInteger", YoVariableType.INT, 2, gUIMessageFrame);
      //    GlobalParameter globalParameterEnum = new GlobalParameter("testParameterEnum", YoVariableType.ENUM, TestEnum.askdjaskdjasdjadajsdajsdjkasdlkj, gUIMessageFrame);

      //    System.out.println("testParameterDouble" + " max number for display = " + globalParameterDouble.getMaximumNumberOfCharactersInValue());
      //    System.out.println("testParameterBoolean" + " max number for display = " + globalParameterBoolean.getMaximumNumberOfCharactersInValue());
      //    System.out.println("testParameterInteger" + " max number for display = " + globalParameterInteger.getMaximumNumberOfCharactersInValue());
      //    System.out.println("testParameterEnum" + " max number for display = " + globalParameterEnum.getMaximumNumberOfCharactersInValue());

      //    System.out.println(Double.MIN_VALUE);

      //    for(int i=0; i<10; i++)
      //    {
      //       globalParameterDouble.setDoubleValue((double) i, " changing, i=" + i + " a whole bunch of stuff to print here so that we go over the edge of the window.");
      //    }

      //    globalParameterBoolean.setBooleanValue(false, "hi");

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
      //    GUIMessageFrame.reset();

   }

   private static void sleep(long sleepMillis)
   {
      try
      {
         Thread.sleep(sleepMillis);
      } 
      catch (InterruptedException e)
      {
      }
   }

}
