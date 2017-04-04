package us.ihmc.simulationConstructionSetTools.util.gui;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Font;

import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.ScrollPaneConstants;

public class YoGUIMessagePanel extends JPanel
{
   /**
    *
    */
   private static final long serialVersionUID = 1715136701882332438L;
   @SuppressWarnings("unused")
   private final String name;
   private final JTextArea textArea;
   private final JScrollPane jScrollPane;
   private boolean QUIET = false;

   private static final int WIDTH = 75;

   protected YoGUIMessagePanel(String name)
   {
      this.name = name;
      this.setLayout(new BorderLayout());
      textArea = new JTextArea();
      textArea.setLineWrap(true);

      // textArea.setColumns(WIDTH);
      // textArea.setRows(44);
      textArea.setSelectedTextColor(Color.WHITE);
      jScrollPane = new JScrollPane();
      jScrollPane.getViewport().add(textArea);
      textArea.setEditable(false);
      textArea.setFont(new Font("Monospaced", Font.PLAIN, 12));

      jScrollPane.setVerticalScrollBarPolicy(ScrollPaneConstants.VERTICAL_SCROLLBAR_ALWAYS);

      this.add(jScrollPane, BorderLayout.CENTER);
   }

   public void clear()
   {
//    int lastLine = textArea.getLineCount() - 1;
//    textArea.replaceRange(" ", 0, lastLine);

      textArea.setText(" ");
   }

   public void setQuiet(boolean quiet)
   {
      this.QUIET = quiet;
   }

   public void appendMessage(String message)
   {
      if (QUIET)
         return;

//    textArea.append(message + "\n");

      if (message.length() > (WIDTH - 1 + 25))
         message = message + "\n";

      textArea.insert(message + "\n", 0);
   }

   public void appendMessage(String message, Color color)
   {
      if (QUIET)
         return;

//    textArea.setForeground(color);
//    textArea.setSelectedTextColor(color);
      textArea.setSelectionColor(color);

//    textArea.append(message + "\n");

      if (message.length() > (WIDTH - 1 + 25))
         message = message + "\n";

      textArea.insert(message + "\n", 0);
   }


   public static void main(String[] args)
   {
      YoGUIMessagePanel gUIMessagePanel = new YoGUIMessagePanel("Test");
      gUIMessagePanel.appendMessage("Hi there", Color.RED);
      gUIMessagePanel.appendMessage("Hi there");
   }


}
