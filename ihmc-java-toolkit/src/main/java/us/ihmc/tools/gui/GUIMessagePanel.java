package us.ihmc.tools.gui;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Font;

import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.ScrollPaneConstants;

public class GUIMessagePanel extends JPanel
{
   private static final long serialVersionUID = 627707062057513407L;

   private final String name;
   private final JTextArea textArea;
   private final JScrollPane jScrollPane;
   private boolean quiet = false;

   private static final int WIDTH = 75;

   public GUIMessagePanel(String name)
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

   @Override
   public String getName()
   {
      return name;
   }

   public void clear()
   {
      textArea.setText(" ");
   }

   public void setQuiet(boolean quiet)
   {
      this.quiet = quiet;
   }
   
   public String getText()
   {
      return textArea.getText();
   }

   public void appendMessage(String message)
   {
      if (quiet)
         return;

      //    textArea.append(message + "\n");

      if (message.length() > (WIDTH - 1 + 25))
         message = message + "\n";

      textArea.insert(message + "\n", 0);
   }

   public void appendMessage(String message, Color color)
   {
      if (quiet)
         return;

      //    textArea.setForeground(color);
      //    textArea.setSelectedTextColor(color);
      textArea.setSelectionColor(color);

      //    textArea.append(message + "\n");

      if (message.length() > (WIDTH - 1 + 25))
         message = message + "\n";

      textArea.insert(message + "\n", 0);
   }

}
