package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import java.awt.BorderLayout;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.Toolkit;
import java.awt.event.ActionEvent;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.BoxLayout;
import javax.swing.ImageIcon;
import javax.swing.JFileChooser;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.ScrollPaneConstants;

public class TextTool extends JPanel
{
   private static JPanel menuBar;
   private static JPanel textPanel;
   private static JTextArea textArea;
   private static JMenuBar jMenuBar;

   private JFileChooser dialog = new JFileChooser(System.getProperty("user.dir"));
   private String currentFile = "Untitled";
   private boolean changed = false;

   public TextTool()
   {
      menuBar = new JPanel(new FlowLayout(FlowLayout.LEFT));
      textArea = new JTextArea(18, 109);
      textPanel = new JPanel();
      jMenuBar = new JMenuBar();
      JMenu file = new JMenu("File");
      JMenu find = new JMenu("Find");

      textArea.setFont(new Font("Monospaced", Font.PLAIN, 14));
      JScrollPane scroller = new JScrollPane(textArea);
      scroller.setVerticalScrollBarPolicy(ScrollPaneConstants.VERTICAL_SCROLLBAR_ALWAYS);
      scroller.setHorizontalScrollBarPolicy(ScrollPaneConstants.HORIZONTAL_SCROLLBAR_ALWAYS);
      textPanel.add(scroller);

      file.add(Open);
      file.add(Save);
      file.add(Quit);
      file.add(SaveAs);
      file.addSeparator();

      Save.setEnabled(false);
      SaveAs.setEnabled(false);
      textArea.addKeyListener(k1);

      jMenuBar.add(file);
      jMenuBar.add(find);
      menuBar.add(jMenuBar);
      setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
      add(menuBar, BorderLayout.WEST);
      add(textPanel);
      setVisible(true);
   }

   private KeyListener k1 = new KeyAdapter()
   {
      public void keyPressed(KeyEvent e)
      {
         changed = true;
         Save.setEnabled(true);
         SaveAs.setEnabled(true);
      }
   };

   Action Open = new AbstractAction("Open", new ImageIcon("open.gif"))
   {
      public void actionPerformed(ActionEvent e)
      {
         saveOld();
         if (dialog.showOpenDialog(null) == JFileChooser.APPROVE_OPTION)
         {
            readInFile(dialog.getSelectedFile().getAbsolutePath());
         }
         SaveAs.setEnabled(true);
      }
   };

   Action Save = new AbstractAction("Save", new ImageIcon("save.gif"))
   {
      public void actionPerformed(ActionEvent e)
      {
         if (!currentFile.equals("Untitled"))
            saveFile(currentFile);
         else
            saveFileAs();
      }
   };

   Action SaveAs = new AbstractAction("Save as...")
   {
      public void actionPerformed(ActionEvent e)
      {
         saveFileAs();
      }
   };

   Action Quit = new AbstractAction("Quit")
   {
      public void actionPerformed(ActionEvent e)
      {
         saveOld();
         System.exit(0);
      }
   };

   Action find = new AbstractAction("Find Element")
   {
      public void actionPerformed(ActionEvent e)
      {
         find();
      }
   };

   //   ActionMap m = textArea.getActionMap();
   //   Action Cut = m.get(DefaultEditorKit.cutAction);
   //   Action Copy = m.get(DefaultEditorKit.copyAction);
   //   Action Paste = m.get(DefaultEditorKit.pasteAction);

   public void openTableData(String path)
   {
      readInFile(path);
   }

   private void saveFileAs()
   {
      if (dialog.showSaveDialog(null) == JFileChooser.APPROVE_OPTION)
         saveFile(dialog.getSelectedFile().getAbsolutePath());
   }

   private void saveOld()
   {
      if (changed)
      {
         if (JOptionPane.showConfirmDialog(this, "Would you like to save " + currentFile + " ?", "Save", JOptionPane.YES_NO_OPTION) == JOptionPane.YES_OPTION)
            saveFile(currentFile);
      }
   }

   private void find()
   {
      System.out.println("Find button has been pressed");
      //JFrame frame = new JFrame();
      JOptionPane.showInputDialog(null, "Enter something to search", 1);
      //String search = (String)JOptionPane.showInputDialog(frame, "Enter something to search", "Find");
   }

   private void readInFile(String fileName)
   {
      try
      {
         FileReader r = new FileReader(fileName);
         textArea.read(r, null);
         r.close();
         currentFile = fileName;
         changed = false;
      }
      catch (IOException e)
      {
         Toolkit.getDefaultToolkit().beep();
         JOptionPane.showMessageDialog(this, "Editor can't find the file called " + fileName);
      }
   }

   private void saveFile(String fileName)
   {
      try
      {
         FileWriter w = new FileWriter(fileName);
         textArea.write(w);
         w.close();
         currentFile = fileName;
         changed = false;
         Save.setEnabled(false);
      }
      catch (IOException e)
      {
      }
   }

}