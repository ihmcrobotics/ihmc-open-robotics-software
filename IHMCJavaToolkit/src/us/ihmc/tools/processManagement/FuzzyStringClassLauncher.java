package us.ihmc.tools.processManagement;

import java.awt.AWTException;
import java.awt.Component;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.SystemTray;
import java.awt.TrayIcon;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.io.File;
import java.io.IOException;
import java.lang.reflect.Method;
import java.net.URL;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.HashMap;

import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.JPopupMenu;
import javax.swing.JTextField;
import javax.swing.JTree;
import javax.swing.SwingUtilities;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;
import javax.swing.tree.DefaultMutableTreeNode;
import javax.swing.tree.DefaultTreeCellRenderer;

import org.apache.lucene.analysis.standard.StandardAnalyzer;
import org.apache.lucene.document.Document;
import org.apache.lucene.document.Field;
import org.apache.lucene.index.DirectoryReader;
import org.apache.lucene.index.IndexWriter;
import org.apache.lucene.index.IndexWriterConfig;
import org.apache.lucene.queryparser.classic.ParseException;
import org.apache.lucene.queryparser.classic.QueryParser;
import org.apache.lucene.search.IndexSearcher;
import org.apache.lucene.search.Query;
import org.apache.lucene.search.ScoreDoc;
import org.apache.lucene.search.TopScoreDocCollector;
import org.apache.lucene.store.Directory;
import org.apache.lucene.store.RAMDirectory;
import org.apache.lucene.util.Version;

/**
 * Because FuzzyStringLauncher utilizes JavaProcessSpawner under the hood, which in turn
 * works its magic by inheriting its classpath from wherever it is invoked, you should
 * "embed" the FuzzyStringClassLauncher somewhere that has dependencies on other projects
 * and jars containing the main methods that you'll most likely want to invoke.
 * 
 * An example would be subclassing, or even just putting an instance of the launcher inside
 * of another class as a field.  An example is the subclass DRCFuzzyStringClassLauncher, which
 * is in the DRC project.  This has dependencies on most of our other projects and so it has a huge
 * classpath, meaning most of our main classes can be launched from that instance of the launcher.
 * 
 * @author dstephen
 */

public class FuzzyStringClassLauncher
{
   private HashMap<String, Class<?>> classList = new HashMap<String, Class<?>>();

   private JFrame frame;
   private JPanel searchPanel;
   private JPanel resultsPanel;
   private JTextField searchField = new JTextField();

   private QueryParser queryParser;
   private IndexSearcher searcher;

   private TopScoreDocCollector collector;

   private String name;

   private ImageIcon javaIcon = new ImageIcon(FuzzyStringClassLauncher.class.getClassLoader().getResource("java.png"));

   public FuzzyStringClassLauncher(String name)
   {
      this.name = name;

      System.out.println("Indexing classes...");
      initClassList();
      System.out.println("Indexing complete.");
      try
      {
         initLucene();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void start()
   {
      SwingUtilities.invokeLater(new Runnable()
      {
         @Override
         public void run()
         {
            setupJFrame();
            //            showJFrame();
         }
      });
   }

   private void setupJFrame()
   {
      frame = new JFrame(name);
      searchPanel = new JPanel(new GridBagLayout());
      searchPanel.setBorder(BorderFactory.createEmptyBorder());

      GridBagConstraints c = new GridBagConstraints();
      Insets oldInsets = c.insets;

      c.gridx = 0;
      c.gridy = 0;
      c.anchor = GridBagConstraints.PAGE_START;
      c.ipadx = 500;
      c.weightx = 0;
      c.insets = new Insets(30, 30, 30, 30);
      c.fill = GridBagConstraints.HORIZONTAL;

      searchField = new JTextField();
      searchField.setColumns(5);
      setupSearchFieldListener();

      searchPanel.add(searchField, c);

      c.insets = oldInsets;

      resultsPanel = new JPanel(new GridBagLayout());
      resultsPanel.setBorder(BorderFactory.createEmptyBorder());
      resultsPanel.setVisible(false);

      frame.getContentPane().setLayout(new BoxLayout(frame.getContentPane(), BoxLayout.Y_AXIS));
      frame.getContentPane().add(searchPanel);
      frame.getContentPane().add(resultsPanel);

      SystemTray tray = SystemTray.getSystemTray();
      TrayIcon icon = new TrayIcon(javaIcon.getImage());

      final JPopupMenu menu = new JPopupMenu("Fuzzy String Class Launcher");
      JMenuItem item = new JMenuItem("Quit");
      menu.add(item);

      item.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            System.exit(0);
         }
      });      

      icon.addMouseListener(new MouseListener()
      {

         @Override
         public void mouseReleased(MouseEvent e)
         {

         }

         @Override
         public void mousePressed(MouseEvent e)
         {

         }

         @Override
         public void mouseExited(MouseEvent e)
         {

         }

         @Override
         public void mouseEntered(MouseEvent e)
         {

         }

         @Override
         public void mouseClicked(final MouseEvent e)
         {
            if (SwingUtilities.isRightMouseButton(e))
            {
               menu.setLocation(e.getX(), 0);
               menu.setInvoker(menu);
               menu.setVisible(true);
            }
            else if (SwingUtilities.isLeftMouseButton(e))
            {
               if (frame.isVisible())
               {
                  searchField.setText("");
                  frame.setVisible(false);
               }
               else
               {
                  SwingUtilities.invokeLater(new Runnable()
                  {
                     @Override
                     public void run()
                     {
                        frame.setLocation(e.getX() - frame.getWidth() / 2, 10);
                        frame.setVisible(true);                        
                     }
                  });
               }
            }
         }
      });

      try
      {
         tray.add(icon);
      }
      catch (AWTException e1)
      {
         e1.printStackTrace();
      }

//      frame.setAutoRequestFocus(true);
      frame.setUndecorated(true);
      frame.pack();
      frame.setResizable(false);
      frame.setDefaultCloseOperation(JFrame.HIDE_ON_CLOSE);
      frame.setLocationRelativeTo(null);
      frame.setAlwaysOnTop(true);
   }
   
   private void setupSearchFieldListener()
   {
      searchField.getDocument().addDocumentListener(new DocumentListener()
      {
         @Override
         public void removeUpdate(DocumentEvent e)
         {
            if (searchField.getText().length() == 0)
            {
               disposeSearchResults();
            }
            else
            {
               updateSearchResults();
            }
         }

         @Override
         public void insertUpdate(DocumentEvent e)
         {
            if (searchField.getText().length() == 0)
            {
               disposeSearchResults();
            }
            else
            {
               updateSearchResults();
            }
         }

         @Override
         public void changedUpdate(DocumentEvent e)
         {
            if (searchField.getText().length() == 0)
            {
               disposeSearchResults();
            }
            else
            {
               updateSearchResults();
            }
         }
      });
   }

   private void initClassList()
   {
      ArrayList<File> dirs = new ArrayList<File>();
      ArrayList<String> allClasses = new ArrayList<String>();
      try
      {
         getSubdirsFromRoots(dirs);

         getAllClasses(dirs, allClasses);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      populateMainClassListFromClasses(allClasses);
   }

   private void populateMainClassListFromClasses(ArrayList<String> allClasses)
   {
      for (String s : allClasses)
      {
         try
         {
            Class<?> clazz = Class.forName(s, false, this.getClass().getClassLoader());

            for (Method m : clazz.getMethods())
            {
               if (m.getName().equals("main"))
               {
                  classList.put(clazz.getSimpleName(), clazz);
               }
            }
         }
         catch (ClassNotFoundException e)
         {
            e.printStackTrace();
         }
      }
   }

   private void getAllClasses(ArrayList<File> dirs, ArrayList<String> classList) throws IOException
   {
      final String binString = "bin" + File.separator;
      final String classesString = "classes" + File.separator;
      final String mainString = "main" + File.separator;
      final String testString = "test" + File.separator;

      for (File dir : dirs)
      {
         for (File f : dir.listFiles())
         {
            String fileName = f.getCanonicalFile().getAbsolutePath();
            if (fileName.contains(binString))
            {
               fileName = fileName.substring(fileName.indexOf(binString) +binString.length());
            }
            if (fileName.contains(classesString))
            {
               fileName = fileName.substring(fileName.indexOf(classesString) + classesString.length());
            }
            if(fileName.startsWith(mainString))
            {
               fileName = fileName.substring(fileName.indexOf(mainString) + mainString.length());
            }
            else if(fileName.startsWith(testString))
            {
               fileName = fileName.substring(fileName.indexOf(testString) + testString.length());
            }

            fileName = fileName.replace(File.separator, ".");

            if (!f.isDirectory() && !fileName.contains("$") && fileName.endsWith(".class"))
            {
               classList.add(fileName.substring(0, fileName.indexOf(".class")));
            }
         }
      }
   }

   private void getSubdirsFromRoots(ArrayList<File> dirs) throws IOException
   {
      Enumeration<URL> roots = this.getClass().getClassLoader().getResources("");
      while (roots.hasMoreElements())
      {
         String nextPath = roots.nextElement().getPath();
         dirs.add(new File(nextPath));
         getAllSubDirs(nextPath, dirs);
      }
   }

   private void getAllSubDirs(String path, ArrayList<File> dirs) throws IOException
   {
      File root = null;
      root = new File(path).getCanonicalFile();

      for (File f : root.listFiles())
      {
         if (f.isDirectory())
         {
            dirs.add(f);
            getAllSubDirs(f.getCanonicalPath(), dirs);
         }
      }
   }

   private void initLucene() throws IOException
   {
      StandardAnalyzer analyzer = new StandardAnalyzer(Version.LUCENE_43);
      Directory index = new RAMDirectory();

      IndexWriterConfig config = new IndexWriterConfig(Version.LUCENE_43, analyzer);
      IndexWriter writer = new IndexWriter(index, config);

      for (String s : classList.keySet())
      {
         Document doc = new Document();
         doc.add(new org.apache.lucene.document.TextField("classname", s, Field.Store.YES));
         writer.addDocument(doc);
      }

      writer.close();

      queryParser = new QueryParser(Version.LUCENE_43, "classname", analyzer);
      searcher = new IndexSearcher(DirectoryReader.open(index));
   }

   private void updateSearchResults()
   {
      collector = TopScoreDocCollector.create(15, true);
      resultsPanel.removeAll();
      final JTree tree = new JTree(new DefaultMutableTreeNode());
      tree.setRowHeight(0);
      tree.setCellRenderer(new DefaultTreeCellRenderer()
      {
         private static final long serialVersionUID = 1L;

         @Override
         public Component getTreeCellRendererComponent(JTree tree, Object value, boolean sel, boolean expanded, boolean leaf, int row, boolean hasFocus)
         {
            JLabel label = (JLabel) super.getTreeCellRendererComponent(tree, value, sel, expanded, leaf, row, hasFocus);
            label.setBorder(BorderFactory.createEmptyBorder((row == 1 ? 0 : 10), 10, 10, 20));
            return label;
         }
      });
      tree.setShowsRootHandles(false);
      ((DefaultTreeCellRenderer) tree.getCellRenderer()).setOpenIcon(null);
      ((DefaultTreeCellRenderer) tree.getCellRenderer()).setClosedIcon(null);
      ((DefaultTreeCellRenderer) tree.getCellRenderer()).setLeafIcon(javaIcon);
      tree.addMouseListener(new MouseListener()
      {

         @Override
         public void mouseReleased(MouseEvent e)
         {

         }

         @Override
         public void mousePressed(MouseEvent e)
         {

         }

         @Override
         public void mouseExited(MouseEvent e)
         {

         }

         @Override
         public void mouseEntered(MouseEvent e)
         {

         }

         @Override
         public void mouseClicked(MouseEvent e)
         {
            if (e.getClickCount() > 1)
            {
               String str = tree.getSelectionPath().getLastPathComponent().toString();
               if (str != null && !str.toLowerCase().contains("no results"))
               {
                  str = str.substring(str.indexOf("\">") + 2, str.indexOf("</"));
                  new JavaProcessSpawner(false, true).spawn(classList.get(str), new String[] { "-Xms2048m", "-Xmx2048m" }, null);
               }
            }
         }
      });
      try
      {
         String searchString = searchField.getText().substring(0, 1) + searchField.getText().substring(1, searchField.getText().length()).replace("", "*");
         Query q = queryParser.parse(searchString + "~");
         searcher.search(q, collector);

         ScoreDoc[] hits = collector.topDocs().scoreDocs;

         for (ScoreDoc hit : hits)
         {
            Document d = searcher.doc(hit.doc);
            DefaultMutableTreeNode newNode = new DefaultMutableTreeNode("<html><body><span style=\"font-weight:bold;font-size:1.2em;\">" + d.get("classname")
                  + "</span></body></html>");
            ((DefaultMutableTreeNode) tree.getModel().getRoot()).add(newNode);
         }
         if (hits.length == 0)
         {
            DefaultMutableTreeNode newNode = new DefaultMutableTreeNode(
                  "<html><body><span style=\"font-weight:bold;font-size:1.2em;\">No Results</span></body></html>");
            ((DefaultMutableTreeNode) tree.getModel().getRoot()).add(newNode);
         }
         GridBagConstraints c = new GridBagConstraints();
         tree.setBorder(BorderFactory.createEtchedBorder());
         tree.expandRow(0);
         c.insets = new Insets(10, 10, 10, 10);
         //         c.ipadx = 100;
         c.fill = GridBagConstraints.BOTH;
         c.weightx = 1;
         resultsPanel.add(tree, c);
         resultsPanel.validate();
         resultsPanel.repaint();
         resultsPanel.setVisible(true);
         frame.pack();
         frame.validate();
         frame.repaint();
      }
      catch (ParseException e)
      {
         e.printStackTrace();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

   }

   private void disposeSearchResults()
   {
      resultsPanel.setVisible(false);
      frame.pack();
      frame.validate();
   }
}
