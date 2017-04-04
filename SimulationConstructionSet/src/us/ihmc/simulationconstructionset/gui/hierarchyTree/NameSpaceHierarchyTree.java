package us.ihmc.simulationconstructionset.gui.hierarchyTree;

import java.awt.Component;
import java.awt.Graphics;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.FocusEvent;
import java.awt.event.FocusListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.io.File;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.LinkedHashMap;
import java.util.StringTokenizer;

import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JMenuItem;
import javax.swing.JPopupMenu;
import javax.swing.JScrollPane;
import javax.swing.JTree;
import javax.swing.filechooser.FileFilter;
import javax.swing.tree.DefaultMutableTreeNode;
import javax.swing.tree.DefaultTreeModel;
import javax.swing.tree.TreeNode;
import javax.swing.tree.TreePath;
import javax.swing.tree.TreeSelectionModel;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.commands.WriteDataCommandExecutor;
import us.ihmc.simulationconstructionset.gui.EventDispatchThreadHelper;
import us.ihmc.simulationconstructionset.gui.ForcedRepaintPopupMenu;
import us.ihmc.simulationconstructionset.gui.RegularExpression;
import us.ihmc.simulationconstructionset.gui.CreatedNewRegistriesListener;
import us.ihmc.simulationconstructionset.gui.RegistrySettingsChangedListener;
import us.ihmc.simulationconstructionset.util.SimpleFileReader;
import us.ihmc.simulationconstructionset.util.SimpleFileWriter;

public class NameSpaceHierarchyTree extends JScrollPane implements MouseListener, FocusListener, CreatedNewRegistriesListener
{
   private static final long serialVersionUID = -8489413083414697473L;
   private JTree tree;
   private JFileChooser fileChooser = new JFileChooser();
   private DefaultTreeModel model;
   private DefaultMutableTreeNode top;
   private LinkedHashMap<YoVariableRegistry, DefaultMutableTreeNode> registryTreeNodeMap = new LinkedHashMap<YoVariableRegistry, DefaultMutableTreeNode>();
   private LinkedHashMap<DefaultMutableTreeNode, YoVariableRegistry> treeNodeRegistryMap = new LinkedHashMap<DefaultMutableTreeNode, YoVariableRegistry>();
   private final JFrame frame;
   private final WriteDataCommandExecutor writeDataCommandExecutor;

   private final RegistrySelectedListener registrySelectedListener;

   protected JPopupMenu popupMenu;

   private final YoVariableRegistry root;
   private ArrayList<RegistrySettingsChangedListener> registrySettingsChangedListeners = new ArrayList<RegistrySettingsChangedListener>();

   public NameSpaceHierarchyTree(RegistrySelectedListener nameSpaceSelectedListener, JFrame frame, WriteDataCommandExecutor writeDataCommandExecutor, YoVariableRegistry rootRegistry)
   {
      super();
      
      EventDispatchThreadHelper.checkThatInEventDispatchThread();

      this.registrySelectedListener = nameSpaceSelectedListener;
      this.root = rootRegistry;
      this.topOfTreeRegistry = root;
      this.frame = frame;
      this.writeDataCommandExecutor = writeDataCommandExecutor;

      top = new DefaultMutableTreeNode(rootRegistry.getNameSpace().getName());
      model = new DefaultTreeModel(top);
      tree = new JTree();
      tree.setModel(model);
      tree.setShowsRootHandles(true);
      this.add(tree);
      this.setViewportView(tree);
      tree.setRootVisible(false);
      tree.getSelectionModel().setSelectionMode(TreeSelectionModel.DISCONTIGUOUS_TREE_SELECTION);
      tree.addMouseListener(this);
      tree.setToggleClickCount(0);

      initPopupMenu();
      setUpTree(rootRegistry);

      tree.setCellRenderer(new NameSpaceHierarchyNodeRenderer(treeNodeRegistryMap));
   }
   
   private String filterText = "";
   public void filter(String filterText)
   {
      this.filterText = filterText;
      createdNewRegistries();
   }


   private boolean needToSetupTree = true;
   private YoVariableRegistry topOfTreeRegistry;
   
   @Override
   public void paintComponent(Graphics g)
   {
      if (needToSetupTree)
      {
         setUpTree(topOfTreeRegistry);
         needToSetupTree = false;
      }
      
      super.paintComponent(g);
   }
   
   private void setUpTree(final YoVariableRegistry currentregistry)
   {
      //TODO: This is incorrect but exo breaks when using invokelater for some reason.
      // Also, reads registry childeren without locks.
//      EventDispatchThreadHelper.justRun(new Runnable(){
//
      //         public void run()
      //         {
      EventDispatchThreadHelper.checkThatInEventDispatchThread();


      addNode(currentregistry);

      for (YoVariableRegistry child : currentregistry.getChildren())
      {
         setUpTree(child);
      }        
      //         }});
   }

   private void initPopupMenu()
   {
      popupMenu = new ForcedRepaintPopupMenu();
      JMenuItem toggleVaribleSend = new JMenuItem("Toggle Variable Send");
      toggleVaribleSend.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            toggleVariableSend(e);
            popupMenu.setVisible(false);
         }
      });
      popupMenu.add(toggleVaribleSend);
      
      JMenuItem recursiveToggleVaribleSend = new JMenuItem("Recursive Toggle Variable Send");
      recursiveToggleVaribleSend.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            recursiveToggleVariableSend(e);
            popupMenu.setVisible(false);
         }
      });
      popupMenu.add(recursiveToggleVaribleSend);

      JMenuItem toggleVariableLog = new JMenuItem("Toggle Variable Log");
      toggleVariableLog.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            toggleVariableLog(e);
            popupMenu.setVisible(false);
         }
      });
      popupMenu.add(toggleVariableLog);
      
      JMenuItem recursiveToggleVariableLog = new JMenuItem("Recursive Toggle Variable Log");
      recursiveToggleVariableLog.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            recursiveToggleVariableLog(e);
            popupMenu.setVisible(false);
         }
      });
      popupMenu.add(recursiveToggleVariableLog);
      
      JMenuItem exportBinaryRegistryData = new JMenuItem("Export to binary format");
      exportBinaryRegistryData.addActionListener(new ActionListener()
      {
         
         @Override
         public void actionPerformed(ActionEvent e)
         {
            exportRegistryData(e, true);
            popupMenu.setVisible(false);
         }
      });
      popupMenu.add(exportBinaryRegistryData);
      
      JMenuItem exportASCIIRegistryData = new JMenuItem("Export to ASCII format");
      exportASCIIRegistryData.addActionListener(new ActionListener()
      {
         
         @Override
         public void actionPerformed(ActionEvent e)
         {
            exportRegistryData(e, false);
            popupMenu.setVisible(false);
         }
      });
      popupMenu.add(exportASCIIRegistryData);


      JMenuItem save = new JMenuItem("Save Configuration");
      save.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            saveConfiguration();
            popupMenu.setVisible(false);
         }
      });
      popupMenu.add(save);

      JMenuItem load = new JMenuItem("Load Configuration");
      load.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            loadConfiguration();
            popupMenu.setVisible(false);
         }
      });
      popupMenu.add(load);


      popupMenu.addFocusListener(this);

      // tree.addFocusListener(this);
      // this.addFocusListener(this);
   }

   private void addNode(YoVariableRegistry registry)
   {
      EventDispatchThreadHelper.checkThatInEventDispatchThread();
      
      if (!tree.isExpanded(new TreePath(top.getPath())))
      {
         tree.expandPath(new TreePath(top.getPath()));
      }
      
      boolean match = RegularExpression.check(registry.getNameSpace().getName(), filterText);
      if(match)
      {
         // create the top of the tree
         if (registry.getParent() == null || !registryTreeNodeMap.containsKey(registry.getParent()))
         {
            DefaultMutableTreeNode newTopLevelAncestor = new DefaultMutableTreeNode(registry.getNameSpace().getShortName());
            registryTreeNodeMap.put(registry, newTopLevelAncestor);
            treeNodeRegistryMap.put(newTopLevelAncestor, registry);
            top.add(newTopLevelAncestor);
            
            model.setRoot(top);
         }
         else
         {
            DefaultMutableTreeNode newChild = new DefaultMutableTreeNode(registry.getNameSpace().getShortName());
            registryTreeNodeMap.put(registry, newChild);
            treeNodeRegistryMap.put(newChild, registry);
            registryTreeNodeMap.get(registry.getParent()).add(newChild);
         }
      }
   }


   public void showNameSpace(YoVariableRegistry registry)
   {
      if (registryTreeNodeMap.containsKey(registry))
      {
         TreePath pathToNameSpace = new TreePath(registryTreeNodeMap.get(registry).getPath());

         tree.expandPath(pathToNameSpace);
         tree.scrollPathToVisible(pathToNameSpace);
         tree.setSelectionPath(pathToNameSpace);
      }
      else
      {
         System.err.println("Warning: " + registry.getNameSpace().getName() + " not found.");
      }
   }

   @Override
   public void mouseClicked(MouseEvent arg0)
   {
      if (arg0.getClickCount() == 2)
      {
         TreePath treePath = tree.getPathForLocation(arg0.getX(), arg0.getY());
         if ((treePath != null) && (treePath.getPathCount() >= 1))
         {
            if (registryTreeNodeMap.containsValue(treePath.getLastPathComponent()))
            {
               YoVariableRegistry registry = treeNodeRegistryMap.get(treePath.getLastPathComponent());
               registrySelectedListener.registryWasSelected(registry);
            }
         }
      }

   }

   @Override
   public void mouseEntered(MouseEvent arg0)
   {
   }

   @Override
   public void mouseExited(MouseEvent arg0)
   {
   }

   @Override
   public void mousePressed(MouseEvent arg0)
   {
      if (arg0.isPopupTrigger())
      {
         popupMenu.show((Component) arg0.getSource(), arg0.getX(), arg0.getY());
      }
   }

   @Override
   public void mouseReleased(MouseEvent arg0)
   {
      if (arg0.isPopupTrigger())
      {
         popupMenu.show((Component) arg0.getSource(), arg0.getX(), arg0.getY());
      }
   }

   private void recursiveToggleVariableSend(ActionEvent arg0)
   {
      boolean toggleSend = true;
      boolean toggleLog = false; 
      boolean recursive = true;
      
      toggleVariableSendOrLog(arg0, toggleSend, toggleLog, recursive);
   }
   
   private void toggleVariableSend(ActionEvent arg0)
   {
      boolean toggleSend = true;
      boolean toggleLog = false; 
      boolean recursive = false;
      
      toggleVariableSendOrLog(arg0, toggleSend, toggleLog, recursive);
   }
   
   private void recursiveToggleVariableLog(ActionEvent arg0)
   {
      boolean toggleSend = false;
      boolean toggleLog = true; 
      boolean recursive = true;
      
      toggleVariableSendOrLog(arg0, toggleSend, toggleLog, recursive);
   }

   private void toggleVariableLog(ActionEvent arg0)
   {
      boolean toggleSend = false;
      boolean toggleLog = true; 
      boolean recursive = false;
      
      toggleVariableSendOrLog(arg0, toggleSend, toggleLog, recursive);
   }
   
   private void toggleVariableSendOrLog(ActionEvent arg0, boolean toggleSend, boolean toggleLog, boolean recursive)
   {
      TreePath[] selectionPaths = tree.getSelectionPaths();
      ArrayList<YoVariableRegistry> changedRegistries = new ArrayList<YoVariableRegistry>();
      for (int i = 0; i < selectionPaths.length; i++)
      {
         TreePath path = selectionPaths[i];
         TreeNode parentNode = (TreeNode) path.getLastPathComponent();
        
         YoVariableRegistry parentRegistry = treeNodeRegistryMap.get(parentNode);
         boolean send = !parentRegistry.isSent();
         boolean log = !parentRegistry.isLogged();

        recursiveToggleVariableSendOrLog(recursive, toggleSend, send, toggleLog, log, parentNode, parentRegistry, changedRegistries);
      }

      notifyRegistrySettingsChangedListeners(changedRegistries);
   }
   
   
   private void exportRegistryData(ActionEvent e, final boolean binary)
   {
      TreePath[] selectionPaths = tree.getSelectionPaths();
      ArrayList<YoVariable<?>> allVariables = new ArrayList<YoVariable<?>>();

      for (int i = 0; i < selectionPaths.length; i++)
      {
         TreePath path = selectionPaths[i];
         TreeNode parentNode = (TreeNode) path.getLastPathComponent();
        
         YoVariableRegistry parentRegistry = treeNodeRegistryMap.get(parentNode);
         allVariables.addAll(parentRegistry.getAllVariables());
      }
      
      FileFilter filterFilter = new FileFilter()
      {

         @Override
         public String getDescription()
         {
            if (binary)
               return ".data.gz";
            else
               return ".m";
         }

         @Override
         public boolean accept(File f)
         {
            if (f.isDirectory())
               return true;

            if (f.getName().toLowerCase().endsWith(getDescription()))
               return true;

            else
               return false;
         }
      };
      
      fileChooser.setFileFilter(filterFilter);
      
      if(fileChooser.showSaveDialog(frame) == JFileChooser.APPROVE_OPTION)
      {
         String fileExt = filterFilter.getDescription();
         File chosenFile = fileChooser.getSelectedFile();
         String fileName = chosenFile.getName();
         
         if (!fileName.endsWith(fileExt))
         {
            fileName = fileName.concat(fileExt);

            if (!chosenFile.getName().equals(fileName))
            {
               chosenFile = new File(chosenFile.getParent(), fileName);
            }
         }
         
         writeDataCommandExecutor.writeData(allVariables, binary, binary?true:false, chosenFile);
      }
      

   }
   
   private void recursiveToggleVariableSendOrLog(boolean recursive, boolean toggleSend, boolean send, boolean toggleLog, boolean log, 
         TreeNode parentNode, YoVariableRegistry parentRegistry, ArrayList<YoVariableRegistry> changedRegistries)
   {
      if (toggleSend) parentRegistry.setSending(send);
      if (toggleLog) parentRegistry.setLogging(log);
      
      changedRegistries.add(parentRegistry);
      model.nodeChanged(parentNode);
      
      if (recursive)
      {         
         Enumeration<TreeNode> childrenNodes = parentNode.children();
         
         while(childrenNodes.hasMoreElements())
         {
            TreeNode childNode = childrenNodes.nextElement();
            YoVariableRegistry childRegistry = treeNodeRegistryMap.get(childNode);

            recursiveToggleVariableSendOrLog(recursive, toggleSend, send, toggleLog, log, childNode, childRegistry, changedRegistries);
         }
      }
   }
  

   @Override
   public void focusGained(FocusEvent e)
   {
   }

   @Override
   public void focusLost(FocusEvent e)
   {
//    popupMenu.setVisible(false);
   }

   public void loadConfiguration()
   {
      File configs = new File("Configurations");
      JFileChooser openDialog = new JFileChooser(configs);
      File selectedFile = null;
      if (JFileChooser.APPROVE_OPTION == openDialog.showOpenDialog(openDialog))
      {
         selectedFile = openDialog.getSelectedFile();
      }

      loadConfiguration(selectedFile);
   }

   public void loadConfiguration(File selectedFile)
   {
      if (selectedFile != null)
      {
         if (selectedFile.exists())
         {
            ArrayList<YoVariableRegistry> treeNodes = root.getAllRegistriesIncludingChildren();

            // getAllRegistrys(rootRegistry, treeNodes);

            // System.out.println(i+" "+realCound+" "+rootRegistry.createVarListsIncludingChildren().size());
            SimpleFileReader reader = new SimpleFileReader(selectedFile);
            String nextLine;
            while ((nextLine = reader.nextLine()) != null)
            {
               nextLine = nextLine.trim();
               StringTokenizer tok = new StringTokenizer(nextLine, "*");
               String name = tok.nextToken();
               YoVariableRegistry current = findRegistry(name, treeNodes);

               if (current != null)
               {
                  if (tok.nextToken().equals("true"))
                  {
                     current.setSending(true);
                  }
                  else
                     current.setSending(false);

                  if (tok.nextToken().equals("true"))
                  {
                     current.setLogging(true);
                  }
                  else
                     current.setLogging(false);

               }
               else
                  System.err.println("Error Loading configuration for YoVariable Registry " + name + "\nit no longer exists");

            }

            notifyRegistrySettingsChangedListeners();
         }
      }
   }

   private void notifyRegistrySettingsChangedListeners()
   {
      for (RegistrySettingsChangedListener listener : registrySettingsChangedListeners)
      {
         listener.registrySettingsChanged();
      }
   }

   private void notifyRegistrySettingsChangedListeners(ArrayList<YoVariableRegistry> yoVariableRegistries)
   {
      for (RegistrySettingsChangedListener listener : registrySettingsChangedListeners)
      {
         listener.registrySettingsChanged(yoVariableRegistries);
      }
   }


   private YoVariableRegistry findRegistry(String name, ArrayList<YoVariableRegistry> allRegistries)
   {
      for (YoVariableRegistry reg : allRegistries)
      {
         if (reg.getNameSpace().getName().equals(name))
         {
            return reg;
         }
      }

      return null;
   }

   public void saveConfiguration()
   {
      System.out.println("save");
      File configs = new File("Configurations");
      JFileChooser saveDialog = new JFileChooser(configs);
      File selectedFile = null;
      if (JFileChooser.APPROVE_OPTION == saveDialog.showSaveDialog(this))
      {
         selectedFile = saveDialog.getSelectedFile();
      }

      if (selectedFile != null)
      {
         SimpleFileWriter writer = new SimpleFileWriter(selectedFile);
         ArrayList<YoVariableRegistry> treeNodes = root.getAllRegistriesIncludingChildren();

         String outString = "";
         for (YoVariableRegistry node : treeNodes)
         {
            outString += node.getNameSpace().getName() + "*" + node.isSent() + "*" + node.isLogged() + "\n";
         }

         writer.write(outString);
         writer.close();
      }
   }

   @Override
   public void createdNewRegistries()
   {
      top.removeAllChildren();
      registryTreeNodeMap.clear();
      treeNodeRegistryMap.clear();
      
      this.topOfTreeRegistry = root;
      this.needToSetupTree = true;
//      setUpTree(root);
   }

   public void addRegistrySettingsChangedListener(RegistrySettingsChangedListener listener)
   {
      this.registrySettingsChangedListeners.add(listener);
   }
}
