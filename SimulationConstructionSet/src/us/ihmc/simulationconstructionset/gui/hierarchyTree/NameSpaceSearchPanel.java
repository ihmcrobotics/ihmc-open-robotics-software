package us.ihmc.simulationconstructionset.gui.hierarchyTree;

import java.awt.BorderLayout;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;

public class NameSpaceSearchPanel extends JPanel
{
   private static final long serialVersionUID = 2531114756584430672L;
   private NameSpaceHierarchyTree nameSpaceHierarchyTree;
   
   public NameSpaceSearchPanel(NameSpaceHierarchyTree nameSpaceHierarchyTree)
   {
      super(new BorderLayout());
      
      this.setName("NameSpaceSearchPanel");
      this.add(new NameSpaceSearchField(), BorderLayout.NORTH);
      this.nameSpaceHierarchyTree = nameSpaceHierarchyTree;
      this.add(nameSpaceHierarchyTree);
   }
   
   public class NameSpaceSearchField extends JPanel implements ActionListener
   {
      private static final long serialVersionUID = 8594680043494017825L;
      private final JTextField searchTextField;
      
      public NameSpaceSearchField()
      {
         this.setLayout(new GridLayout(1, 1));
         searchTextField = new JTextField();
         searchTextField.addActionListener(this);
         
         DocumentListener documentListener = new DocumentListener()
         {
            @Override
            public void insertUpdate(DocumentEvent e)
            {
               filterNameSpaceTree();
            }

            @Override
            public void removeUpdate(DocumentEvent e)
            {
               filterNameSpaceTree();
            }

            @Override
            public void changedUpdate(DocumentEvent e)
            {
            }
         };

         searchTextField.getDocument().addDocumentListener(documentListener);
         
         this.add(searchTextField);
      }
      
      @Override
      public void actionPerformed(ActionEvent arg0)
      {
         filterNameSpaceTree();
      }
      
      private void filterNameSpaceTree()
      {
         nameSpaceHierarchyTree.filter(searchTextField.getText().toString());
      }
   }
}
