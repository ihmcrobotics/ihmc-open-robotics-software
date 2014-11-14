package us.ihmc.valkyrie.simulation;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.JButton;

import us.ihmc.yoUtilities.dataStructure.registry.NameSpace;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class ValkyrieDataFileNamespaceRenamer
{
   public ValkyrieDataFileNamespaceRenamer()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("null"));
      
      YoVariableRegistry rootRegistry = scs.getRootRegistry();
      
      ChangeNamespacesToMatchSimButton button = new ChangeNamespacesToMatchSimButton(rootRegistry);
      scs.addButton(button);
      
      scs.startOnAThread();
   }
   
   private class ChangeNamespacesToMatchSimButton extends JButton implements ActionListener
   {
      private static final long serialVersionUID = -6919907212022890245L;
      private final YoVariableRegistry rootRegistry;
      
      public ChangeNamespacesToMatchSimButton(YoVariableRegistry rootRegistry)
      {
         super("ChangeNamespaces");
         
         this.rootRegistry = rootRegistry;
         
         this.addActionListener(this);
      }

      @Override
      public void actionPerformed(ActionEvent e)
      {
         recursivelyPrintNamespaces(rootRegistry);
      }
   }
   
   
   private void recursivelyPrintNamespaces(YoVariableRegistry registry)
   {
      NameSpace nameSpace = registry.getNameSpace();
      
      String nameSpaceString = nameSpace.getName().replaceAll("loggedmain", "V1.DRCSimulation");
      
      if (nameSpaceString.startsWith("V1.")) nameSpaceString = "root." + nameSpaceString;
      registry.changeNameSpace(nameSpaceString);
      
      System.out.println(nameSpaceString);
      
      ArrayList<YoVariableRegistry> children = registry.getChildren();
      
      for (YoVariableRegistry yoVariableRegistry : children)
      {
         recursivelyPrintNamespaces(yoVariableRegistry);
      }
   }
   
   public static void main(String[] args)
   {
      new ValkyrieDataFileNamespaceRenamer();
   }
}
