package us.ihmc.valkyrie.simulation;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;

import us.ihmc.robotics.dataStructures.registry.NameSpaceRenamer;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class ValkyrieDataFileNamespaceRenamer
{
   public ValkyrieDataFileNamespaceRenamer()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("null"));
      YoVariableRegistry rootRegistry = scs.getRootRegistry();
      
      NameSpaceRenamer valkyrieNameSpaceRenamer = new ValkyrieNameSpaceRenamer();
      ChangeNamespacesToMatchSimButton changeNamespacesToMatchSimButton = new ChangeNamespacesToMatchSimButton("ChangeValkyrieNamespaces", rootRegistry, valkyrieNameSpaceRenamer);
      scs.addButton(changeNamespacesToMatchSimButton);
      
      NameSpaceRenamer stepprNameSpaceRenamer = new StepprNameSpaceRenamer();
      ChangeNamespacesToMatchSimButton changeStepprNamespacesToMatchSimButton = new ChangeNamespacesToMatchSimButton("ChangeStepprNamespaces", rootRegistry, stepprNameSpaceRenamer);
      scs.addButton(changeStepprNamespacesToMatchSimButton);
      
      scs.startOnAThread();
   }
   
   private class ChangeNamespacesToMatchSimButton extends JButton implements ActionListener
   {
      private static final long serialVersionUID = -6919907212022890245L;
      private final YoVariableRegistry rootRegistry;
      private final NameSpaceRenamer nameSpaceRenamer;
      
      public ChangeNamespacesToMatchSimButton(String name, YoVariableRegistry rootRegistry, NameSpaceRenamer nameSpaceRenamer)
      {
         super(name);
         
         this.rootRegistry = rootRegistry;
         this.nameSpaceRenamer = nameSpaceRenamer;
         
         this.addActionListener(this);
      }

      @Override
      public void actionPerformed(ActionEvent e)
      {
         rootRegistry.recursivelyChangeNamespaces(nameSpaceRenamer);
      }
   }
   
   private class StepprNameSpaceRenamer implements NameSpaceRenamer
   {
      @Override
      public String changeNamespaceString(String nameSpaceString)
      {
         String newNameSpaceString = nameSpaceString.replaceAll("loggedmain", "bono.DRCSimulation");
         if (newNameSpaceString.startsWith("bono.")) newNameSpaceString = "root." + newNameSpaceString;

         return newNameSpaceString;
      }
   }
   
   private class ValkyrieNameSpaceRenamer implements NameSpaceRenamer
   {
      @Override
      public String changeNamespaceString(String nameSpaceString)
      {
         String newNameSpaceString = nameSpaceString.replaceAll("loggedmain", "V1.DRCSimulation");
         if (newNameSpaceString.startsWith("V1.")) newNameSpaceString = "root." + newNameSpaceString;

         return newNameSpaceString;
      }
   }
  
   
   public static void main(String[] args)
   {
      new ValkyrieDataFileNamespaceRenamer();
   }
}
