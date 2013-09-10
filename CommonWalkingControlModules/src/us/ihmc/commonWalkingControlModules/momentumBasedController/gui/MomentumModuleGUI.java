package us.ihmc.commonWalkingControlModules.momentumBasedController.gui;

import java.awt.GridLayout;
import java.util.ArrayList;

import javax.swing.JFrame;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleDataObject;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleSolution;

public class MomentumModuleGUI
{
  private JFrame jFrame;
  private final DesiredJointAccelerationMultipleJPanel desiredJointAccelerationMultipleJPanel;  
  private final DesiredSpatialAccelerationMultipleJPanel desiredSpatialAccelerationMultipleJPanel;  
  
   public MomentumModuleGUI()
   {
      jFrame = new JFrame("MomentumModuleGUI");
      desiredJointAccelerationMultipleJPanel = new DesiredJointAccelerationMultipleJPanel();
      desiredSpatialAccelerationMultipleJPanel = new DesiredSpatialAccelerationMultipleJPanel();
      
      jFrame.getContentPane().setLayout(new GridLayout(2, 1));
      jFrame.getContentPane().add(desiredJointAccelerationMultipleJPanel);
      jFrame.getContentPane().add(desiredSpatialAccelerationMultipleJPanel);
      jFrame.setSize(1000, 1000);
      jFrame.setVisible(true);
   }

   public void setDesiredsAndSolution(MomentumModuleDataObject momentumModuleDataObject, MomentumModuleSolution momentumModuleSolution)
   {
      ArrayList<DesiredJointAccelerationCommand> desiredJointAccelerationCommands = momentumModuleDataObject.getDesiredJointAccelerationCommands();
      ArrayList<DesiredSpatialAccelerationCommand> desiredSpatialAccelerationCommands = momentumModuleDataObject.getDesiredSpatialAccelerationCommands();
      
      desiredJointAccelerationMultipleJPanel.setDesiredJointAccelerations(desiredJointAccelerationCommands);
      desiredSpatialAccelerationMultipleJPanel.setDesiredSpatialAccelerations(desiredSpatialAccelerationCommands);
   }

   public void reset()
   {
      // TODO Auto-generated method stub
      
   }

   public void initialize()
   {
      // TODO Auto-generated method stub
      
   }
}
