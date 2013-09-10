package us.ihmc.commonWalkingControlModules.momentumBasedController.gui;

import java.awt.GridLayout;
import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;

public class DesiredJointAccelerationMultipleJPanel extends AbstractMultipleReusableJPanel<DesiredJointAccelerationJPanel>
{
   private static final long serialVersionUID = -4892971095902174475L;
   
   public DesiredJointAccelerationMultipleJPanel()
   {
//    this.setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
//      this.setLayout(new GridLayout(39, 1));
//      this.setLayout(new GridLayout(39, 1));
    
      this.repaint();
   }

   public void setDesiredJointAccelerations(ArrayList<DesiredJointAccelerationCommand> desiredJointAccelerationCommands)
   {
      rearrangePanelsIfNecessary(desiredJointAccelerationCommands.size());

      for (int i = 0; i < desiredJointAccelerationCommands.size(); i++)
      {
         DesiredJointAccelerationCommand desiredJointAccelerationCommand = desiredJointAccelerationCommands.get(i);
         DesiredJointAccelerationJPanel desiredJointAccelerationJPanel = getJPanel(i);

         desiredJointAccelerationJPanel.setDesiredJointAccelerationCommand(desiredJointAccelerationCommand);
      }

      this.repaint();
   }

   @Override
   public DesiredJointAccelerationJPanel constructNewJPanel()
   {
      return new DesiredJointAccelerationJPanel();
   }
}
