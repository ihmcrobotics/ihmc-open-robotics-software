package us.ihmc.commonWalkingControlModules.momentumBasedController.gui;


import java.awt.GridLayout;
import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;

public class DesiredSpatialAccelerationMultipleJPanel extends AbstractMultipleReusableJPanel<DesiredSpatialAccelerationJPanel>
{
   private static final long serialVersionUID = -4892971095902174475L;
   
   public DesiredSpatialAccelerationMultipleJPanel()
   {
//    this.setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
//      this.setLayout(new GridLayout(39, 1));
//      this.setLayout(new GridLayout(39, 1));
    
      this.repaint();
   }

   public void setDesiredSpatialAccelerations(ArrayList<DesiredSpatialAccelerationCommand> desiredSpatialAccelerationCommands)
   {
      rearrangePanelsIfNecessary(desiredSpatialAccelerationCommands.size());

      for (int i = 0; i < desiredSpatialAccelerationCommands.size(); i++)
      {
         DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand = desiredSpatialAccelerationCommands.get(i);
         DesiredSpatialAccelerationJPanel desiredJointAccelerationJPanel = getJPanel(i);

         desiredJointAccelerationJPanel.setDesiredJointAccelerationCommand(desiredSpatialAccelerationCommand);
      }

      this.repaint();
   }

   @Override
   public DesiredSpatialAccelerationJPanel constructNewJPanel()
   {
      return new DesiredSpatialAccelerationJPanel();
   }
}

