package us.ihmc.commonWalkingControlModules.momentumBasedController.gui;


import java.awt.Graphics;
import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommandAndMotionConstraint;

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

   public synchronized void setDesiredSpatialAccelerations(ArrayList<DesiredSpatialAccelerationCommandAndMotionConstraint> desiredSpatialAccelerationCommandAndMotionConstraints)
   {
      rearrangePanelsIfNecessary(desiredSpatialAccelerationCommandAndMotionConstraints.size());

      for (int i = 0; i < desiredSpatialAccelerationCommandAndMotionConstraints.size(); i++)
      {
         DesiredSpatialAccelerationCommandAndMotionConstraint desiredSpatialAccelerationCommandAndMotionConstraint = desiredSpatialAccelerationCommandAndMotionConstraints.get(i);
         
         DesiredSpatialAccelerationJPanel desiredJointAccelerationJPanel = getJPanel(i);

         desiredJointAccelerationJPanel.setDesiredSpatialAccelerationCommand(desiredSpatialAccelerationCommandAndMotionConstraint);
      }

      this.repaint();
   }

   public DesiredSpatialAccelerationJPanel constructNewJPanel()
   {
      return new DesiredSpatialAccelerationJPanel();
   }

   public synchronized void paintComponent(Graphics g)
   {
      super.paintComponent(g);
   }

}

