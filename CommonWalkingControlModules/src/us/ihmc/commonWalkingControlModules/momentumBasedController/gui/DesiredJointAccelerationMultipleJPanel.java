package us.ihmc.commonWalkingControlModules.momentumBasedController.gui;

import java.awt.Graphics;
import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommandAndMotionConstraint;

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

   public synchronized void setDesiredJointAccelerations(ArrayList<DesiredJointAccelerationCommandAndMotionConstraint> desiredJointAccelerationCommandAndMotionConstraints)
   {
      rearrangePanelsIfNecessary(desiredJointAccelerationCommandAndMotionConstraints.size());

      for (int i = 0; i < desiredJointAccelerationCommandAndMotionConstraints.size(); i++)
      {
         DesiredJointAccelerationCommandAndMotionConstraint desiredJointAccelerationCommandAndMotionConstraint = desiredJointAccelerationCommandAndMotionConstraints.get(i);
         
         DesiredJointAccelerationJPanel desiredJointAccelerationJPanel = getJPanel(i);
         desiredJointAccelerationJPanel.setDesiredJointAccelerationCommand(desiredJointAccelerationCommandAndMotionConstraint);
      }

      this.repaint();
   }

   public DesiredJointAccelerationJPanel constructNewJPanel()
   {
      return new DesiredJointAccelerationJPanel();
   }

   public synchronized void paintComponent(Graphics g)
   {
      super.paintComponent(g);
   }

}
