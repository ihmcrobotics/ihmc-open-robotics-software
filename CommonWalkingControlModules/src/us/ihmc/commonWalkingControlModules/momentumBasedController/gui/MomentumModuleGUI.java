package us.ihmc.commonWalkingControlModules.momentumBasedController.gui;

import java.awt.BorderLayout;
import java.awt.GraphicsDevice;
import java.awt.GraphicsEnvironment;
import java.util.ArrayList;

import javax.swing.JFrame;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.momentumBasedController.AllMomentumModuleListener;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommandAndMotionConstraint;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommandAndMotionConstraint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class MomentumModuleGUI
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private JFrame jFrame;
   private final DesiredJointAccelerationMultipleJPanel desiredJointAccelerationMultipleJPanel;
   private final DesiredSpatialAccelerationMultipleJPanel desiredSpatialAccelerationMultipleJPanel;
   private final JointAccelerationSolutionJPanel jointAccelerationSolutionJPanel;
   private boolean packed = false;
   public MomentumModuleGUI(YoVariableRegistry parentRegistry)
   {
      jFrame = new JFrame("MomentumModuleGUI");
      desiredJointAccelerationMultipleJPanel = new DesiredJointAccelerationMultipleJPanel();
      desiredSpatialAccelerationMultipleJPanel = new DesiredSpatialAccelerationMultipleJPanel();
      jointAccelerationSolutionJPanel = new JointAccelerationSolutionJPanel();

      jFrame.getContentPane().add(desiredJointAccelerationMultipleJPanel, BorderLayout.NORTH);
      jFrame.getContentPane().add(desiredSpatialAccelerationMultipleJPanel, BorderLayout.CENTER);
      jFrame.getContentPane().add(jointAccelerationSolutionJPanel, BorderLayout.SOUTH);

      jFrame.setSize(1400, 942);
      jFrame.setVisible(true);
      jFrame.setLocationRelativeTo(null);
      

      jFrame.setResizable(false);
      //jFrame.setAlwaysOnTop(true);

      int showOnScreenNumber = 1;
      showOnScreen(showOnScreenNumber, jFrame);

      parentRegistry.addChild(registry);
      
   }

   //   public void setDesiredsAndSolution(MomentumModuleDataObject momentumModuleDataObject, MomentumModuleSolution momentumModuleSolution)
   //   {
   //      ArrayList<DesiredJointAccelerationCommand> desiredJointAccelerationCommands = momentumModuleDataObject.getDesiredJointAccelerationCommands();
   //      ArrayList<DesiredSpatialAccelerationCommand> desiredSpatialAccelerationCommands = momentumModuleDataObject.getDesiredSpatialAccelerationCommands();
   //      
   //      desiredJointAccelerationMultipleJPanel.setDesiredJointAccelerations(desiredJointAccelerationCommands);
   //      desiredSpatialAccelerationMultipleJPanel.setDesiredSpatialAccelerations(desiredSpatialAccelerationCommands);
   //   }

   public void update(AllMomentumModuleListener allMomentumModuleListener)
   {
      ArrayList<DesiredJointAccelerationCommandAndMotionConstraint> desiredJointAccelerationCommandAndMotionConstraints = allMomentumModuleListener
            .getDesiredJointAccelerationCommandAndMotionConstraints();
      desiredJointAccelerationMultipleJPanel.setDesiredJointAccelerations(desiredJointAccelerationCommandAndMotionConstraints);

      ArrayList<DesiredSpatialAccelerationCommandAndMotionConstraint> desiredSpatialAccelerationCommandAndMotionConstraints = allMomentumModuleListener
            .getDesiredSpatialAccelerationCommandAndMotionConstraints();
      desiredSpatialAccelerationMultipleJPanel.setDesiredSpatialAccelerations(desiredSpatialAccelerationCommandAndMotionConstraints);

      InverseDynamicsJoint[] jointsToOptimizeFor = allMomentumModuleListener.getJointsToOptimizeFor();
      DenseMatrix64F jointAccelerationsSolution = allMomentumModuleListener.getJointAccelerationsSolution();
      jointAccelerationSolutionJPanel.setJointAccelerationSolution(jointsToOptimizeFor, jointAccelerationsSolution);
    
      if(!packed)
      {
    	  jFrame.pack();
    	  packed=true;
      }
   }

   public void reset()
   {
	   packed=false;
   }

   public void initialize()
   {
	   packed=false;
   }

   public static void showOnScreen(int screen, JFrame frame)
   {
      GraphicsEnvironment ge = GraphicsEnvironment.getLocalGraphicsEnvironment();
      GraphicsDevice[] gd = ge.getScreenDevices();
      if (screen > -1 && screen < gd.length)
      {
         frame.setLocation(gd[screen].getDefaultConfiguration().getBounds().x + 10, frame.getY());
      }
      else if (gd.length > 0)
      {
         frame.setLocation(gd[0].getDefaultConfiguration().getBounds().x + 10, frame.getY());
      }
      else
      {
         throw new RuntimeException("No Screens Found");
      }
   }
}
