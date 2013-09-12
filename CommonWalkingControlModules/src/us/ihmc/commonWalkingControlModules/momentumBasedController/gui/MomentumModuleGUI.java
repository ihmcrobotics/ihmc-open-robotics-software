package us.ihmc.commonWalkingControlModules.momentumBasedController.gui;

import java.awt.GridLayout;
import java.util.ArrayList;

import javax.swing.JFrame;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.momentumBasedController.AllMomentumModuleListener;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommandAndMotionConstraint;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommandAndMotionConstraint;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class MomentumModuleGUI
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private JFrame jFrame;
   private final DesiredJointAccelerationMultipleJPanel desiredJointAccelerationMultipleJPanel;  
   private final DesiredSpatialAccelerationMultipleJPanel desiredSpatialAccelerationMultipleJPanel;  
   private final JointAccelerationSolutionJPanel jointAccelerationSolutionJPanel;  

   public MomentumModuleGUI(YoVariableRegistry parentRegistry)
   {
      jFrame = new JFrame("MomentumModuleGUI");
      desiredJointAccelerationMultipleJPanel = new DesiredJointAccelerationMultipleJPanel();
      desiredSpatialAccelerationMultipleJPanel = new DesiredSpatialAccelerationMultipleJPanel();
      jointAccelerationSolutionJPanel = new JointAccelerationSolutionJPanel();
      
      jFrame.getContentPane().setLayout(new GridLayout(3, 1));
      jFrame.getContentPane().add(desiredJointAccelerationMultipleJPanel);
      jFrame.getContentPane().add(desiredSpatialAccelerationMultipleJPanel);
      jFrame.getContentPane().add(jointAccelerationSolutionJPanel);
      
      jFrame.setSize(1400, 1000);
      jFrame.setVisible(true);
      
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
      ArrayList<DesiredJointAccelerationCommandAndMotionConstraint> desiredJointAccelerationCommandAndMotionConstraints = allMomentumModuleListener.getDesiredJointAccelerationCommandAndMotionConstraints();
      desiredJointAccelerationMultipleJPanel.setDesiredJointAccelerations(desiredJointAccelerationCommandAndMotionConstraints);

      ArrayList<DesiredSpatialAccelerationCommandAndMotionConstraint> desiredSpatialAccelerationCommandAndMotionConstraints = allMomentumModuleListener.getDesiredSpatialAccelerationCommandAndMotionConstraints();
      desiredSpatialAccelerationMultipleJPanel.setDesiredSpatialAccelerations(desiredSpatialAccelerationCommandAndMotionConstraints);
   
      
      InverseDynamicsJoint[] jointsToOptimizeFor = allMomentumModuleListener.getJointsToOptimizeFor();
      DenseMatrix64F jointAccelerationsSolution = allMomentumModuleListener.getJointAccelerationsSolution();
      jointAccelerationSolutionJPanel.setJointAccelerationSolution(jointsToOptimizeFor, jointAccelerationsSolution);
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
