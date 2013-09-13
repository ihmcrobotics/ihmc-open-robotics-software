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

   private JFrame jFrameOne, jFrameTwo;
   private final DesiredJointAccelerationMultipleJPanel desiredJointAccelerationMultipleJPanel;  
   
   private final DesiredSpatialAccelerationMultipleJPanel desiredSpatialAccelerationMultipleJPanel; 
   private final DesiredSpatialAccelerationMultipleJPanel desiredNullspaceForSpatialAccelerationMultipleJPanel; 
   
   private final JointAccelerationSolutionJPanel jointAccelerationSolutionJPanel;  
   private final MotionConstraintJMatrixJPanel motionConstraintJMatrixJPanel;
   
   public MomentumModuleGUI(YoVariableRegistry parentRegistry)
   {
      jFrameOne = new JFrame("MomentumModuleGUI Frame 1");
      jFrameTwo = new JFrame("MomentumModuleGUI Frame2");
      desiredJointAccelerationMultipleJPanel = new DesiredJointAccelerationMultipleJPanel();
      desiredSpatialAccelerationMultipleJPanel = new DesiredSpatialAccelerationMultipleJPanel();
      desiredNullspaceForSpatialAccelerationMultipleJPanel = new DesiredSpatialAccelerationMultipleJPanel();
      jointAccelerationSolutionJPanel = new JointAccelerationSolutionJPanel();
      motionConstraintJMatrixJPanel = new MotionConstraintJMatrixJPanel();
      
      jFrameOne.getContentPane().setLayout(new GridLayout(2, 2));
      jFrameOne.getContentPane().add(desiredJointAccelerationMultipleJPanel);
      jFrameOne.getContentPane().add(desiredSpatialAccelerationMultipleJPanel);
      jFrameOne.getContentPane().add(desiredNullspaceForSpatialAccelerationMultipleJPanel);
      jFrameOne.getContentPane().add(jointAccelerationSolutionJPanel);
      
      jFrameOne.setSize(1400, 1000);
      jFrameOne.setVisible(true);
      
      jFrameTwo.getContentPane().setLayout(new GridLayout(1, 1));
      jFrameTwo.getContentPane().add(motionConstraintJMatrixJPanel);
      
      jFrameTwo.setSize(1400, 1000);
      jFrameTwo.setVisible(true);
      
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
   
      ArrayList<DesiredSpatialAccelerationCommandAndMotionConstraint> desiredSpatialAccelerationCommandAndNullspaceMotionConstraints = allMomentumModuleListener.getDesiredSpatialAccelerationCommandAndNullspaceMotionConstraints();
      desiredNullspaceForSpatialAccelerationMultipleJPanel.setDesiredSpatialAccelerations(desiredSpatialAccelerationCommandAndNullspaceMotionConstraints);
         
      InverseDynamicsJoint[] jointsToOptimizeFor = allMomentumModuleListener.getJointsToOptimizeFor();
      DenseMatrix64F jointAccelerationsSolution = allMomentumModuleListener.getJointAccelerationsSolution();
      jointAccelerationSolutionJPanel.setJointAccelerationSolution(jointsToOptimizeFor, jointAccelerationsSolution);
      
      motionConstraintJMatrixJPanel.setMotionConstraintJMatrix(allMomentumModuleListener.getPrimaryMotionConstraintJMatrix());
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
