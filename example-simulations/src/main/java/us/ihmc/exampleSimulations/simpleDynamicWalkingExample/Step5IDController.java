package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;


import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.exampleSimulations.simpleDynamicWalkingExample.RobotParameters.JointNames;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.robotics.screwTheory.DifferentialIDMassMatrixCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class Step5IDController implements RobotController
{

   private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final Step5IDandSCSRobot robot;
   
   private final YoDouble qDesiredHip = new YoDouble("qDesiredHip", registry);
   private final YoDouble qdDesiredHip = new YoDouble("qdDesiredHip", registry);
   private final YoDouble qddDesiredHip = new YoDouble("qddDesiredHip", registry);
   
   private final YoDouble gain = new YoDouble("gain", registry), damp = new YoDouble("damp", registry);
   
   private final OneDoFJoint hipJoint;
   
   private final YoDouble hipAmplitude = new YoDouble("hipAmplitude", registry);
   private final YoDouble hipPeriod = new YoDouble("hipPeriod", registry);
   
   
    // Joint-space bias force: gathers centrifugal, Coriolis, and gravity generalized forces
   private final DenseMatrix64F biasForce = new DenseMatrix64F(2, 1);
   
    // Mass matrix of the robot 
   private final DenseMatrix64F massMatrix = new DenseMatrix64F(2, 2);
   
   //Temporary variables 
   private final DenseMatrix64F desiredJointAccelerations = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F jointTorques = new DenseMatrix64F(2, 1);
   
   // The inverse dynamics calculator is required either to do inverse dynamics based control or to compute the joint-space bias force    
   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   
   //  Mass matrix calculator: Based on the composite rigid body algorithm proposed by Roy Featherstone (see his book Rigid Body Dynamics Algorithms)
   private final CompositeRigidBodyMassMatrixCalculator crbMassMatrixCalculator;
  
    //  Mass matrix calculator: Using the inverse dynamics calculator (highly inefficient)
   private final DifferentialIDMassMatrixCalculator differentialIDMassMatrixCalculator;
   
   
   
   public Step5IDController(Step5IDandSCSRobot robot, String name, double deltaT)
   {
      this.robot = robot;
      hipJoint = robot.getLegJoint(JointNames.HIP, RobotSide.LEFT);
      
      inverseDynamicsCalculator = new InverseDynamicsCalculator(robot.getElevator(), -robot.getGravityZ());
      crbMassMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(robot.getElevator());
      differentialIDMassMatrixCalculator = new DifferentialIDMassMatrixCalculator(worldFrame, robot.getElevator());
      
      initialize();
   }
   
   public void initialize()
   {
      gain.set(20.0);
      damp.set(5.0);
      
      hipAmplitude.set(1.5);
      hipPeriod.set(5.0);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return registry.getName();
   }

   public String getDescription()
   {
      return getName();
   }

   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   public void doControl()
   {
      robot.updatePositionsIDrobot();

      double hipPulse = 2.0 * Math.PI / hipPeriod.getDoubleValue();
      qDesiredHip.set(hipAmplitude.getDoubleValue() * Math.sin(hipPulse * robot.getTime()));
      qdDesiredHip.set(hipPulse * hipAmplitude.getDoubleValue() * Math.cos(hipPulse * robot.getTime()));
      qddDesiredHip.set(
            -MathTools.square(hipPulse) * hipAmplitude.getDoubleValue() * Math.sin(hipPulse * robot.getTime())
            + gain.getDoubleValue() * (qDesiredHip.getDoubleValue() - hipJoint.getQ())
            + damp.getDoubleValue() * (qdDesiredHip.getDoubleValue() - hipJoint.getQd()));
      
      doControlUsingInverseDynamicsCalculatorOnly();
      
      robot.updateTorquesSCSrobot();
   }

   private void doControlUsingInverseDynamicsCalculatorOnly()
   {
      hipJoint.setQddDesired(qddDesiredHip.getDoubleValue());
      inverseDynamicsCalculator.compute();
   }
}
