package us.ihmc.moonwalking.models.SeriesElasticActuator;

import com.mathworks.jama.Matrix;
import com.yobotics.simulationconstructionset.gui.BodePlotConstructor;

import us.ihmc.utilities.linearDynamicSystems.LinearDynamicSystem;
import us.ihmc.utilities.linearDynamicSystems.TransferFunction;
import us.ihmc.utilities.linearDynamicSystems.TransferFunctionMatrix;

public class SeriesElasticActuatorLinearDynamicSystem 
{
   private LinearDynamicSystem openLoopDynamicSystem, closedForceLoopDynamicSystem, closedPositionLoopDynamicSystem;
   private TransferFunction openLoopTransferFunction, closedForceLoopTransferFunction, closedPositionLoopTransferFunction;

   public SeriesElasticActuatorLinearDynamicSystem()
   {
      createLinearDynamicSystem();
   }

   public void createLinearDynamicSystem()
   {
      Matrix matrixA = new Matrix(6, 6);

      matrixA.set(0, 0, 0.0);
      matrixA.set(0, 1, 1.0);
      matrixA.set(0, 2, 0.0);
      matrixA.set(0, 3, 0.0);
      matrixA.set(0, 4, 0.0);
      matrixA.set(0, 5, 0.0);
      
      matrixA.set(1, 0, -SEAParameters.kpGearDefault/(SEAParameters.MOTOR_ROTOR_I * SEAParameters.GEAR_RATIO * SEAParameters.GEAR_RATIO));
      matrixA.set(1, 1, -SEAParameters.kdGearDefault/(SEAParameters.MOTOR_ROTOR_I * SEAParameters.GEAR_RATIO * SEAParameters.GEAR_RATIO));
      matrixA.set(1, 2, SEAParameters.kpGearDefault/(SEAParameters.MOTOR_ROTOR_I * SEAParameters.GEAR_RATIO));
      matrixA.set(1, 3, SEAParameters.kdGearDefault/(SEAParameters.MOTOR_ROTOR_I * SEAParameters.GEAR_RATIO));
      matrixA.set(1, 4, 0.0);
      matrixA.set(1, 5, 0.0);

      matrixA.set(2, 0, 0.0);
      matrixA.set(2, 1, 0.0);
      matrixA.set(2, 2, 0.0);
      matrixA.set(2, 3, 1.0);
      matrixA.set(2, 4, 0.0);
      matrixA.set(2, 5, 0.0);

      matrixA.set(3, 0, SEAParameters.kpGearDefault/(SEAParameters.GEARBOX_OUTPUT_I * SEAParameters.GEAR_RATIO));
      matrixA.set(3, 1, SEAParameters.kdGearDefault/(SEAParameters.GEARBOX_OUTPUT_I * SEAParameters.GEAR_RATIO));
      matrixA.set(3, 2, -SEAParameters.kpGearDefault/(SEAParameters.GEARBOX_OUTPUT_I));
      matrixA.set(3, 3, -(SEAParameters.kdGearDefault + SEAParameters.gearboxLinearDampingDefault)/(SEAParameters.GEARBOX_OUTPUT_I));
      matrixA.set(3, 4, SEAParameters.springConstantDefault/(SEAParameters.GEARBOX_OUTPUT_I));
      matrixA.set(3, 5, 0.0);

      matrixA.set(4, 0, 0.0);
      matrixA.set(4, 1, 0.0);
      matrixA.set(4, 2, 0.0);
      matrixA.set(4, 3, 0.0);
      matrixA.set(4, 4, 0.0);
      matrixA.set(4, 5, 1.0);
      
      matrixA.set(5, 0, -SEAParameters.kpGearDefault/(SEAParameters.GEARBOX_OUTPUT_I * SEAParameters.GEAR_RATIO));
      matrixA.set(5, 1, -SEAParameters.kdGearDefault/(SEAParameters.GEARBOX_OUTPUT_I * SEAParameters.GEAR_RATIO));
      matrixA.set(5, 2, SEAParameters.kpGearDefault/(SEAParameters.GEARBOX_OUTPUT_I));
      matrixA.set(5, 3,(SEAParameters.kdGearDefault + SEAParameters.gearboxLinearDampingDefault)/(SEAParameters.GEARBOX_OUTPUT_I));
      matrixA.set(5, 4, -SEAParameters.springConstantDefault * (1.0/SEAParameters.GEARBOX_OUTPUT_I + 1.0/SEAParameters.LOAD_INERTIA));
      matrixA.set(5, 5, 0.0);

      
      Matrix matrixB = new Matrix(6, 1);

      matrixB.set(0, 0, 0.0);
      matrixB.set(1, 0, 1.0/(SEAParameters.MOTOR_ROTOR_I * SEAParameters.GEAR_RATIO));
      matrixB.set(2, 0, 0.0);
      matrixB.set(3, 0, 0.0);
      matrixB.set(4, 0, 0.0);
      matrixB.set(5, 0, 0.0);

      Matrix matrixCLoadTorque = new Matrix(4, 6);

      matrixCLoadTorque.set(0, 0, 0.0);
      matrixCLoadTorque.set(0, 1, 0.0);
      matrixCLoadTorque.set(0, 2, 0.0);
      matrixCLoadTorque.set(0, 3, 0.0);
      matrixCLoadTorque.set(0, 4, -SEAParameters.springConstantDefault);
      matrixCLoadTorque.set(0, 5, 0.0);
      
      matrixCLoadTorque.set(1, 0, 0.0);
      matrixCLoadTorque.set(1, 1, 0.0);
      matrixCLoadTorque.set(1, 2, 0.0);
      matrixCLoadTorque.set(1, 3, 0.0);
      matrixCLoadTorque.set(1, 4, 0.0);
      matrixCLoadTorque.set(1, 5, -SEAParameters.springConstantDefault);
      
      matrixCLoadTorque.set(2, 0, 0.0);
      matrixCLoadTorque.set(2, 1, 0.0);
      matrixCLoadTorque.set(2, 2, 1.0);
      matrixCLoadTorque.set(2, 3, 0.0);
      matrixCLoadTorque.set(2, 4, 1.0);
      matrixCLoadTorque.set(2, 5, 0.0);
      
      matrixCLoadTorque.set(3, 0, 0.0);
      matrixCLoadTorque.set(3, 1, 0.0);
      matrixCLoadTorque.set(3, 2, 0.0);
      matrixCLoadTorque.set(3, 3, 1.0);
      matrixCLoadTorque.set(3, 4, 0.0);
      matrixCLoadTorque.set(3, 5, 1.0);

      openLoopDynamicSystem = new LinearDynamicSystem(matrixA, matrixB, matrixCLoadTorque, null);

      // Open loop transfer function from drive torque as seen at the output of the gears to actuator output torque.
      TransferFunctionMatrix openLoopMatrix = openLoopDynamicSystem.getTransferFunctionMatrix();
      if ((openLoopMatrix.getRows() != 4) || (openLoopMatrix.getColumns() != 1))
      {
	 throw new RuntimeException("Should be 4x1!");
      }

      openLoopTransferFunction = openLoopMatrix.get(0, 0);
      System.out.println("openLoopForceTransferFunction = \n" + openLoopTransferFunction);


      // Figure out tau gains:

      double naturalFreq = 40.0 * 2.0 * Math.PI;
      double dampingRatio = 0.7;

      double Iequiv = SEAParameters.GEARBOX_OUTPUT_I + SEAParameters.MOTOR_ROTOR_I * SEAParameters.GEAR_RATIO * SEAParameters.GEAR_RATIO;

      double kpSEA = (naturalFreq * naturalFreq * Iequiv - SEAParameters.springConstantDefault) / (SEAParameters.springConstantDefault);
      double kdSEA = 2.0 * dampingRatio * naturalFreq * Iequiv / (SEAParameters.springConstantDefault);

      double openLoopWn = Math.sqrt(SEAParameters.springConstantDefault/Iequiv);
      System.out.println("openLoopWn = " + openLoopWn + " rad/sec = " + openLoopWn/(2.0*Math.PI) + " Hertz"); 
      
      System.out.println("kpSEA = " + kpSEA);
      System.out.println("kdSEA = " + kdSEA);
      	

      // Add force control feedback:
      Matrix matrixK = new Matrix(1, 4);
      matrixK.set(0, 0, kpSEA);
      matrixK.set(0, 1, kdSEA);
      matrixK.set(0, 2, 0.0);
      matrixK.set(0, 3, 0.0);

      Matrix matrixFF = new Matrix(1, 4);
      matrixFF.set(0, 0, 1.0); 
      matrixFF.set(0, 1, -kdSEA); // Don't add derivative to the desired torque. Causes spikes on steps.
      matrixFF.set(0, 2, 0.0);
      matrixFF.set(0, 3, 0.0);


      closedForceLoopDynamicSystem = openLoopDynamicSystem.addOutputStateFeedback(matrixK, matrixFF);
      TransferFunctionMatrix closedForceLoopMatrix = closedForceLoopDynamicSystem.getTransferFunctionMatrix();
      if ((closedForceLoopMatrix.getRows() != 4) || (closedForceLoopMatrix.getColumns() != 4))
      {
	 throw new RuntimeException("Should be 4x4!");
      }
      
      closedForceLoopTransferFunction = closedForceLoopMatrix.get(0, 0);
      System.out.println("closedForceLoopTransferFunction = \n" + closedForceLoopTransferFunction);

      
      // Add Position Control Loop on top of the force control loop:
      double naturalFreqPosition = 10.0 * 2.0 * Math.PI;
      double dampingRatioPosition = 1.0;

      double kpPosition = naturalFreqPosition * naturalFreqPosition * SEAParameters.LOAD_INERTIA;
      double kdPosition = 2.0 * dampingRatioPosition * naturalFreqPosition * SEAParameters.LOAD_INERTIA;

      System.out.println("kpPosition = " + kpPosition);
      System.out.println("kdPosition = " + kdPosition);
      
      // Add position control loop:
      Matrix matrixKPosition = new Matrix(4, 4);
      matrixKPosition.set(0, 2, kpPosition);
      matrixKPosition.set(0, 3, kdPosition);

//      Matrix matrixFFPosition = new Matrix(4, 4);
//      matrixFFPosition.set(0, 3, -kdPosition);  // Don't add derivative to the desired torque. Causes spikes on steps.
      
      closedPositionLoopDynamicSystem = closedForceLoopDynamicSystem.addOutputStateFeedback(matrixKPosition, null); //matrixFFPosition);

      TransferFunctionMatrix closedPositionLoopMatrix = closedPositionLoopDynamicSystem.getTransferFunctionMatrix();
      if ((closedPositionLoopMatrix.getRows() != 4) || (closedPositionLoopMatrix.getColumns() != 4))
      {
	 throw new RuntimeException("Should be 4x4!");
      }
      
      closedPositionLoopTransferFunction = closedPositionLoopMatrix.get(2, 2);
      System.out.println("closedPositionLoopTransferFunction = \n" + closedPositionLoopTransferFunction);
    
   }

   private void plotBodeDiagrams() 
   {
      // Display the theoretical bode diagram:
      double[] omega = generateLinearSpace(2000, 1.2, 0.2);
      String openLoopLabel = "Open loop transfer function";
      BodePlotConstructor.plotBodeForTransferFunction(openLoopLabel, openLoopTransferFunction, omega);
      
      String closedForceLoopLable = "Closed force loop transfer function";
      BodePlotConstructor.plotBodeForTransferFunction(closedForceLoopLable, closedForceLoopTransferFunction, omega);
      
      String closedPositionLoopLable = "Closed position loop transfer function";
      BodePlotConstructor.plotBodeForTransferFunction(closedPositionLoopLable, closedPositionLoopTransferFunction, omega);

   }

   public static double[] generateLinearSpace(int numPoints, double x0, double xIncrement)
   {
      double[] ret = new double[numPoints];
      for (int i = 0; i < numPoints; i++)
      {
	 ret[i] = x0 + xIncrement * ((double) i);
      }

      return ret;
   }


   public static void main(String[] args)
   {
      SeriesElasticActuatorLinearDynamicSystem system = new SeriesElasticActuatorLinearDynamicSystem();

      system.plotBodeDiagrams();
   }


}
