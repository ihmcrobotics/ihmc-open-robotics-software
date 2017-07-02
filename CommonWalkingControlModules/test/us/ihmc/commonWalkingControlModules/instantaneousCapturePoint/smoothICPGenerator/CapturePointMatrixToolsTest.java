package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoFrameTrajectory3D;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CapturePointMatrixToolsTest
{
   private static final int nTests = 20;
   private static final double omega0 = 3.4;
   private static final double EPSILON = 10e-6;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private Random random = new Random();
   
   String namePrefix = "ReferenceICPTrajectoryFromCMPPolynomialGeneratorTest";
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMatricesPrimeLinear()
   {
      // Linear polynomial: y(x) = a0 + a1*x
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
      int numberOfCoefficients = 2;
      YoFrameTrajectory3D linear3D = new YoFrameTrajectory3D(namePrefix + "Linear", numberOfCoefficients, worldFrame, registry);
      
      for(int i = 0; i < nTests; i++)
      {
         double scaleTFinal = 1.0 / Math.random();
         double t0 = 0.0, tFinal = t0 + scaleTFinal * Math.random();
                    
         FramePoint z0 = new FramePoint(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         FramePoint zFinal = new FramePoint(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         
         linear3D.setLinear(t0, tFinal, z0, zFinal);
         
         double time = t0 + Math.random() * (tFinal - t0);
         
         DenseMatrix64F alphaPrimeAutomatic = new DenseMatrix64F(3, numberOfCoefficients);
         DenseMatrix64F alphaPrimeManual = new DenseMatrix64F(3, numberOfCoefficients);
      
         CapturePointMatrixTools.calculateGeneralizedAlphaPrimeOnCMPSegment(omega0, time, alphaPrimeAutomatic, 0, linear3D);
         calculateAlphaPrimeLinear(omega0 , time, tFinal, alphaPrimeManual);

         for(int j = 0; j < numberOfCoefficients; j++)
         {
            for(int k = 0; k < 3; k++)
            {
               assertEquals(alphaPrimeAutomatic.get(k, j), alphaPrimeManual.get(k, j), EPSILON);
            }
         }
         
         DenseMatrix64F betaPrimeAutomatic = new DenseMatrix64F(3, numberOfCoefficients);
         DenseMatrix64F betaPrimeManual = new DenseMatrix64F(3, numberOfCoefficients);
      
         CapturePointMatrixTools.calculateGeneralizedBetaPrimeOnCMPSegment(omega0, time, betaPrimeAutomatic, 0, linear3D);
         calculateBetaPrimeLinear(omega0 , time, tFinal, betaPrimeManual);
         
         for(int j = 0; j < numberOfCoefficients; j++)
         {
            for(int k = 0; k < 3; k++)
            {
               assertEquals(betaPrimeAutomatic.get(k, j), betaPrimeManual.get(k, j), EPSILON);
            }
         }
         
         DenseMatrix64F gammaPrimeAutomatic = new DenseMatrix64F(1, 1);
         DenseMatrix64F gammaPrimeManual = new DenseMatrix64F(1, 1);
      
         CapturePointMatrixTools.calculateGeneralizedGammaPrimeOnCMPSegment(omega0, time, gammaPrimeAutomatic, 0, linear3D);
         calculateGammaPrimeLinear(omega0 , time, tFinal, gammaPrimeManual);
         
         assertEquals(gammaPrimeAutomatic.get(0), gammaPrimeManual.get(0), EPSILON);
         
         //EuclidCoreTestTools.assertTuple3DEquals("", arrayToPack.get(i).getPoint3dCopy(), pointBetweenFeet, 1e-10);
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMatricesPrimeCubic()
   {
      // Cubic polynomial: y(x) = a0 + a1*x + a2*x^2 + a3*x^3
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
      int numberOfCoefficients = 4;
      YoFrameTrajectory3D cubic3D = new YoFrameTrajectory3D(namePrefix + "Cubic", numberOfCoefficients, worldFrame, registry);
      
      for(int i = 0; i < nTests; i++)
      {
         double scaleTFinal = 1.0 / Math.random();
         double t0 = 0.0, tFinal = t0 + scaleTFinal * Math.random();
                    
         FramePoint z0 = new FramePoint(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         FramePoint zFinal = new FramePoint(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         
         FrameVector zd0 = new FrameVector(worldFrame, new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         FrameVector zdFinal = new FrameVector(worldFrame, new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         
         cubic3D.setCubic(t0, tFinal, z0, zd0, zFinal,zdFinal);
         
         double time = t0 + Math.random() * (tFinal - t0);
         
         DenseMatrix64F alphaPrimeAutomatic = new DenseMatrix64F(3, numberOfCoefficients);
         DenseMatrix64F alphaPrimeManual = new DenseMatrix64F(3, numberOfCoefficients);
      
         CapturePointMatrixTools.calculateGeneralizedAlphaPrimeOnCMPSegment(omega0, time, alphaPrimeAutomatic, 0, cubic3D);
         calculateAlphaPrimeCubic(omega0 , time, tFinal, alphaPrimeManual);
         
         PrintTools.debug("A auto: " + alphaPrimeAutomatic);
         PrintTools.debug("A hand: " + alphaPrimeManual);

         for(int j = 0; j < numberOfCoefficients; j++)
         {
            for(int k = 0; k < 3; k++)
            {
               assertEquals(alphaPrimeAutomatic.get(k, j), alphaPrimeManual.get(k, j), EPSILON);
            }
         }
         
         DenseMatrix64F betaPrimeAutomatic = new DenseMatrix64F(3, numberOfCoefficients);
         DenseMatrix64F betaPrimeManual = new DenseMatrix64F(3, numberOfCoefficients);
      
         CapturePointMatrixTools.calculateGeneralizedBetaPrimeOnCMPSegment(omega0, time, betaPrimeAutomatic, 0, cubic3D);
         calculateBetaPrimeCubic(omega0 , time, tFinal, betaPrimeManual);
         
         PrintTools.debug("B auto: " + betaPrimeAutomatic);
         PrintTools.debug("B hand: " + betaPrimeManual);
         
         for(int j = 0; j < numberOfCoefficients; j++)
         {
            for(int k = 0; k < 3; k++)
            {
               assertEquals(betaPrimeAutomatic.get(k, j), betaPrimeManual.get(k, j), EPSILON);
            }
         }
         
         DenseMatrix64F gammaPrimeAutomatic = new DenseMatrix64F(1, 1);
         DenseMatrix64F gammaPrimeManual = new DenseMatrix64F(1, 1);
      
         CapturePointMatrixTools.calculateGeneralizedGammaPrimeOnCMPSegment(omega0, time, gammaPrimeAutomatic, 0, cubic3D);
         calculateGammaPrimeCubic(omega0 , time, tFinal, gammaPrimeManual);
         
         PrintTools.debug("C auto: " + gammaPrimeAutomatic);
         PrintTools.debug("C hand: " + gammaPrimeManual);
         
         assertEquals(gammaPrimeAutomatic.get(0), gammaPrimeManual.get(0), EPSILON);
         
         //EuclidCoreTestTools.assertTuple3DEquals("", arrayToPack.get(i).getPoint3dCopy(), pointBetweenFeet, 1e-10);
      }
   }
      
      
   public void calculateAlphaPrimeLinear(double omega0, double time, double timeTotal, DenseMatrix64F alphaPrimeLinear)
   {
      alphaPrimeLinear.set(0, 0, 1);
      alphaPrimeLinear.set(0, 1, time + 1.0/omega0);
      
      alphaPrimeLinear.set(1, 0, 1);
      alphaPrimeLinear.set(1, 1, time + 1.0/omega0);
      
      alphaPrimeLinear.set(2, 0, 1);
      alphaPrimeLinear.set(2, 1, time + 1.0/omega0);
   }
   
   public void calculateBetaPrimeLinear(double omega0, double time, double timeTotal, DenseMatrix64F betaPrimeLinear)
   {
      betaPrimeLinear.set(0, 0, Math.exp(omega0 * (time - timeTotal))*1);
      betaPrimeLinear.set(0, 1, Math.exp(omega0 * (time - timeTotal))*(timeTotal + 1.0/omega0));
      
      betaPrimeLinear.set(1, 0, Math.exp(omega0 * (time - timeTotal))*1);
      betaPrimeLinear.set(1, 1, Math.exp(omega0 * (time - timeTotal))*(timeTotal + 1.0/omega0));
      
      betaPrimeLinear.set(2, 0, Math.exp(omega0 * (time - timeTotal))*1);
      betaPrimeLinear.set(2, 1, Math.exp(omega0 * (time - timeTotal))*(timeTotal + 1.0/omega0));
   }
   
   public void calculateGammaPrimeLinear(double omega0, double time, double timeTotal, DenseMatrix64F gammaPrimeLinear)
   {
      gammaPrimeLinear.set(0, 0, Math.exp(omega0 * (time - timeTotal)));
   }
   
   public void calculateAlphaPrimeCubic(double omega0, double time, double timeTotal, DenseMatrix64F alphaPrimeLinear)
   {
      alphaPrimeLinear.set(0, 0, 1);
      alphaPrimeLinear.set(0, 1, time + 1.0/omega0);
      alphaPrimeLinear.set(0, 2, Math.pow(time, 2) + 2.0 * time/omega0 + 2.0/Math.pow(omega0, 2));
      alphaPrimeLinear.set(0, 3, Math.pow(time, 3) + 3.0 * Math.pow(time, 2)/omega0 + 6.0 * time/Math.pow(omega0, 2) + 6.0/Math.pow(omega0, 3));
      
      alphaPrimeLinear.set(1, 0, 1);
      alphaPrimeLinear.set(1, 1, time + 1.0/omega0);
      alphaPrimeLinear.set(1, 2, Math.pow(time, 2) + 2.0 * time/omega0 + 2.0/Math.pow(omega0, 2));
      alphaPrimeLinear.set(1, 3, Math.pow(time, 3) + 3.0 * Math.pow(time, 2)/omega0 + 6.0 * time/Math.pow(omega0, 2) + 6.0/Math.pow(omega0, 3));
      
      alphaPrimeLinear.set(2, 0, 1);
      alphaPrimeLinear.set(2, 1, time + 1.0/omega0);
      alphaPrimeLinear.set(2, 2, Math.pow(time, 2) + 2.0 * time/omega0 + 2.0/Math.pow(omega0, 2));
      alphaPrimeLinear.set(2, 3, Math.pow(time, 3) + 3.0 * Math.pow(time, 2)/omega0 + 6.0 * time/Math.pow(omega0, 2) + 6.0/Math.pow(omega0, 3));
   }
   
   public void calculateBetaPrimeCubic(double omega0, double time, double timeTotal, DenseMatrix64F betaPrimeLinear)
   {
      betaPrimeLinear.set(0, 0, Math.exp(omega0 * (time - timeTotal))*1);
      betaPrimeLinear.set(0, 1, Math.exp(omega0 * (time - timeTotal))*(timeTotal + 1.0/omega0));
      betaPrimeLinear.set(0, 2, Math.exp(omega0 * (time - timeTotal))*(Math.pow(timeTotal, 2) + 2.0 * timeTotal/omega0 + 2.0/Math.pow(omega0, 2)));
      betaPrimeLinear.set(0, 3, Math.exp(omega0 * (time - timeTotal))*(Math.pow(timeTotal, 3) + 3.0 * Math.pow(timeTotal, 2)/omega0 + 6.0 * timeTotal/Math.pow(omega0, 2) + 6.0/Math.pow(omega0, 3)));
      
      betaPrimeLinear.set(1, 0, Math.exp(omega0 * (time - timeTotal))*1);
      betaPrimeLinear.set(1, 1, Math.exp(omega0 * (time - timeTotal))*(timeTotal + 1.0/omega0));
      betaPrimeLinear.set(1, 2, Math.exp(omega0 * (time - timeTotal))*(Math.pow(timeTotal, 2) + 2.0 * timeTotal/omega0 + 2.0/Math.pow(omega0, 2)));
      betaPrimeLinear.set(1, 3, Math.exp(omega0 * (time - timeTotal))*(Math.pow(timeTotal, 3) + 3.0 * Math.pow(timeTotal, 2)/omega0 + 6.0 * timeTotal/Math.pow(omega0, 2) + 6.0/Math.pow(omega0, 3)));
      
      betaPrimeLinear.set(2, 0, Math.exp(omega0 * (time - timeTotal))*1);
      betaPrimeLinear.set(2, 1, Math.exp(omega0 * (time - timeTotal))*(timeTotal + 1.0/omega0));
      betaPrimeLinear.set(2, 2, Math.exp(omega0 * (time - timeTotal))*(Math.pow(timeTotal, 2) + 2.0 * timeTotal/omega0 + 2.0/Math.pow(omega0, 2)));
      betaPrimeLinear.set(2, 3, Math.exp(omega0 * (time - timeTotal))*(Math.pow(timeTotal, 3) + 3.0 * Math.pow(timeTotal, 2)/omega0 + 6.0 * timeTotal/Math.pow(omega0, 2) + 6.0/Math.pow(omega0, 3)));
   }
   
   public void calculateGammaPrimeCubic(double omega0, double time, double timeTotal, DenseMatrix64F gammaPrimeLinear)
   {
      gammaPrimeLinear.set(0, 0, Math.exp(omega0 * (time - timeTotal)));
   }
}
