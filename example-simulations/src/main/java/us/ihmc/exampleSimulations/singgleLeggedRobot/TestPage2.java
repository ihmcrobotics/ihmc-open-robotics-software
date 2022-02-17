package us.ihmc.exampleSimulations.singgleLeggedRobot;

import org.jfree.ui.RefineryUtilities;

public class TestPage2
{
   
   public static void main(String[] args)
   {
      MPCSolversForSinggleLeggedRobot MPCSolver = new MPCSolversForSinggleLeggedRobot();
      double f0;
      double initialPosition = 0.425;
      double currentPosition = initialPosition;
      double previousVelocity = 0.0;
      double currentVelocity = 0.0;
      double desiredPosition = 0.35;
       
      double tempDesiredPosition = 0.0;
      double prevTempDesiredPosition;
      
      double dT = 0.005;
      double mass = 5.0;
      double g = 9.81;
      
      double penalty = 1.0;
      
      int k = 10;
      int totalIteration = 200;
      MatrixForML desiredForce = new MatrixForML(2,totalIteration);
      MatrixForML positionError = new MatrixForML(2,totalIteration);
      MatrixForML desiredPositionSeq = new MatrixForML(k,1);
      MatrixForML desiredPos = new MatrixForML(2,totalIteration);
      MatrixForML currentPos = new MatrixForML(2,totalIteration);
      MatrixForML positionTrajectory = new MatrixForML(400,1);
      for(int i = 0; i<400; i++)
      {
         if (i < 50)
         {
            positionTrajectory.set(i, 0, initialPosition + (desiredPosition - initialPosition) * (i / 50.0));
         }
         else
         {
            positionTrajectory.set(i, 0, desiredPosition);
         }
      }
      
      for (int i =0; i<totalIteration;i++)
      {
         System.out.println(i);
         
         for (int j = 0 ; j<k ; j++)
         {
            desiredPositionSeq.set(j,0, positionTrajectory.getDoubleValue(i+j , 0));
         }
         tempDesiredPosition = desiredPositionSeq.getDoubleValue(0, 0);
         f0 = MPCSolver.GradientDescentMethodVer1(k, currentPosition, currentVelocity, previousVelocity, dT, mass, penalty, desiredPositionSeq);
//         f0 = MPCSolver.LMMethod(k, currentPosition, currentVelocity, previousVelocity, dT, mass, penalty, desiredPositionSeq);
         currentPosition = currentPosition + dT * dT / mass * f0 + dT * previousVelocity - dT * dT * g;
         System.out.println(previousVelocity);
         previousVelocity = previousVelocity + dT*(f0 / mass - g);
         System.out.println(currentPosition);
         System.out.println(f0);
         desiredForce.set(0,i,i*dT);
         desiredForce.set(1,i,f0);
         desiredPos.set(0,i,i*dT);
         desiredPos.set(1,i,tempDesiredPosition);
         
         currentPos.set(0,i,i*dT);
         currentPos.set(1,i,currentPosition);
         
         positionError.set(0,i,tempDesiredPosition - currentPosition + 0.0);
      }
      positionError = desiredPos.sub(currentPos);
      for (int i =0 ; i<totalIteration ; i++)
      {
         positionError.set(0,i,i*dT);
      }
      
      TestPlot demo1 = new TestPlot("desiredPos",desiredPos);
      demo1.pack();
      RefineryUtilities.centerFrameOnScreen(demo1);
      demo1.setVisible(true);
      
      TestPlot demo2 = new TestPlot("currentPos",currentPos);
      demo2.pack();
      RefineryUtilities.centerFrameOnScreen(demo2);
      demo2.setVisible(true);
      
      TestPlot demo3 = new TestPlot("desiredForce",desiredForce);
      demo3.pack();
      RefineryUtilities.centerFrameOnScreen(demo3);
      demo3.setVisible(true);
      
      TestPlot demo4 = new TestPlot("position Error",positionError);
      demo4.pack();
      RefineryUtilities.centerFrameOnScreen(demo4);
      demo4.setVisible(true);
      
      
      
   }
}

