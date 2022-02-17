package us.ihmc.exampleSimulations.singgleLeggedRobot;

import org.jfree.ui.RefineryUtilities;

public class TestPage
{    
  
   public static void main(String[] args)
   {
      
      

      MatrixForML matPlot;
      

      SinggleLeggedTrajectoryGenerator trajectoryGen;

      trajectoryGen = new SinggleLeggedTrajectoryGenerator();
      
      double[][] alpha = {{0, -60.2570, 737.1044, -1029, 755, 0}};
      MatrixForML matAlpha = new MatrixForML(1,6);
      matAlpha.set(alpha);
      
      
      double currentTime = 10.0;
      double currentPos = 0.4298;
      double currentVel = 0.0;
      double stanceTime = 0.275;
      trajectoryGen.setBazierPolynomialTrajectory(currentVel , currentPos, currentTime, stanceTime, matAlpha);
      
      int idx = (int) (stanceTime/0.005);
      double[][] temp = new double[2][idx];

      for (int i = 0; i<idx; i++)
      {
         temp[0][i] = i*0.005;
         temp[1][i] = trajectoryGen.bazierPolynomialTrajectory(currentTime);
         currentTime += 0.005;
      }
      
      matPlot = new MatrixForML(2,idx);
      matPlot.set(temp);
   
      final TestPlot demo = new TestPlot("z trajectory",matPlot);
      demo.pack();
      RefineryUtilities.centerFrameOnScreen(demo);
      demo.setVisible(true);
 
 
      
      
      
      
      
      
      
      
      
      

      
      
      
      
      
   }
   
   
}
