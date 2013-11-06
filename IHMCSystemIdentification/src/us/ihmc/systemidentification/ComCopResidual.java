/**
 * This is a residual function built on top of DataBuffer and Associate Robot object in from SCS instance
 * 
 *  residual =  (model predicted CenterOfMass point) - (measured Center-of-pressure) 
 * 
 *  When the robot is static, the projection of CoM on the flatground should be equal to CoP
 *  
 */
package us.ihmc.systemidentification;

import java.awt.BorderLayout;
import java.awt.Color;
import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.JScrollPane;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ddogleg.optimization.functions.FunctionNtoM;

import us.ihmc.plotting.Plotter;
import us.ihmc.plotting.PlotterPanel;
import us.ihmc.plotting.shapes.CircleArtifact;

import com.yobotics.simulationconstructionset.DataBuffer;
import com.yobotics.simulationconstructionset.Link;
import com.yobotics.simulationconstructionset.Robot;

public class ComCopResidual implements FunctionNtoM
{
   private Robot robot;
   private Link targetLink;
   private DataBuffer dataBuffer;
   private int[] selectedFrames;
   

   public ComCopResidual(Robot robot, String linkName, DataBuffer dataBuffer, int numSubsampleBetweenInOut)
   {
      this.dataBuffer = dataBuffer;

      this.robot = robot;
      targetLink = robot.getLink(linkName);
      if(targetLink==null)
      {
         System.err.println("target link not found " + linkName);
         throw new RuntimeException("target link not found");
      }
      
      System.out.println("target link "+ targetLink.getName()+"mass " + targetLink.getMass() + "kg, com " + getCurrentLinkCom());

      //select subframes
      if (numSubsampleBetweenInOut > dataBuffer.getBufferInOutLength())
      {
         numSubsampleBetweenInOut = dataBuffer.getBufferInOutLength();
         System.err.println("truncate numberSubSampleBetweenInOut to dataBufferSize " + dataBuffer.getBufferInOutLength());
      }
      
      selectedFrames = new int[numSubsampleBetweenInOut];
      for(int i=0;i<numSubsampleBetweenInOut; i++ )
      {
         selectedFrames[i] = (int)Math.floor(i*dataBuffer.getBufferInOutLength()/numSubsampleBetweenInOut);
      }
   }
   
   public Vector3d getCurrentLinkCom()
   {
      Vector3d comOffset = new Vector3d();
      targetLink.getComOffset(comOffset);
      return comOffset;
   }
   
   public void packRobotComCopSeries(ArrayList<Point3d> outCom, ArrayList<Point3d> outCop)
   {
      outCom.clear();
      outCop.clear();
      for (int i = 0; i < selectedFrames.length; i++)
      {
         dataBuffer.setIndex(selectedFrames[i]);
         // model predicted CoM
         robot.update();
         Point3d modelCoM = new Point3d();
         robot.computeCenterOfMass(modelCoM);
         outCom.add(modelCoM);
         
         // sensedCoP
         Point3d sensedCoP = new Point3d(
               dataBuffer.getVariable("sensedCoPX").getValueAsDouble(),
               dataBuffer.getVariable("sensedCoPY").getValueAsDouble(),
               dataBuffer.getVariable("sensedCoPZ").getValueAsDouble());
         outCop.add(sensedCoP);
      }
   }
   

   @Override
   public void process(double[] inParameter, double[] outError)
   {
      targetLink.setComOffset(new Vector3d(inParameter));
      ArrayList<Point3d> com = new ArrayList<>(selectedFrames.length);
      ArrayList<Point3d> cop = new ArrayList<>(selectedFrames.length);
      
      packRobotComCopSeries(com, cop);

      for(int i=0;i<com.size();i++)
      {
         outError[2*i  ] = cop.get(i).x - com.get(i).x;
         outError[2*i+1] = cop.get(i).y - com.get(i).y;
      }
   }

   @Override
   public int getN()
   {
      // dim parameter

      return 3; //x,y,z
   }

   
   @Override
   public int getM()
   {
      // dim error
      return getNumSamples() *2; //x and y, 2d
   }
   
   public int getNumSamples()
   {
      return selectedFrames.length;
   }
   
   //Plot model-pred - measurement on a 2d graph
   public void showSample(int numPlotSample)
   {

      Plotter plotter = createPlotter();
      // filter out zero velocity region
      int nSamples =getNumSamples();
      ArrayList<Point3d> com = new ArrayList<>(nSamples);
      ArrayList<Point3d> cop = new ArrayList<>(nSamples);
      packRobotComCopSeries(com, cop);

      for (int i = 0; i < nSamples; i += nSamples/numPlotSample )
      {
         plotter.addArtifact(new CircleArtifact("sensedCoP" + i, cop.get(i).x, cop.get(i).y, 0.005, true, Color.RED));
         plotter.addArtifact(new CircleArtifact("modelCoM"  + i, com.get(i).x, com.get(i).y,  0.01, false, Color.RED));
      }
   }

   
   
   public static Plotter createPlotter()
   {
      PlotterPanel plotterPanel = new PlotterPanel();
      Plotter plotter = plotterPanel.getPlotter();
      plotter.setRangeLimit(1, 2, -.2, .2, .2, -.2);
      JFrame f = new JFrame("Plotter Panel");
      f.getContentPane().add(new JScrollPane(plotterPanel), BorderLayout.CENTER);
      f.pack();
      f.setVisible(true);
      return plotter;
   }
}

