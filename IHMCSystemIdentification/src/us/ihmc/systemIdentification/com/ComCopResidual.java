/**
 * This is a residual function built on top of DataBuffer and Associate Robot object in from SCS instance
 * 
 *  residual =  (model predicted CenterOfMass point) - (measured Center-of-pressure) 
 * 
 *  When the robot is static, the projection of CoM on the flatground should be equal to CoP
 *  
 */
package us.ihmc.systemIdentification.com;

import java.awt.BorderLayout;
import java.awt.Color;
import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.JScrollPane;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ddogleg.optimization.functions.FunctionNtoM;

import us.ihmc.graphics3DDescription.plotting.artifact.CircleArtifact;
import us.ihmc.plotting.Plotter;
import us.ihmc.plotting.PlotterPanel;
import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;

/** Todo
 * 
 * @author tingfan
 *
 * Ideally this class should cache all the values once loaded from dataBuffer (or some other constructors)
 * Or there should be an independent class independent from dataBuffer, so we can unittest it;
 * But the fact Robot model is tie with dataBuffer make it really confusing to do so. 
 */

public class ComCopResidual implements FunctionNtoM
{
   static boolean lockComY = false;
   private Robot robot;
   private Link targetLink;
   private DataBuffer dataBuffer;
   private int[] selectedFrames;

   /**
    * 
    * @param robot - the robot model
    * @param linkName - the link to be ID'ed
    * @param dataBuffer - scs.getDataBuffer();
    * @param numSubsampleBetweenInOut: <0 (use keyPoints) >0 (equally spaced between in/out points);
    */
   public ComCopResidual(Robot robot, String linkName, DataBuffer dataBuffer, int numSubsampleBetweenInOut)
   {
      this.dataBuffer = dataBuffer;

      this.robot = robot;
      targetLink = robot.getLink(linkName);
      if (targetLink == null)
      {
         System.err.println("target link not found " + linkName);
         throw new RuntimeException("target link not found");
      }

      System.out.println("target link " + targetLink.getName() + "mass " + targetLink.getMass() + "kg, com " + getCurrentLinkCom());

      //select subframes
      ArrayList<Integer> keyPoints = dataBuffer.getKeyPoints();
      if (numSubsampleBetweenInOut < 0)
      {
         System.out.println("Using key frames");
         selectedFrames = new int[keyPoints.size()];
         for (int i = 0; i < keyPoints.size(); i++)
            selectedFrames[i] = keyPoints.get(i);
      }
      else
      {
         System.out.println("Using equally spaced frames between in/out");
         if (numSubsampleBetweenInOut > dataBuffer.getBufferInOutLength())
         {
            numSubsampleBetweenInOut = dataBuffer.getBufferInOutLength();
            System.err.println("truncate numberSubSampleBetweenInOut to dataBufferSize " + dataBuffer.getBufferInOutLength());
         }

         selectedFrames = new int[numSubsampleBetweenInOut];
         for (int i = 0; i < numSubsampleBetweenInOut; i++)
         {
            selectedFrames[i] = (int) Math.floor(i * dataBuffer.getBufferInOutLength() / numSubsampleBetweenInOut);
            keyPoints.add(selectedFrames[i]);
         }
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
         dataBuffer.setIndexButDoNotNotifySimulationRewoundListeners(selectedFrames[i]);
         // model predicted CoM
         robot.update(); //this pull data from dataBuffer magically through YoVariables
         Point3d modelCoM = new Point3d();
         robot.computeCenterOfMass(modelCoM);
         outCom.add(modelCoM);

         // sensedCoP
         Point3d sensedCoP = new Point3d(dataBuffer.getVariable("sensedCoPX").getValueAsDouble(), dataBuffer.getVariable("sensedCoPY").getValueAsDouble(),
               dataBuffer.getVariable("sensedCoPZ").getValueAsDouble());
         outCop.add(sensedCoP);
      }
   }

   @Override
   public void process(double[] inParameter, double[] outError)
   {
      if (lockComY)
      {
         Vector3d lastCom = new Vector3d();
         targetLink.getComOffset(lastCom);
         targetLink.setComOffset(inParameter[0], lastCom.getY(), inParameter[2]);
      }
      else
      {
         targetLink.setComOffset(new Vector3d(inParameter));
      }
      ArrayList<Point3d> com = new ArrayList<>(selectedFrames.length);
      ArrayList<Point3d> cop = new ArrayList<>(selectedFrames.length);

      packRobotComCopSeries(com, cop);

      for (int i = 0; i < com.size(); i++)
      {
         outError[2 * i] = cop.get(i).getX() - com.get(i).getX();
         outError[2 * i + 1] = cop.get(i).getY() - com.get(i).getY();
      }
   }

   @Override
   public int getNumOfInputsN()
   {
      return 3; //x,y,z
   }

   @Override
   public int getNumOfOutputsM()
   {
      // dim error
      return getNumSamples() * 2; //x and y, 2d
   }

   public int getNumSamples()
   {
      return selectedFrames.length;
   }

   //Plot model-pred - measurement on a 2d graph
   public void showSample(int numPlotSample, String frameTitle)
   {

      Plotter plotter = createPlotter(frameTitle);
      // filter out zero velocity region
      int nSamples = getNumSamples();
      ArrayList<Point3d> com = new ArrayList<>(nSamples);
      ArrayList<Point3d> cop = new ArrayList<>(nSamples);
      packRobotComCopSeries(com, cop);

      for (int i = 0; i < nSamples; i += nSamples / numPlotSample)
      {
         plotter.addArtifact(new CircleArtifact("sensedCoP" + i, cop.get(i).getX(), cop.get(i).getY(), 0.005, true, Color.RED));
         plotter.addArtifact(new CircleArtifact("modelCoM" + i, com.get(i).getX(), com.get(i).getY(), 0.01, false, Color.RED));
      }
   }

   static int plotterPanelId = 0;

   public static Plotter createPlotter(String frameTitle)
   {
      PlotterPanel plotterPanel = new PlotterPanel();
      Plotter plotter = plotterPanel.getPlotter();
      plotter.setViewRange(1.0);
      if (frameTitle == null)
      {
         frameTitle = "Plotter Panel " + plotterPanelId;
      }
      JFrame f = new JFrame(frameTitle);
      f.getContentPane().add(new JScrollPane(plotterPanel), BorderLayout.CENTER);
      f.pack();
      f.setVisible(true);
      return plotter;
   }
}
