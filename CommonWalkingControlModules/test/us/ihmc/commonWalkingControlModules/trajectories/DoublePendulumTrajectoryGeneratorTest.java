package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.assertTrue;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.ArrayList;

import javax.swing.JFrame;
import javax.vecmath.Point2d;

import org.junit.Test;

import us.ihmc.plotting.Plotter;
import us.ihmc.simulationconstructionset.util.graphs.JFreeGraph;
import us.ihmc.simulationconstructionset.util.graphs.JFreeGraphGroup;
import us.ihmc.simulationconstructionset.util.graphs.JFreePlot;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.robotics.geometry.Line2d;

/**
 * Created by agrabertilton on 2/9/15.
 */

@BambooPlan(planType={BambooPlanType.Exclude})
public class DoublePendulumTrajectoryGeneratorTest
{
   @EstimatedDuration(duration = 1.0)
   @Test (timeout=600000)
   public void testHybridPendulum()
   {
      double length1 = 1.0;
      double comRadius1 = 0.5;
      double mass1 = 1.0;
      double momentOfInertia1 = 0.0;

      double length2 = 1.0;
      double comRadius2 = 0.5;
      double mass2 = 1.0;
      double momentOfInertia2 = 0.0;


      double startTime = 0;
      double[] startPosition = {Math.PI / 6, 0.0};
      double[] startVelocity = {0.0, 0.0};

      double endTime = 1.0;
      double[] endPosition = {-Math.PI / 6, 0.0};
      double[] endVelocity = {0.0, 0.0};

      double middleTime = startTime + (endTime - startTime) * 0.5;
      double[] middlePosition = {0.0, Math.PI / 2.0};
      double[] middleVelocity = {0.0, 0.0};

      HybridDoublePendulumTrajectoryGenerator trajectoryGenerator = new HybridDoublePendulumTrajectoryGenerator( length1, comRadius1, mass1, momentOfInertia1, length2, comRadius2, mass2, momentOfInertia2);
      trajectoryGenerator.setStartValues(startTime, startPosition, startVelocity);
      trajectoryGenerator.setEndValues(endTime, endPosition, endVelocity);
      trajectoryGenerator.setMidpoint(middleTime, middlePosition, middleVelocity);
      trajectoryGenerator.computeTrajectory();

      double[] timeValues = trajectoryGenerator.getTimeValues();
      ArrayList<double[]> jointAngles = new ArrayList<>();
      ArrayList<double[]> jointVelocities = new ArrayList<>();
      ArrayList<double[]> jointAccelerations = new ArrayList<>();
      ArrayList<Point2d> pelvisPosition = new ArrayList<>();
      for (int i = 0; i < timeValues.length; i++){
         jointAngles.add(trajectoryGenerator.getPosition(timeValues[i]));
         jointVelocities.add(trajectoryGenerator.getVelocity(timeValues[i]));
         jointAccelerations.add(trajectoryGenerator.getAcceleration(timeValues[i]));
         pelvisPosition.add(new Point2d());
      }
      visualizeInvertedPendulum(length1, length2, pelvisPosition, jointAngles, 20);
      visualizeJointGraphs(timeValues, jointAngles, jointVelocities, jointAccelerations);

      try
      {
         Thread.sleep(1000000);
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }

   @EstimatedDuration(duration = 1.0)
   @Test (timeout=600000)
   public void testNoTorquePendulum()
   {
      double length1 = 1.0;
      double comRadius1 = 0.5;
      double mass1 = 1.0;
      double momentOfInertia1 = 0.0;

      double length2 = 1.0;
      double comRadius2 = 0.5;
      double mass2 = 1.0;
      double momentOfInertia2 = 0.0;


      double startTime = 0;
      double[] startPosition = {-Math.PI / 6, 0.0};
      double[] startVelocity = {0.0, 0.0};

      double endTime = 1.0;
      double[] endPosition = {Math.PI / 6, 0.0};
      double[] endVelocity = {0.0, 0.0};

      DoublePendulumTrajectoryGenerator trajectoryGenerator = new DoublePendulumTrajectoryGenerator( length1, comRadius1, mass1, momentOfInertia1, length2, comRadius2, mass2, momentOfInertia2);
      trajectoryGenerator.setStartValues(startTime, endPosition, endVelocity);
      trajectoryGenerator.setEndValues(endTime, endPosition, endVelocity);
      trajectoryGenerator.computeTrajectory();

      double[] timeValues = trajectoryGenerator.getTimeValues();
      ArrayList<double[]> jointAngles = new ArrayList<>();
      ArrayList<Point2d> pelvisPosition = new ArrayList<>();
      for (int i = 0; i < timeValues.length; i++){
         jointAngles.add(trajectoryGenerator.getPosition(timeValues[i]));
         pelvisPosition.add(new Point2d());
      }
      visualizeInvertedPendulum(length1, length2, pelvisPosition, jointAngles, 20);

      try
      {
         Thread.sleep(1000000);
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }

   private void visualizeInvertedPendulum(double length1, double length2, ArrayList<Point2d > hipPositions, ArrayList<double[]> jointAngles, int skipCount){
      assertTrue(hipPositions.size() == jointAngles.size());
      Plotter p = new Plotter();
      Point2d hipPosition = new Point2d();
      Point2d kneePosition = new Point2d();
      Point2d anklePosition = new Point2d();


      double[] jointAngleSet;
      for (int index = 0; index < jointAngles.size(); index += skipCount){
         hipPosition = hipPositions.get(index);
         jointAngleSet = jointAngles.get(index);
         kneePosition.setX(hipPosition.x -length1 * Math.sin(jointAngleSet[0]));
         kneePosition.setY(hipPosition.y -length1 * Math.cos(jointAngleSet[0]));
         anklePosition.setX(kneePosition.x - length2 * Math.sin(jointAngleSet[0] + jointAngleSet[1]));
         anklePosition.setY(kneePosition.y - length2 * Math.cos(jointAngleSet[0] + jointAngleSet[1]));

         p.createAndAddLineArtifact("Femur" + index, new Line2d(hipPosition, kneePosition), Color.black);
         p.createAndAddLineArtifact("Shin" + index, new Line2d(kneePosition, anklePosition), Color.gray);
         p.createAndAddPointArtifact("Ankle" + index, new Point2d(anklePosition), Color.blue);

         index++;
      }

      JFrame f = new JFrame("Plotter Test");
      f.addWindowListener(new WindowAdapter()
      {
         public void windowClosing(WindowEvent e)
         {
            System.exit(0);
         }
      });

      f.getContentPane().add(p, BorderLayout.CENTER);
      f.pack();
      f.setVisible(true);
   }

   private void visualizeJointGraphs(double[] times, ArrayList<double[]> jointAngles, ArrayList<double[]> jointVelocities, ArrayList<double[]> jointAccelerations){
      double[] hipAngles = new double[times.length];
      double[] kneeAngles = new double[times.length];
      double[] hipVelocities = new double[times.length];
      double[] kneeVelocities = new double[times.length];
      double[] hipAccelerations = new double[times.length];
      double[] kneeAccelerations = new double[times.length];

      for (int i = 0; i < times.length; i++){
         hipAngles[i] = jointAngles.get(i)[0];
         kneeAngles[i] = jointAngles.get(i)[1];
         hipVelocities[i] = jointVelocities.get(i)[0];
         kneeVelocities[i] = jointVelocities.get(i)[1];
         hipAccelerations[i] = jointAccelerations.get(i)[0];
         kneeAccelerations[i] = jointAccelerations.get(i)[1];
      }

      //visualize joint angles

      JFreeGraph jointPositionGraph = new JFreeGraph("Joint Angle Graph", "Time", "Joint Angles");
      JFreeGraph jointVelocityGraph = new JFreeGraph("Joint Velocity Graph", "Time", "Joint Velocities");
      JFreeGraph jointAccelerationGraph = new JFreeGraph("Joint Acceleration Graph", "Time", "Joint Accelerations");

      JFreePlot hipAnglePlot = new JFreePlot("Hip Angle", times, hipAngles);
      JFreePlot hipVelocityPlot = new JFreePlot("Hip Velocity", times, hipVelocities);
      JFreePlot hipAccelerationPlot = new JFreePlot("Hip Acceleration", times, hipAccelerations);
      hipAnglePlot.setColor(Color.BLUE);
      hipVelocityPlot.setColor(Color.BLUE);
      hipAccelerationPlot.setColor(Color.BLUE);

      JFreePlot kneeAnglePlot = new JFreePlot("Knee Angle", times, kneeAngles);
      JFreePlot kneeVelocityPlot = new JFreePlot("Knee Velocity", times, kneeVelocities);
      JFreePlot kneeAccelerationPlot = new JFreePlot("Knee Acceleration", times, kneeAccelerations);
      kneeAnglePlot.setColor(Color.GREEN);
      kneeVelocityPlot.setColor(Color.GREEN);
      kneeAccelerationPlot.setColor(Color.GREEN);


      jointPositionGraph.addPlot(hipAnglePlot);
      jointPositionGraph.addPlot(kneeAnglePlot);
      jointVelocityGraph.addPlot(hipVelocityPlot);
      jointVelocityGraph.addPlot(kneeVelocityPlot);
      jointAccelerationGraph.addPlot(hipAccelerationPlot);
      jointAccelerationGraph.addPlot(kneeAccelerationPlot);

      JFreeGraphGroup graphGroup = new JFreeGraphGroup();
      graphGroup.addGraph(jointPositionGraph);
      graphGroup.addGraph(jointVelocityGraph);
      graphGroup.addGraph(jointAccelerationGraph);

      JFrame frame = new JFrame("Plotter Test");
      frame.setSize(1024, 768);
      frame.getContentPane().add(graphGroup);
      frame.setVisible(true);
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
   }
}
