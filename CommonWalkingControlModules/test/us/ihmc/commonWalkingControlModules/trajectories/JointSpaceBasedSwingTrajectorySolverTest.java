package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertTrue;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JFrame;
import javax.vecmath.Point2d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.plotting.Plotter;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.trajectories.ParametricSplineTrajectory;
import us.ihmc.robotics.trajectories.ParametricSplineTrajectorySolver;
import us.ihmc.simulationconstructionset.util.graphs.JFreeGraph;
import us.ihmc.simulationconstructionset.util.graphs.JFreeGraphGroup;
import us.ihmc.simulationconstructionset.util.graphs.JFreePlot;

/**
 * Created by agrabertilton on 2/5/15.
 */
@ContinuousIntegrationPlan(categories = {IntegrationCategory.EXCLUDE})
public class JointSpaceBasedSwingTrajectorySolverTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testOneDoFTrajectory()
   {
      boolean VISUALIZE = false;
      double tolerance = 1e-13;

      int numberOfJoints = 1;
      double startTime = 0;
      double[] startPosition = {0.0};
      double[] startVelocity = {0.0};
      double[] startAcceleration = {0.0};

      double[] middlePosition = {1.0};

      double endTime = 1.0;
      double[] endPosition = {0.0};
      double[] endVelocity = {0.0};
      double[] endAcceleration = {0.0};

      double middleTime = startTime + (0.5) * (endTime - startTime);
      double highKneeTime = startTime + (0.75) * (endTime - startTime);
      double[] highKneePosition = {1.0};

      int numberOfConstraint = 8;
      ParametricSplineTrajectorySolver solver = new ParametricSplineTrajectorySolver(3, 3, numberOfConstraint, numberOfJoints);
      solver.setPositionConstraint(startTime, startPosition);
      solver.setVelocityConstraint(startTime, startVelocity);
      solver.setAccelerationConstraint(startTime, startAcceleration);
      solver.setPositionConstraint(endTime, endPosition);
      solver.setVelocityConstraint(endTime, endVelocity);
      solver.setAccelerationConstraint(endTime, endAcceleration);
      solver.setPositionConstraint(middleTime, middlePosition);
      solver.setPositionConstraint(highKneeTime, highKneePosition);

      ParametricSplineTrajectory trajectory = solver.computeTrajectory();

      assertArrayEquals(startPosition, trajectory.getPositions(startTime), tolerance);
      assertArrayEquals(startVelocity, trajectory.getVelocities(startTime), tolerance);
      assertArrayEquals(startAcceleration, trajectory.getAccelerations(startTime), tolerance);
      assertArrayEquals(middlePosition, trajectory.getPositions((startTime + endTime) / 2.0), tolerance);
      assertArrayEquals(endPosition, trajectory.getPositions(endTime), tolerance);
      assertArrayEquals(endVelocity, trajectory.getVelocities(endTime), tolerance);
      assertArrayEquals(endAcceleration, trajectory.getAccelerations(endTime), tolerance);

      if (VISUALIZE)
      {
         List<Point2d> positionValues = new ArrayList<Point2d>();
         int discretization = 100;
         double time;
         double position;
         for (int i = 0; i <= discretization; i++)
         {
            time = startTime + (endTime - startTime) * (i * 1.0 / discretization);
            position = (trajectory.getPositions(time))[0];
            positionValues.add(new Point2d(time, position));
         }

         plotValues(positionValues);
         try
         {
            Thread.sleep(100000);
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testPendulumTrajectory()
   {
      boolean VISUALIZE = false;
      double tolerance = 1e-12;

      int numberOfJoints = 2;
      double length1 = 1.0;
      double length2 = 1.0;

      double startTime = 0;
      double endTime = 1.0;

      //hip angle, knee angle, pelvis x
      double startPelvis = 0;
      double endPelvis = 0;

      double[] startPosition = {Math.PI / 8, 0.0};
      double[] startVelocity = {0.0, 0.0};
      double[] startAcceleration = {0.0, 0.0};

      double[] endPosition = {-Math.PI / 8, 0.0}; //Math.PI/4};
      double[] endVelocity = {0.0, 0.0};
      double[] endAcceleration = {0.0, 0.0};

      double middleTime = (startTime + endTime) / 2.0;
      double[] middlePosition = {-Math.PI / 8, Math.PI / 2};
      double highKneeBend = Math.PI / 2;
      double[] highKneePosition = {-highKneeBend / 2.0 + endPosition[0] / 2, highKneeBend};

      ParametricSplineTrajectorySolver solver = new ParametricSplineTrajectorySolver(3, 3, 7, numberOfJoints);
      int numberOfPoints = solver.getNumberOfSplines() + 1;
      double[] times = new double[numberOfPoints];
      for (int i = 0; i < times.length; i++)
      {
         times[i] = Double.NaN;
      }
      times[0] = startTime;
      times[times.length - 1] = endTime;
      times[2] = middleTime;
      solver.setTimes(times);

      solver.setPositionConstraint(startTime, startPosition);
      solver.setVelocityConstraint(startTime, startVelocity);
      solver.setAccelerationConstraint(startTime, startAcceleration);
      solver.setPositionConstraint(endTime, endPosition);
      solver.setVelocityConstraint(endTime, endVelocity);
      solver.setAccelerationConstraint(endTime, endAcceleration);
      solver.setPositionConstraint(middleTime, middlePosition);

      ParametricSplineTrajectory trajectory = solver.computeTrajectory();

      assertArrayEquals(startPosition, trajectory.getPositions(startTime), tolerance);
      assertArrayEquals(startVelocity, trajectory.getVelocities(startTime), tolerance);
      assertArrayEquals(startAcceleration, trajectory.getAccelerations(startTime), tolerance);
      assertArrayEquals(middlePosition, trajectory.getPositions((startTime + endTime) / 2.0), tolerance);
      assertArrayEquals(endPosition, trajectory.getPositions(endTime), tolerance);
      assertArrayEquals(endVelocity, trajectory.getVelocities(endTime), tolerance);
      assertArrayEquals(endAcceleration, trajectory.getAccelerations(endTime), tolerance);

      if (VISUALIZE)
      {
         List<double[]> positionValues = new ArrayList<double[]>();
         ArrayList<Point2d> pelvisPositions = new ArrayList<Point2d>();

         int discretization = 100;
         double time;
         Point2d cartesianPosition = new Point2d();
         double[] currentPositions;
         double[] currentVelocities;
         double[] currentAcceleratinos;
         double[] timeValues = new double[discretization + 1];
         double[] hipAngles = new double[discretization + 1];
         double[] kneeAngles = new double[discretization + 1];
         double[] hipVelocities = new double[discretization + 1];
         double[] kneeVelocities = new double[discretization + 1];
         double[] hipAccelerations = new double[discretization + 1];
         double[] kneeAccelerations = new double[discretization + 1];

         for (int i = 0; i <= discretization; i++)
         {
            time = startTime + (endTime - startTime) * (i * 1.0 / discretization);
            pelvisPositions.add(new Point2d(startPelvis + (endPelvis - startPelvis) * (i * 1.0 / discretization), 0));
            currentPositions = trajectory.getPositions(time);
            currentVelocities = trajectory.getVelocities(time);
            currentAcceleratinos = trajectory.getAccelerations(time);

            positionValues.add(currentPositions);
            timeValues[i] = time;
            hipAngles[i] = currentPositions[0];
            kneeAngles[i] = currentPositions[1];
            hipVelocities[i] = currentVelocities[0];
            kneeVelocities[i] = currentVelocities[1];
            hipAccelerations[i] = currentAcceleratinos[0];
            kneeAccelerations[i] = currentAcceleratinos[1];
         }
         visualizeInvertedPendulum(length1, length2, pelvisPositions, positionValues);

         //visualize joint angles

         JFreeGraph jointPositionGraph = new JFreeGraph("Joint Angle Graph", "Time", "Joint Angles");
         JFreeGraph jointVelocityGraph = new JFreeGraph("Joint Velocity Graph", "Time", "Joint Velocities");
         JFreeGraph jointAccelerationGraph = new JFreeGraph("Joint Acceleration Graph", "Time", "Joint Accelerations");

         JFreePlot hipAnglePlot = new JFreePlot("Hip Angle", timeValues, hipAngles);
         JFreePlot hipVelocityPlot = new JFreePlot("Hip Velocity", timeValues, hipVelocities);
         JFreePlot hipAccelerationPlot = new JFreePlot("Hip Acceleration", timeValues, hipAccelerations);
         hipAnglePlot.setColor(Color.BLUE);
         hipVelocityPlot.setColor(Color.BLUE);
         hipAccelerationPlot.setColor(Color.BLUE);

         JFreePlot kneeAnglePlot = new JFreePlot("Knee Angle", timeValues, kneeAngles);
         JFreePlot kneeVelocityPlot = new JFreePlot("Knee Velocity", timeValues, kneeVelocities);
         JFreePlot kneeAccelerationPlot = new JFreePlot("Knee Acceleration", timeValues, kneeAccelerations);
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

         try
         {
            Thread.sleep(1000000);
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }
      }
   }

   private void visualizeInvertedPendulum(double length1, double length2, ArrayList<Point2d> hipPositions, List<double[]> jointAngles)
   {
      assertTrue(hipPositions.size() == jointAngles.size());
      Plotter p = new Plotter();
      Point2d hipPosition = new Point2d();
      Point2d kneePosition = new Point2d();
      Point2d anklePosition = new Point2d();

      int index = 0;
      for (double[] jointAngleSet : jointAngles)
      {
         hipPosition = hipPositions.get(index);
         kneePosition.setX(hipPosition.getX() - length1 * Math.sin(jointAngleSet[0]));
         kneePosition.setY(hipPosition.getY() - length1 * Math.cos(jointAngleSet[0]));
         anklePosition.setX(kneePosition.getX() - length2 * Math.sin(jointAngleSet[0] + jointAngleSet[1]));
         anklePosition.setY(kneePosition.getY() - length2 * Math.cos(jointAngleSet[0] + jointAngleSet[1]));

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

      f.getContentPane().add(p.getJPanel(), BorderLayout.CENTER);
      f.pack();
      f.setVisible(true);
   }

   private void plotValues(List<Point2d> points)
   {
      plotValues(points, Color.blue);
   }

   private void plotValues(List<Point2d> points, Color color)
   {
      Plotter p = new Plotter();
      int index = 0;
      for (Point2d point : points)
      {
         p.createAndAddPointArtifact("Point" + (index++), point, color);
      }

      JFrame f = new JFrame("Plotter Test");
      f.addWindowListener(new WindowAdapter()
      {
         public void windowClosing(WindowEvent e)
         {
            System.exit(0);
         }
      });

      f.getContentPane().add(p.getJPanel(), BorderLayout.CENTER);
      f.pack();
      f.setVisible(true);
   }
}
