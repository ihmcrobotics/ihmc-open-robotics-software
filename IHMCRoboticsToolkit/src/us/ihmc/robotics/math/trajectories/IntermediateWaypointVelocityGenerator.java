package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;

/**
 * This velocity generator will generate velocities at intermediate way points of a trajectory which will result in a smooth path trough those way points.
 * It is based on the paper "Curvature-continuous 3D Path-planning Using QPMI Method" which can be found at http://cdn.intechopen.com/pdfs-wm/48591.pdf
 * Assuming the first and the last way point of a path have a velocity of 0, all way points in between are inserted in this velocity generator.
 *
 * @author Olger
 */
public class IntermediateWaypointVelocityGenerator
{
   private final ArrayList<double[]> parametersX;
   private final ArrayList<double[]> parametersY;
   private final ArrayList<double[]> parametersZ;

   private final ArrayList<Vector3D> waypoints;
   private final ArrayList<Vector3D> xDot;
   private final ArrayList<Double> time;

   private final Matrix3D timeMatrix;
   private final int numberOfWayPoints;

   public IntermediateWaypointVelocityGenerator(ArrayList<Double> time, ArrayList<Vector3D> waypoints)
   {
      numberOfWayPoints = waypoints.size();

      parametersX = new ArrayList<>(numberOfWayPoints);
      parametersY = new ArrayList<>(numberOfWayPoints);
      parametersZ = new ArrayList<>(numberOfWayPoints);
      xDot = new ArrayList<>(numberOfWayPoints);
      timeMatrix = new Matrix3D();

      for (int i = 0; i < numberOfWayPoints; i++)
      {
         parametersX.add(new double[3]);
         parametersY.add(new double[3]);
         parametersZ.add(new double[3]);
      }

      this.waypoints = waypoints;
      this.time = time;
   }

   public ArrayList<Vector3D> calculateVelocities()
   {
      ArrayList<Double> subTime = new ArrayList<>(time.subList(0, 3));
      ArrayList<Vector3D> subWayPoints = new ArrayList<>(waypoints.subList(0, 3));
      getParameters(subTime, subWayPoints, parametersX.get(0), parametersY.get(0), parametersZ.get(0));

      for (int i = 1; i < numberOfWayPoints - 1; i++)
      {
         subTime = new ArrayList<>(time.subList(i - 1, i + 2));
         subWayPoints = new ArrayList<>(waypoints.subList(i - 1, i + 2));
         getParameters(subTime, subWayPoints, parametersX.get(i), parametersY.get(i), parametersZ.get(i));
      }

      subTime = new ArrayList<>(time.subList(numberOfWayPoints - 3, numberOfWayPoints));
      subWayPoints = new ArrayList<>(waypoints.subList(numberOfWayPoints - 3, numberOfWayPoints));
      getParameters(subTime, subWayPoints, parametersX.get(numberOfWayPoints - 1), parametersY.get(numberOfWayPoints - 1),
            parametersZ.get(numberOfWayPoints - 1));

      for (int i = 0; i < numberOfWayPoints; i++)
      {
         xDot.add(new Vector3D(2 * parametersX.get(i)[0] * time.get(i) + parametersX.get(i)[1], 2 * parametersY.get(i)[0] * time.get(i) + parametersY.get(i)[1],
               2 * parametersZ.get(i)[0] * time.get(i) + parametersZ.get(i)[1]));
      }

      return xDot;
   }

   private void getParameters(ArrayList<Double> time, ArrayList<Vector3D> waypoints, double[] parametersXToPack, double[] parametersYToPack,
         double[] parametersZToPack)
   {
      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            timeMatrix.setElement(i, j, MathTools.pow(time.get(i), 2 - j));
         }
      }

      System.out.println(timeMatrix);
      timeMatrix.invert();
      Vector3D tempVector = new Vector3D();

      for (int i = 0; i < 3; i++)
      {
         timeMatrix.getRow(i, tempVector);
         parametersXToPack[i] = (tempVector.dot(new Vector3D(waypoints.get(0).getX(), waypoints.get(1).getX(), waypoints.get(2).getX())));
      }
      for (int i = 0; i < 3; i++)
      {
         timeMatrix.getRow(i, tempVector);
         parametersYToPack[i] = (tempVector.dot(new Vector3D(waypoints.get(0).getY(), waypoints.get(1).getY(), waypoints.get(2).getY())));
      }
      for (int i = 0; i < 3; i++)
      {
         timeMatrix.getRow(i, tempVector);
         parametersZToPack[i] = (tempVector.dot(new Vector3D(waypoints.get(0).getZ(), waypoints.get(1).getZ(), waypoints.get(2).getZ())));
      }
   }

   public void printResultsForTest()
   {
      System.out.println(timeMatrix);
      System.out.println(time.get(0) + " : " + time.get(1) + " : " + time.get(2));
      System.out.println(parametersX.get(1)[0] + " : " + parametersX.get(1)[1] + " : " + parametersX.get(1)[2]);
      System.out.println(parametersY.get(1)[0] + " : " + parametersY.get(1)[1] + " : " + parametersY.get(1)[2]);
      System.out.println(parametersZ.get(1)[0] + " : " + parametersZ.get(1)[1] + " : " + parametersZ.get(1)[2]);

      System.out.println(parametersX.get(1)[0] * MathTools.square(time.get(0)) + parametersX.get(1)[1] * time.get(0) + parametersX.get(1)[2]);
      System.out.println(parametersX.get(1)[0] * MathTools.square(time.get(1)) + parametersX.get(1)[1] * time.get(1) + parametersX.get(1)[2]);
      System.out.println(parametersX.get(1)[0] * MathTools.square(time.get(2)) + parametersX.get(1)[1] * time.get(2) + parametersX.get(1)[2]);

      System.out.println(2 * parametersX.get(1)[0] * time.get(0) + parametersX.get(1)[1]);
      System.out.println(2 * parametersX.get(1)[0] * time.get(1) + parametersX.get(1)[1]);
      System.out.println(2 * parametersX.get(1)[0] * time.get(2) + parametersX.get(1)[1]);

      System.out.println("final check");
      System.out.println(xDot);
   }

}
