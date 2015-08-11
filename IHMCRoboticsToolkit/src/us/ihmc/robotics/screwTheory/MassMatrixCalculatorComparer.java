package us.ihmc.robotics.screwTheory;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.Random;

public class MassMatrixCalculatorComparer
{
   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);

   private final Random random = new Random(1776L);
   private final ArrayList<MassMatrixCalculator> massMatrixCalculators = new ArrayList<MassMatrixCalculator>();
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ArrayList<RevoluteJoint> joints;
   private final RigidBody elevator;

   public MassMatrixCalculatorComparer()
   {
      joints = new ArrayList<RevoluteJoint>();
      elevator = new RigidBody("elevator", worldFrame);
      Vector3d[] jointAxes = {X, Y, Z, Z, X, Z, Z, X, Y, Y};
      ScrewTestTools.createRandomChainRobot("", joints, elevator, jointAxes, random);

      massMatrixCalculators.add(new DifferentialIDMassMatrixCalculator(worldFrame, elevator));
      massMatrixCalculators.add(new CompositeRigidBodyMassMatrixCalculator(elevator));
   }

   public void compare()
   {
      for (MassMatrixCalculator massMatrixCalculator : massMatrixCalculators)
      {
         long startTime = System.nanoTime();
         int nIterations = 10000;
         for (int i = 0; i < nIterations; i++)
         {
            ScrewTestTools.setRandomPositions(joints, random);
            elevator.updateFramesRecursively();
            massMatrixCalculator.compute();
         }

         long endTime = System.nanoTime();
         double timeTaken = (endTime - startTime) / (1e9);
         double timeTakenPerIteration = timeTaken / nIterations;
         System.out.println("Time taken per iteration: " + timeTakenPerIteration + " s");
      }
   }

   public static void main(String[] args)
   {
      new MassMatrixCalculatorComparer().compare();
   }
}
