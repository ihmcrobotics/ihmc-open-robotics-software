package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedOrderedJointMap;
import us.ihmc.robotics.partNames.QuadrupedJointName;

public class GenericQuadrupedSquaredUpInitialPosition extends GenericQuadrupedInitialPositionParameters
{
   private final Point3D initialBodyPosition;
   private final Quaternion initialBodyOrientation;

   public GenericQuadrupedSquaredUpInitialPosition()
   {
      initialBodyPosition = new Point3D(0.0, 0.0, 0.566);
      initialBodyOrientation = new Quaternion();
   }

   @Override
   public Point3D getInitialBodyPosition()
   {
      return initialBodyPosition;
   }

   @Override
   public Quaternion getInitialBodyOrientation()
   {
      return initialBodyOrientation;
   }

   @Override
   double getHipRollAngle()
   {
      return -0.1;
   }

   @Override
   double getHipPitchAngle()
   {
      return 0.363;
   }

   @Override
   double getKneePitchAngle()
   {
      return -1.275;
   }

   public static void main(String[] args)
   {
      GenericQuadrupedSquaredUpInitialPosition initialPositionParameters = new GenericQuadrupedSquaredUpInitialPosition();
      System.out.println("Initial position: " + initialPositionParameters.getInitialBodyPosition());
      System.out.println("Initial orientation: " + initialPositionParameters.getInitialBodyOrientation());

      System.out.println("Initial positions:");
      for (GenericQuadrupedOrderedJointMap jointName : GenericQuadrupedOrderedJointMap.values)
      {
         QuadrupedJointName quadrupedJointName = QuadrupedJointName.getName(jointName.getRobotQuadrant(), jointName.getLegJointName());
         double initialJointPosition = initialPositionParameters.getInitialJointPosition(quadrupedJointName);
         System.out.println(quadrupedJointName + "\t\t" + initialJointPosition);
      }
   }
}
