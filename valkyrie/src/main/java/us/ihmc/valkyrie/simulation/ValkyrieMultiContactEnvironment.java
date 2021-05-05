package us.ihmc.valkyrie.simulation;

import us.ihmc.euclid.Axis3D;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.robotSide.RobotSide;

public enum ValkyrieMultiContactEnvironment
{
   FLAT_GROUND,
   FLAT_IN_FRONT,
   _30DEG_SLOPE_IN_FRONT,
   FLAT_ON_SIDE,
   _30DEG_SLOPE_ON_SIDE,
   GROUND_AND_WALLS,
   TILTED_TILES,
   BOXES_FOR_HELP_GETTING_DOWN,
   FLAT_HAND_HOLDS,
   TILTED_HANDHOLDS;

   public static ValkyrieMultiContactEnvironment environment = FLAT_HAND_HOLDS;

   public static PlanarRegionsList createPlanarRegions()
   {
      return environment.createPlanarRegionsInternal();
   }

   private PlanarRegionsList createPlanarRegionsInternal()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      switch (this)
      {
         case GROUND_AND_WALLS:
         {
            generator.addRectangle(5.0, 5.0);

            double wallHeight = 2.0;
            double wallSpacing = 1.5;

            generator.translate(0.0, 0.0, 0.5 * wallHeight);
            for (RobotSide robotSide : RobotSide.values)
            {
               generator.translate(0.0, 0.5 * robotSide.negateIfRightSide(wallSpacing), 0.0);
               generator.rotate(robotSide.negateIfRightSide(0.5 * Math.PI), Axis3D.X);
               generator.addRectangle(1.0, wallHeight);
               generator.rotate(robotSide.negateIfLeftSide(0.5 * Math.PI), Axis3D.X);
               generator.translate(0.0, 0.5 * robotSide.negateIfLeftSide(wallSpacing), 0.0);
            }
            generator.translate(0.0, 0.0, -0.5 * wallHeight);

            generator.translate(0.9, 0.0, 0.3);
            generator.rotate(-0.25 * Math.PI, Axis3D.Y);
            generator.addRectangle(0.7, 0.7);

            return generator.getPlanarRegionsList();
         }
         case FLAT_IN_FRONT:
         {
            generator.addRectangle(5.0, 5.0);

            double offsetX = 0.5;
            double boxDepth = 0.5;
            double boxWidth = 1.0;
            double boxHeight = 0.25;

            generator.translate(offsetX + 0.5 * boxDepth, 0.0, 0.0);
            generator.addCubeReferencedAtBottomMiddle(boxDepth, boxWidth, boxHeight);
            return generator.getPlanarRegionsList();
         }
         case _30DEG_SLOPE_IN_FRONT:
         {
            generator.addRectangle(5.0, 5.0);

            double angle = Math.toRadians(30.0);
            generator.translate(0.5, 0.0, 0.15);
            generator.rotate(-angle, Axis3D.Y);

            double lengthX = 0.7;
            generator.translate(0.5 * lengthX, 0.0, 0.0);
            generator.addRectangle(lengthX, 1.0);
            return generator.getPlanarRegionsList();
         }
         case FLAT_ON_SIDE:
         {
            generator.addRectangle(5.0, 5.0);

            double offsetX = 0.2;
            double offsetY = 0.5;
            double boxLength = 1.0;
            double boxWidth = 0.5;
            double boxHeight = 0.25;

            for (RobotSide robotSide : RobotSide.values)
            {
               generator.identity();
               generator.translate(offsetX, robotSide.negateIfRightSide(offsetY + 0.5 * boxWidth), 0.0);
               generator.addCubeReferencedAtBottomMiddle(boxLength, boxWidth, boxHeight);
            }

            return generator.getPlanarRegionsList();
         }
         case BOXES_FOR_HELP_GETTING_DOWN:
         {
            generator.addRectangle(5.0, 5.0);

            double offsetX = 0.3;
            double offsetY = 0.35;
            double boxLength = 0.3;
            double boxWidth = 0.5;

            for (RobotSide robotSide : RobotSide.values)
            {
               generator.identity();
               generator.translate(offsetX, robotSide.negateIfRightSide(offsetY + 0.5 * boxWidth), 0.0);
               generator.addCubeReferencedAtBottomMiddle(boxLength, boxWidth, 0.35);

               generator.translate(boxLength, 0.0, 0.0);
               generator.addCubeReferencedAtBottomMiddle(boxLength, boxWidth, 0.18);

               generator.translate(boxLength, 0.0, 0.0);
               generator.addCubeReferencedAtBottomMiddle(boxLength, boxWidth, 0.10);
            }

            return generator.getPlanarRegionsList();
         }
         case FLAT_HAND_HOLDS:
         {
            generator.addRectangle(5.0, 5.0);

            double boxHeight = 0.91;
            double boxWidth = 0.3;
            double offsetX = 0.35;
            double offsetY = 0.45;
            double offsetZ = 0.7;

            for (RobotSide robotSide : RobotSide.values())
            {
               generator.identity();
               generator.translate(offsetX, robotSide.negateIfRightSide(offsetY), offsetZ);
               generator.addCubeReferencedAtBottomMiddle(boxWidth, boxWidth, boxHeight - offsetZ);
            }

            return generator.getPlanarRegionsList();
         }

         case TILTED_HANDHOLDS:
         {
            generator.addRectangle(5.0, 5.0);

            double boxHeight = 0.85;
            double boxWidth = 0.3;
            double offsetX = 0.0;
            double offsetY = 0.45;
            double offsetZ = 0.7;
            double angle = Math.toRadians(45.0);

            for (RobotSide robotSide : RobotSide.values())
            {
               generator.identity();
               generator.translate(offsetX, robotSide.negateIfRightSide(offsetY), offsetZ);
               generator.rotate(angle, Axis3D.Y);
               generator.addCubeReferencedAtBottomMiddle(boxWidth, boxWidth, boxHeight - offsetZ);
            }

            return generator.getPlanarRegionsList();
         }

         case FLAT_GROUND:
         default:
         {
            generator.translate(0.0, 0.0, 0.0);
            generator.addRectangle(5.0, 5.0);
            return generator.getPlanarRegionsList();
         }
      }
   }

}
