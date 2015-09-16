package us.ihmc.commonWalkingControlModules.terrain;

import java.awt.Color;
import java.io.File;
import java.util.ArrayList;

import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.vecmath.Point2d;

import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphics3DAdapter.HeightMapWithNormals;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.util.ground.AlternatingSlopesGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.BumpyGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.GroundProfileFromFile;
import us.ihmc.simulationconstructionset.util.ground.RandomRockyGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.RollingGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.StairGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.steppingStones.SteppingStone;
import us.ihmc.simulationconstructionset.util.ground.steppingStones.SteppingStones;
import us.ihmc.simulationconstructionset.util.ground.steppingStones.SteppingStonesGroundProfile;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class CommonTerrain
{
   private final SteppingStones steppingStones;
   private final GroundProfile3D groundProfile;

   public CommonTerrain(TerrainType terrainType)
   {
      this.steppingStones = null;
      this.groundProfile = setUpTerrain(terrainType);
   }

   public CommonTerrain(ConvexPolygon2d footPolygon, SteppingStonePattern steppingStonePattern, boolean useSteppingStonesGroundModel)
   {
      this.steppingStones = setUpSteppingStones(footPolygon, steppingStonePattern, useSteppingStonesGroundModel);

      if (useSteppingStonesGroundModel)
      {
         System.out.println("Using a stepping stone ground model");
         groundProfile = new SteppingStonesGroundProfile(steppingStones);
      }
      else
      {
         System.out.println("Drawing SteppingStones but using a flat ground model.");
         groundProfile = new FlatGroundProfile();
      }
   }

   public CommonTerrain(GroundProfile3D groundProfile)
   {
      this.steppingStones = null;
      this.groundProfile = groundProfile;
   }

   public GroundProfile3D getGroundProfile()
   {
      return groundProfile;
   }

   public SteppingStones getSteppingStones()
   {
      return steppingStones;
   }

   public ArrayList<Graphics3DObject> createLinkGraphics(boolean drawGroundBelow)
   {
      if (steppingStones != null)
      {
         ArrayList<Graphics3DObject> linkGraphics = steppingStones.createLinkGraphics();

         if (drawGroundBelow)
         {
            Graphics3DObject groundBelow = new Graphics3DObject();
            groundBelow.translate(0.0, 0.0, -0.2);
            groundBelow.addCube(10.0, 10.0, 0.1, YoAppearance.Green());

            linkGraphics.add(groundBelow);
         }

         return linkGraphics;
      }

      if (groundProfile != null)
      {
         ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();
         
         Graphics3DObject texturedGroundLinkGraphics = new Graphics3DObject();
         
         HeightMapWithNormals heightMap = groundProfile.getHeightMapIfAvailable();
         texturedGroundLinkGraphics.addHeightMap(heightMap, 300, 300, YoAppearance.DarkGreen());
         ret.add(texturedGroundLinkGraphics);
         return ret;
      }

      throw new RuntimeException("Both steppingStones and groundProfile are null!");

   }

   public void registerSteppingStonesArtifact(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      System.out.println("Registering Stepping Stones Artifact");

      SteppingStonesArtifact steppingStonesArtifact = new SteppingStonesArtifact("SteppingStones", steppingStones, Color.magenta, Color.GREEN);
      yoGraphicsListRegistry.registerArtifact("Stepping Stones", steppingStonesArtifact);
   }

   private SteppingStones setUpSteppingStones(ConvexPolygon2d footPolygon, SteppingStonePattern steppingStonePattern, boolean useSteppingStonesGroundModel)
   {
      SteppingStones steppingStones;
      switch (steppingStonePattern)
      {
         case WHOLE_WORLD :
         {
            steppingStones = generateWholeWorldSteppingStones(footPolygon);

            break;
         }

         case SIMPLE_RECTANGULAR_GRID :
         {
            steppingStones = generateSimpleRectangularGridSteppingStones(footPolygon);

            break;
         }

         case GROUP_OF_THREE :
         {
            steppingStones = generateGroupsOfThreeSteppingStones(footPolygon);

            break;
         }

         case SIMPLE_BEAM_BALANCE :
         {
            steppingStones = generateRectangularBeamBalance(footPolygon);

            // steppingStones = generateSimpleRectangularBeamBalance();

            break;
         }

         case STEP_DOWNS :
         {
            steppingStones = generateStepDowns(footPolygon);

            break;
         }

         case CRISS_CROSS_BEAMS :
         {
            steppingStones = generateCrissCrossBeams(footPolygon);

            break;
         }

         default :
         {
            throw new RuntimeException("Shouldn't get here!");
         }
      }
      
      return steppingStones;
   }


   private static final SteppingStones generateGroupsOfThreeSteppingStones(ConvexPolygon2d footPolygon)
   {
      double startXPosition = 0.45;    // 0.65; //0.55; //-0.1;
      double startYPosition = -0.4;    // -0.6;    // -0.9; //-0.75;
      double radius = 0.2;    // 0.25;
      int numSides = 6;
      double baseZ = -0.1;
      double height = 0.0;
      int numStones = 30;    // 60;
      double xBetweenStoneGroups = 0.15;    // 0.2; //0.25;

      // Please don't change these
      boolean MATTS_PUSH_TEST = false;
      if (MATTS_PUSH_TEST)
      {
         startXPosition = 0.45;
         startYPosition = -0.6;
         radius = 0.2;
         numSides = 6;
         baseZ = -0.1;
         height = 0.0;
         numStones = 60;
         xBetweenStoneGroups = 0.15;
      }

      boolean R2_TEST = true;
      if (R2_TEST)
      {
         startXPosition = 0.25;
         startYPosition = -0.2;
         radius = 0.3;    // 0.25;
         numSides = 6;
         baseZ = -0.1;
         height = 0.0;
         numStones = 30;
         xBetweenStoneGroups = 0.15;
      }

      SteppingStones steppingStones = SteppingStones.generateRandomPolygonalRandomPatternOneSteppingStones(startXPosition, startYPosition, radius, numSides,
                                         xBetweenStoneGroups, baseZ, height, numStones, footPolygon, false);

      return steppingStones;


//    steppingStones = SteppingStones.createSampleSteppingStones();
//    steppingStones = SteppingStones.generateRandomSteppingStones(random, 50);
//    steppingStones.addSteppingStone(SteppingStone.generateRandomCicularStone("base", random, 0.5, -0.2, -0.2, 0.0, 1.5));

//    steppingStones = SteppingStones.generateRandomPolygonalUniformSteppingStones(-0.0,-0.8,0.2,6,0.0,0.0,-0.1,0.0,3,20);
//    steppingStones = SteppingStones.generateRandomPolygonalChessBoardSteppingStones(-0.1, -0.75, 0.25, 6, 0.0, -0.15, -0.1, 0.0, 4, 20);
//    steppingStones = SteppingStones.generateRandomPolygonalUniformSteppingStones(-0.0,-0.8,0.2,6,0.0,0.0,-0.1,0.0,3, 1);

//    steppingStones = SteppingStones.generateRandomPolygonalChessBoardSteppingStones(startXPosition, startYPosition, radius, numSides, spacingInX, spacingInY, baseZ, height, numRows, numColumns);
//    steppingStones = SteppingStones.generateRandomPolygonalRandomPatternOneSteppingStones(startXPosition, startYPosition, radius, numSides, xBetweenStoneGroups, baseZ, height, numStones);
//    steppingStones = SteppingStones.generateRandomPolygonalRandomPatternOneSteppingStones(startXPosition, startYPosition, radius, numSides, 0.2, baseZ, height, numStones);
//    steppingStones = SteppingStones.generateRandomPolygonalRandomPatternTwoSteppingStones(startXPosition, startYPosition, radius, numSides, 0.0, baseZ, height, numStones);
   }

   private static final SteppingStones generateWholeWorldSteppingStones(ConvexPolygon2d footPolygon)
   {
      ArrayList<Point2d> points = new ArrayList<Point2d>();

      points.add(new Point2d(-100.0, -100.0));
      points.add(new Point2d(-100.0, 100.0));
      points.add(new Point2d(100.0, -100.0));
      points.add(new Point2d(100.0, 100.0));

      SteppingStone steppingStone = new SteppingStone("wholeWorld", -0.02, 0.0, points, footPolygon);

      SteppingStones steppingStones = new SteppingStones();
      steppingStones.addSteppingStone(steppingStone);

      return steppingStones;
   }

   private static final SteppingStones generateSimpleRectangularGridSteppingStones(ConvexPolygon2d footPolygon)
   {
      double startXPosition = 0.65;    // 0.65; //0.55; //-0.1;
      double startYPosition = -0.92;    // -0.9; //-0.75;
      double xDimension = 0.28;
      double yDimension = 0.18;
      double spacingInX = 0.16;    // 0.1;
      double spacingInY = 0.05;    // -0.15;
      double baseZ = -0.1;
      double height = 0.0;
      int numRows = 8;
      int numColumns = 20;


      SteppingStones steppingStones = SteppingStones.generateRectangularUniformSteppingStones(startXPosition, startYPosition, xDimension, yDimension,
                                         spacingInX, spacingInY, baseZ, height, numRows, numColumns, footPolygon, false);

      return steppingStones;
   }

   @SuppressWarnings("unused")
   private static final SteppingStones generateSimpleRectangularBeamBalance(ConvexPolygon2d footPolygon)
   {
      double startXPosition = 0.65;    // 0.65; //0.55; //-0.1;
      double startYPosition = -0.745;    // -0.9; //-0.75;
      double xDimension = 0.28;
      double yDimension = 0.14;    // 0.16; //0.18;
      double spacingInX = -0.2;    // 0.0; //0.1;
      double spacingInYSmall = 0.05;    // -0.15;
      double spacingInYLarge = 0.1;
      double baseZ = -0.1;
      double height = 0.0;
      int numRows = 4;
      int numColumns = 40;    // 20;

      SteppingStones steppingStones = SteppingStones.generateRectangularBeamBalance(startXPosition, startYPosition, xDimension, yDimension, spacingInX,
                                         spacingInYSmall, spacingInYLarge, baseZ, height, numRows, numColumns, footPolygon);

      return steppingStones;
   }

   private static final SteppingStones generateRectangularBeamBalance(ConvexPolygon2d footPolygon)
   {
      double startXPosition = 0.0;    // 7.5 // 0.65; //0.55; //-0.1;
      double startYPosition = 0.0;    // 0.0 //-0.72 // -0.68;//-0.745; //-0.9; //-0.75;
      double xDimension = 15.0;    // 14.0
      double yDimension = 0.5;    // 0.16 // 0.14; //0.16; //0.18;
      double spacingInX = -0.2;    // -0.2 // 0.0; //0.1;
      double spacingInYSmall = 0.05;    // 0.05 // -0.15;
      double spacingInYLarge = 0.1;    // 0.1
      double baseZ = -0.1;    // -0.1
      double height = 0.0;    // 0.0
      int numRows = 1;    // 4
      int numColumns = 1;    // 20;

      SteppingStones steppingStones = SteppingStones.generateRectangularBeamBalance(startXPosition, startYPosition, xDimension, yDimension, spacingInX,
                                         spacingInYSmall, spacingInYLarge, baseZ, height, numRows, numColumns, footPolygon);

      return steppingStones;
   }

   private static final SteppingStones generateStepDowns(ConvexPolygon2d footPolygon)
   {
      double startXPosition = 0.65;    // 0.65; //0.55; //-0.1;
      double startYPosition = -0.92;    // -0.9; //-0.75;
      double xDimension = 0.28;
      double yDimension = 0.18;
      double spacingInX = 0.16;    // 0.1;
      double spacingInY = 0.05;    // -0.15;
      double baseZ = 0.0;
      double height = 0.0;
      int numRows = 8;
      int numColumns = 20;

      SteppingStones steppingStones = SteppingStones.generateRectangularUniformSteppingStones(startXPosition, startYPosition, xDimension, yDimension,
                                         spacingInX, spacingInY, baseZ, height, numRows, numColumns, footPolygon, true);

      return steppingStones;
   }

   private static final SteppingStones generateCrissCrossBeams(ConvexPolygon2d footPolygon)
   {
      SteppingStones steppingStones = new SteppingStones();

      double startXPosition = 0.95;    // 0.65; //0.55; //-0.1;
      double startYPosition = -2.65;    // -0.9; //-0.75;
      double xDimension = 4.00;
      double yDimension = 0.2;
      double spacingInX = 3.00;    // 0.1;
      double spacingInY = 2.0;    // -0.15;
      double baseZ = -0.1;
      double height = 0.0;
      int numRows = 3;
      int numColumns = 2;

      steppingStones = SteppingStones.generateRectangularCrissCrossBeams(startXPosition, startYPosition, xDimension, yDimension, spacingInX, spacingInY, baseZ,
              height, numRows, numColumns, footPolygon, false);

      return steppingStones;
   }


   public static GroundProfile3D setUpTerrain(TerrainType terrainType)
   {
      GroundProfile3D groundProfile;
      switch (terrainType)
      {
         case FLAT_Z_ZERO :
            groundProfile = new FlatGroundProfile(0.0);

            break;
            
         case FLAT_Z_NEGATIVE_TWO :
            groundProfile = new FlatGroundProfile(-2.0);

            break;

         case ROLLING_HILLS :
            groundProfile = new RollingGroundProfile(0.1, 0.3, 0.0, -20.0, 20.0, -20.0, 20.0);

            break;

         case BUMPY :
            groundProfile = new BumpyGroundProfile(0.04, 0.4, 0.1, 0.2, 0.02, 0.35, 0.0, 5.0, -20.0, 20.0, -20.0, 20.0);

            break;

         case ROCKY :

            groundProfile = new RandomRockyGroundProfile(10);

            break;

         case STEP_UP :
            groundProfile = new StepUpGroundProfile(1.4, 0.1);

            break;

         case STAIR :
              groundProfile = new StairGroundProfile(0.15, 0.35);
//              groundProfile = new ListOfHeightsStairGroundProfile(new double[] {0.1, 0.2, 0.3}, new double[] {0.1, 0.2}, 0.0, 0.0);

            break;

         case ALTERNATING_SLOPES :
         {
            double xMin = -1.0, xMax = 12.0;
            double yMin = -2.0, yMax = 2.0;
            double[][] xSlopePairs = new double[][]
            {
               {1.0, 0.05}, {3.0, 0.0}, {5.0, 0.1}, {7.0, 0.0}, {9.0, 0.1}, {11.0, 0.0}
            };
            groundProfile = new AlternatingSlopesGroundProfile(xSlopePairs, xMin, xMax, yMin, yMax);

            break;
         }

         case DOWNHILL :
         {
            double xMin = -1.0, xMax = 12.0;
            double yMin = -2.0, yMax = 2.0;
            double[][] xSlopePairs = new double[][]
            {
               {1.0, -0.05}, {3.0, 0.0}, {5.0, -0.1}, {7.0, -0.0}, {9.0, -0.1}, {11.0, 0.0}
            };
            groundProfile = new AlternatingSlopesGroundProfile(xSlopePairs, xMin, xMax, yMin, yMax);

            break;
         }

         case STEEP_UPSLOPE :
         {
            double xMin = -1.0, xMax = 10.0;
            double yMin = -2.0, yMax = 2.0;
            double[][] xSlopePairs = new double[][]
            {
               {1.0, 0.2}, {6.0, 0.0}, {10.0, 0.0}
            };
            groundProfile = new AlternatingSlopesGroundProfile(xSlopePairs, xMin, xMax, yMin, yMax);

            break;
         }

         case MOON :
            groundProfile = new GroundProfileFromFile("GroundModels" + File.separator + "moonNew.asc", new RigidBodyTransform());

            break;

         case LOADED :
            JFileChooser fc = new JFileChooser();
            int returnVal = fc.showOpenDialog(new JFrame());

            if (returnVal == JFileChooser.APPROVE_OPTION)
            {
               File file = fc.getSelectedFile();

               // This is where a real application would open the file.
               System.out.println("Opening: " + file.getName());
               groundProfile = new GroundProfileFromFile(file.getAbsolutePath(), new RigidBodyTransform());
            }
            else
            {
               System.out.println("Open command cancelled by user.");
               groundProfile = new FlatGroundProfile();
            }



            break;

         default :
            groundProfile = new FlatGroundProfile();
      }

      return groundProfile;
   }



}
