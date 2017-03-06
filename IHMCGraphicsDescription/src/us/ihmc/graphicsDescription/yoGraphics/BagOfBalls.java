package us.ihmc.graphicsDescription.yoGraphics;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 *
 * <p>
 * Title: BagOfBalls
 * </p>
 *
 * <p>
 * Description: Class for displaying multiple points in the SCS GUI.
 * </p>
 *
 * <p>
 * Copyright: Copyright (c) 2006-2009
 * </p>
 *
 * <p>
 * Company: IHMC
 * </p>
 *
 * @author IHMC LearningLocomotion Team
 * @version 1.0
 */
public class BagOfBalls
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double DEFAULT_SIZE = 0.01;
   private static final int DEFAULT_NUMBER_OF_BALLS = 50;
   private static final String DEFAULT_NAME = "BagOfBalls";
   private static final AppearanceDefinition DEFAULT_COLOR = YoAppearance.Black();
   private static final GraphicType DEFAULT_GRAPHIC_TYPE = null;

   private final ArrayList<YoGraphicPosition> dynamicGraphicPositions = new ArrayList<>();
   private int index;
   private boolean outOfBallsWarning = false;
   private YoGraphicsList yoGraphicsList;
   private ArtifactList artifactList;

   public BagOfBalls(YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(DEFAULT_NUMBER_OF_BALLS, DEFAULT_SIZE, DEFAULT_NAME, DEFAULT_COLOR, DEFAULT_GRAPHIC_TYPE, parentRegistry, yoGraphicsListRegistry);
   }

   public BagOfBalls(int numberOfBalls, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(numberOfBalls, DEFAULT_SIZE, DEFAULT_NAME, DEFAULT_COLOR, DEFAULT_GRAPHIC_TYPE, parentRegistry, yoGraphicsListRegistry);
   }

   public BagOfBalls(int numberOfBalls, double sizeInMeters, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(numberOfBalls, sizeInMeters, DEFAULT_NAME, DEFAULT_COLOR, DEFAULT_GRAPHIC_TYPE, parentRegistry, yoGraphicsListRegistry);
   }

   public BagOfBalls(int numberOfBalls, double sizeInMeters, String name, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(numberOfBalls, sizeInMeters, name, DEFAULT_COLOR, DEFAULT_GRAPHIC_TYPE, parentRegistry, yoGraphicsListRegistry);
   }

   public BagOfBalls(int numberOfBalls, double sizeInMeters, AppearanceDefinition appearance, YoVariableRegistry parentRegistry,
                     YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(numberOfBalls, sizeInMeters, DEFAULT_NAME, appearance, DEFAULT_GRAPHIC_TYPE, parentRegistry, yoGraphicsListRegistry);
   }

   public BagOfBalls(int numberOfBalls, double sizeInMeters, AppearanceDefinition appearance, GraphicType graphicType, YoVariableRegistry parentRegistry,
                     YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(numberOfBalls, sizeInMeters, DEFAULT_NAME, appearance, graphicType, parentRegistry, yoGraphicsListRegistry);
   }

   /**
    * Creates a BagOfBalls with the given number of balls, and all the balls with the given
    * Appearance.
    *
    * @param numberOfBalls int Number of balls to create.
    * @param sizeInMeters double Size of each ball in meters.
    * @param name String Name of the BagOfBalls
    * @param appearance Appearance for each of the balls.
    * @param parentRegistry YoVariableRegistry to register the BagOfBalls with.
    * @param yoGraphicsListRegistry YoGraphicsListRegistry to register the BagOfBalls
    *           with.
    */
   public BagOfBalls(int numberOfBalls, double sizeInMeters, String name, AppearanceDefinition appearance, YoVariableRegistry parentRegistry,
                     YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(numberOfBalls, sizeInMeters, name, appearance, DEFAULT_GRAPHIC_TYPE, parentRegistry, yoGraphicsListRegistry);
   }

   /**
    * Creates a BagOfBalls with the given number of balls, and all the balls with the given
    * Appearance.
    *
    * @param numberOfBalls int Number of balls to create.
    * @param sizeInMeters double Size of each ball in meters.
    * @param name String Name of the BagOfBalls
    * @param appearance Appearance for each of the balls.
    * @param parentRegistry YoVariableRegistry to register the BagOfBalls with.
    * @param yoGraphicsListRegistry YoGraphicsListRegistry to register the BagOfBalls
    *           with.
    */
   public BagOfBalls(int numberOfBalls, double sizeInMeters, String name, AppearanceDefinition appearance, GraphicType graphicType,
                     YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(sizeInMeters, name, toList(appearance, numberOfBalls), graphicType, parentRegistry, yoGraphicsListRegistry);
   }

   /**
    * Creates a BagOfBalls with the size being the same as the number of Appearances given.
    *
    * @param sizeInMeters double Size of each ball in meters.
    * @param name String Name of the BagOfBalls
    * @param appearances ArrayList of the Appearance for each of the balls.
    * @param parentRegistry YoVariableRegistry to register the BagOfBalls with.
    * @param yoGraphicsListRegistry YoGraphicsListRegistry to register the BagOfBalls
    *           with.
    */
   public BagOfBalls(double sizeInMeters, String name, List<AppearanceDefinition> appearances, YoVariableRegistry parentRegistry,
                     YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(sizeInMeters, name, appearances, DEFAULT_GRAPHIC_TYPE, parentRegistry, yoGraphicsListRegistry);
   }

   public BagOfBalls(double sizeInMeters, String name, List<AppearanceDefinition> appearances, GraphicType graphicType, YoVariableRegistry parentRegistry,
                     YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(name + "Balls");

      for (int i = 0; i < appearances.size(); i++)
      {
         YoFramePoint yoFramePoint = new YoFramePoint(name + i, "", worldFrame, registry);
         YoGraphicPosition newPosition;
         if (graphicType != null)
            newPosition = new YoGraphicPosition(name + i, yoFramePoint, sizeInMeters, appearances.get(i), graphicType);
         else
            newPosition = new YoGraphicPosition(name + i, yoFramePoint, sizeInMeters, appearances.get(i));

         dynamicGraphicPositions.add(newPosition);
      }

      index = 0;

      registerYoGraphics(name, registry, parentRegistry, yoGraphicsListRegistry);
      if (graphicType != null)
         registerArtifacts(name, yoGraphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   private static List<AppearanceDefinition> toList(AppearanceDefinition appearanceDefinition, int size)
   {
      List<AppearanceDefinition> list = new ArrayList<>(size);
      while (list.size() < size)
         list.add(appearanceDefinition);
      return list;
   }

   /**
    * Create a Bag of Balls with alternating ball color going through Red, White, and Blue.
    *
    * @param numberOfBalls int Number of balls to create.
    * @param sizeInMeters double Size of each ball in meters.
    * @param name String Name of the BagOfBalls to create.
    * @param parentYoVariableRegistry YoVariableRegistry to register the BagOfBalls with.
    * @param yoGraphicsListRegistry YoGraphicsListRegistry to register the BagOfBalls
    *           with.
    * @return BagOfBalls
    */
   public static BagOfBalls createPatrioticBag(int numberOfBalls, double sizeInMeters, String name, YoVariableRegistry parentYoVariableRegistry,
                                               YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      AppearanceDefinition[] redWhiteBlue = new AppearanceDefinition[] {YoAppearance.Red(), YoAppearance.White(), YoAppearance.Blue()};

      ArrayList<AppearanceDefinition> appearances = new ArrayList<>();

      for (int i = 0; i < numberOfBalls; i++)
      {
         appearances.add(redWhiteBlue[i % redWhiteBlue.length]);
      }

      return new BagOfBalls(sizeInMeters, name, appearances, parentYoVariableRegistry, yoGraphicsListRegistry);
   }

   /**
    * Create a Bag of Balls with alternating ball color going through the cycle of the colors of the
    * rainbow.
    *
    * @param numberOfBalls int Number of balls to create.
    * @param sizeInMeters double Size of each ball in meters.
    * @param name String Name of the BagOfBalls to create.
    * @param parentYoVariableRegistry YoVariableRegistry to register the BagOfBalls with.
    * @param yoGraphicsListRegistry YoGraphicsListRegistry to register the BagOfBalls
    *           with.
    * @return BagOfBalls
    */
   public static BagOfBalls createRainbowBag(int numberOfBalls, double sizeInMeters, String name, YoVariableRegistry parentYoVariableRegistry,
                                             YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      AppearanceDefinition[] rainbow = YoAppearance.getStandardRoyGBivRainbow();

      ArrayList<AppearanceDefinition> appearances = new ArrayList<>();

      for (int i = 0; i < numberOfBalls; i++)
      {
         appearances.add(rainbow[i % rainbow.length]);
      }

      return new BagOfBalls(sizeInMeters, name, appearances, parentYoVariableRegistry, yoGraphicsListRegistry);
   }

   private void registerYoGraphics(String name, YoVariableRegistry registry, YoVariableRegistry parentYoVariableRegistry,
                                   YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (yoGraphicsListRegistry != null)
      {
         yoGraphicsList = new YoGraphicsList(name + "Balls");

         for (YoGraphicPosition dynamicGraphicPosition : dynamicGraphicPositions)
         {
            yoGraphicsList.add(dynamicGraphicPosition);
         }

         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      }
   }

   private void registerArtifacts(String name, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (yoGraphicsListRegistry != null)
      {
         artifactList = new ArtifactList(name + "Balls");

         for (YoGraphicPosition dynamicGraphicPosition : dynamicGraphicPositions)
         {
            artifactList.add(dynamicGraphicPosition.createArtifact());
         }

         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }
   }

   public void setBall(FramePoint location, int ballIndex)
   {
      setBall(location, null, ballIndex);
   }

   /**
    * Sets the next ball to the given location. If all the balls have been set, then does nothing.
    *
    * @param location FramePoint to set the next ball to.
    */
   public void setBall(FramePoint location)
   {
      setBall(location, null);
   }

   public void setBall(FramePoint location, AppearanceDefinition appearance)
   {
      setBall(location, appearance, index);
      index++;
   }

   public void setBall(Point3D pointToTestAboveRamp)
   {
      setBall(new FramePoint(worldFrame, pointToTestAboveRamp));
   }

   public void setBall(Point3D pointToTestAboveRamp, AppearanceDefinition appearance)
   {
      setBall(new FramePoint(worldFrame, pointToTestAboveRamp), appearance);
   }

   /**
    * Sets the next ball to the given location, and gives it the given appearance. If all the balls
    * have been set, then does nothing.
    *
    * @param location FramePoint to set the next ball to.
    * @param appearance Appearance to give the next ball.
    */
   public void setBall(FramePoint location, AppearanceDefinition appearance, int ballIndex)
   {
      //TODO: PDN, note that with current implementation of JME, you can only "set" the appearance once. After that, it will ignore all appearance sets
      if (!location.getReferenceFrame().isWorldFrame())
         throw new RuntimeException(location + " must be in a World Frame!");

      setBall(location.getX(), location.getY(), location.getZ(), appearance, ballIndex);
   }

   public void setBall(double x, double y, double z)
   {
      setBall(x, y, z, null);
   }

   public void setBall(double x, double y, double z, AppearanceDefinition appearance)
   {
      setBall(x, y, z, appearance, index);
      index++;
   }

   public void setBall(double x, double y, double z, AppearanceDefinition appearance, int ballIndex)
   {
      if (ballIndex < dynamicGraphicPositions.size())
      {
         YoGraphicPosition dynamicGraphicPosition = dynamicGraphicPositions.get(ballIndex);
         dynamicGraphicPosition.setPosition(x, y, z);
         if (appearance != null)
            dynamicGraphicPosition.setAppearance(appearance);
      }
      else
      {
         if (!outOfBallsWarning)
         {
            //          System.err.println("Bag of Balls doesn't have enough footstep graphic positions!");
            outOfBallsWarning = true;
         }
      }
   }

   /**
    * Sets the next ball to the given location. If all the balls have been set, then loops to the
    * first ball.
    *
    * @param location FramePoint to set the next ball to.
    */
   public void setBallLoop(FramePoint location)
   {
      setBallLoop(location, null);
   }

   /**
    * Sets the next ball to the given location with the given Appearance. If all the balls have been
    * set, then loops to the first ball.
    *
    * @param location FramePoint to set the next ball to.
    * @param appearance Appearance to give the next ball.
    */
   public void setBallLoop(FramePoint location, AppearanceDefinition appearance)
   {
      location.changeFrame(worldFrame);

      if (index >= dynamicGraphicPositions.size())
      {
         index = 0;
      }

      YoGraphicPosition dynamicGraphicPosition = dynamicGraphicPositions.get(index);
      dynamicGraphicPosition.setPosition(location);
      if (appearance != null)
         dynamicGraphicPosition.setAppearance(appearance);

      index++;
   }

   /**
    * Resets by placing all the balls at (0, 0, 0) and making the first ball be the next ball to
    * place.
    */
   public void reset()
   {
      index = 0;

      for (int i = 0; i < dynamicGraphicPositions.size(); i++)
      {
         YoGraphicPosition yoGraphicPosition = dynamicGraphicPositions.get(i);
         yoGraphicPosition.setPosition(Double.NaN, Double.NaN, Double.NaN);
      }
   }

   /**
    * Hides all the balls.
    */
   public void hideAll()
   {
      index = 0;
      for (int i = 0; i < dynamicGraphicPositions.size(); i++)
      {
         dynamicGraphicPositions.get(i).setPositionToNaN();
      }
   }

   public void setVisible(boolean visible)
   {
      index = 0;
      yoGraphicsList.setVisible(visible);
   }

   public int getNumberOfBalls()
   {
      return dynamicGraphicPositions.size();
   }
}
