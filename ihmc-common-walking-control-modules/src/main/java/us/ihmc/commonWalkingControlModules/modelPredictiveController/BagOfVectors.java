package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

/**
 * <p>
 * Title: BagOfVectors
 * </p>
 * <p>
 * Description: Class for displaying multiple points in the SCS GUI.
 * </p>
 */
public class BagOfVectors
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double DEFAULT_SIZE = 0.01;
   private static final int DEFAULT_NUMBER_OF_VECTORS = 50;
   private static final String DEFAULT_NAME = "BagOfVectors";
   private static final AppearanceDefinition DEFAULT_COLOR = YoAppearance.Black();

   private final ArrayList<YoGraphicVector> yoGraphicVectors = new ArrayList<>();
   private int index;
   private boolean outOfVectorsWarning = false;
   private YoGraphicsList yoGraphicsList;
   private ArtifactList artifactList;

   public BagOfVectors(YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(DEFAULT_NUMBER_OF_VECTORS, DEFAULT_SIZE, DEFAULT_NAME, DEFAULT_COLOR, parentRegistry, yoGraphicsListRegistry);
   }

   public BagOfVectors(int numberOfVectors, YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(numberOfVectors, DEFAULT_SIZE, DEFAULT_NAME, DEFAULT_COLOR, parentRegistry, yoGraphicsListRegistry);
   }

   public BagOfVectors(int numberOfVectors, double sizeInMeters, YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(numberOfVectors, sizeInMeters, DEFAULT_NAME, DEFAULT_COLOR, parentRegistry, yoGraphicsListRegistry);
   }

   public BagOfVectors(int numberOfVectors, double sizeInMeters, String name, YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(numberOfVectors, sizeInMeters, name, DEFAULT_COLOR, parentRegistry, yoGraphicsListRegistry);
   }

   public BagOfVectors(int numberOfVectors, double sizeInMeters, AppearanceDefinition appearance, YoRegistry parentRegistry,
                       YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(numberOfVectors, sizeInMeters, DEFAULT_NAME, appearance, parentRegistry, yoGraphicsListRegistry);
   }

   /**
    * Creates a BagOfVectors with the given number of balls, and all the balls with the given Appearance.
    *
    * @param numberOfVectors        int Number of balls to create.
    * @param sizeInMeters           double Size of each ball in meters.
    * @param name                   String Name of the BagOfVectors.
    * @param appearance             Appearance for each of the balls.
    * @param parentRegistry         YoRegistry to register the BagOfVectors with.
    * @param yoGraphicsListRegistry YoGraphicsListRegistry to register the BagOfVectors with.
    */
   public BagOfVectors(int numberOfVectors, double sizeInMeters, String name, AppearanceDefinition appearance,
                       YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(sizeInMeters, name, toList(appearance, numberOfVectors), parentRegistry, yoGraphicsListRegistry);
   }

   /**
    * Creates a BagOfVectors with the size being the same as the number of Appearances given.
    *
    * @param sizeInMeters           double Size of each vector in meters.
    * @param name                   String Name of the BagOfVectors
    * @param appearances            ArrayList of the Appearance for each of the balls.
    * @param parentRegistry         YoRegistry to register the BagOfVectors with.
    * @param yoGraphicsListRegistry YoGraphicsListRegistry to register the BagOfVectors with.
    */
   public BagOfVectors(double sizeInMeters, String name, List<AppearanceDefinition> appearances, YoRegistry parentRegistry,
                       YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoRegistry registry = new YoRegistry(name + "Vectors");

      for (int i = 0; i < appearances.size(); i++)
      {
         YoFramePoint3D yoFramePoint = new YoFramePoint3D(name + i, "Point", worldFrame, registry);
         YoFrameVector3D yoFrameVector = new YoFrameVector3D(name + i, "Vector", worldFrame, registry);

         YoGraphicVector newVector = new YoGraphicVector(name + i, yoFramePoint, yoFrameVector, sizeInMeters, appearances.get(i));

         yoGraphicVectors.add(newVector);
      }

      index = 0;

      registerYoGraphics(name, yoGraphicsListRegistry);

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
    * Create a Bag of Vectors with alternating ball color going through Red, White, and Blue.
    *
    * @param numberOfVectors          int Number of balls to create.
    * @param sizeInMeters             double Size of each ball in meters.
    * @param name                     String Name of the BagOfVectors to create.
    * @param parentYoVariableRegistry YoRegistry to register the BagOfVectors with.
    * @param yoGraphicsListRegistry   YoGraphicsListRegistry to register the BagOfVectors with.
    * @return BagOfVectors
    */
   public static BagOfVectors createPatrioticBag(int numberOfVectors, double sizeInMeters, String name, YoRegistry parentYoVariableRegistry,
                                                 YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      AppearanceDefinition[] redWhiteBlue = new AppearanceDefinition[] {YoAppearance.Red(), YoAppearance.White(), YoAppearance.Blue()};

      ArrayList<AppearanceDefinition> appearances = new ArrayList<>();

      for (int i = 0; i < numberOfVectors; i++)
      {
         appearances.add(redWhiteBlue[i % redWhiteBlue.length]);
      }

      return new BagOfVectors(sizeInMeters, name, appearances, parentYoVariableRegistry, yoGraphicsListRegistry);
   }

   /**
    * Create a Bag of Balls with alternating ball color going through the cycle of the colors of the
    * rainbow.
    *
    * @param numberOfBalls            int Number of balls to create.
    * @param sizeInMeters             double Size of each ball in meters.
    * @param name                     String Name of the BagOfBalls to create.
    * @param parentYoVariableRegistry YoRegistry to register the BagOfBalls with.
    * @param yoGraphicsListRegistry   YoGraphicsListRegistry to register the BagOfBalls with.
    * @return BagOfBalls
    */
   public static BagOfVectors createRainbowBag(int numberOfBalls, double sizeInMeters, String name, YoRegistry parentYoVariableRegistry,
                                               YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      AppearanceDefinition[] rainbow = YoAppearance.getStandardRoyGBivRainbow();

      ArrayList<AppearanceDefinition> appearances = new ArrayList<>();

      for (int i = 0; i < numberOfBalls; i++)
      {
         appearances.add(rainbow[i % rainbow.length]);
      }

      return new BagOfVectors(sizeInMeters, name, appearances, parentYoVariableRegistry, yoGraphicsListRegistry);
   }

   private void registerYoGraphics(String name, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (yoGraphicsListRegistry != null)
      {
         yoGraphicsList = new YoGraphicsList(name + "Balls");

         for (YoGraphicVector yoGraphicPosition : yoGraphicVectors)
         {
            yoGraphicsList.add(yoGraphicPosition);
         }

         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      }
   }

   private void registerArtifacts(String name, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (yoGraphicsListRegistry != null)
      {
         artifactList = new ArtifactList(name + "Balls");

         for (YoGraphicVector yoGraphicPosition : yoGraphicVectors)
         {
            artifactList.add(yoGraphicPosition.createArtifact());
         }

         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }
   }

   /**
    * Sets the next vector to the given location and velocity. If all the balls have been set, then does nothing.
    *
    * @param location FramePoint to set the next ball to.
    */
   public void setVector(FramePoint3DReadOnly location, FrameVector3DReadOnly velocity)
   {
      setVector(location, velocity, index);
      index++;
   }

   /**
    * Sets the next ball to the given location, and gives it the given appearance. If all the balls
    * have been set, then does nothing.
    *
    * @param location   FramePoint to set the next ball to.
    */
   public void setVector(FramePoint3DReadOnly location, FrameVector3DReadOnly velocity, int ballIndex)
   {
      //TODO: PDN, note that with current implementation of JME, you can only "set" the appearance once. After that, it will ignore all appearance sets
      if (!location.getReferenceFrame().isWorldFrame())
         throw new RuntimeException(location + " must be in a World Frame!");

      setVector(location.getX(), location.getY(), location.getZ(), velocity.getX(), velocity.getY(), velocity.getZ(), ballIndex);
   }

   public void setVector(double pointX, double pointY, double pointZ, double velocityX, double velocityY, double velocityZ)
   {
      setVector(pointX, pointY, pointZ, velocityX, velocityY, velocityZ, index);
      index++;
   }

   public void setVector(double pointX, double pointY, double pointZ, double velocityX, double velocityY, double velocityZ, int ballIndex)
   {
      if (ballIndex < yoGraphicVectors.size())
      {
         YoGraphicVector yoGraphicPosition = yoGraphicVectors.get(ballIndex);
         yoGraphicPosition.set(pointX, pointY, pointZ, velocityX, velocityY, velocityZ);
      }
      else
      {
         if (!outOfVectorsWarning)
         {
            outOfVectorsWarning = true;
         }
      }
   }

   /**
    * Sets the next ball to the given location. If all the balls have been set, then loops to the first
    * ball.
    *
    * @param location FramePoint to set the next ball to.
    */
   public void setVectorLoop(FramePoint3DReadOnly location, FrameVector3DReadOnly velocity)
   {
      location.checkReferenceFrameMatch(worldFrame);
      velocity.checkReferenceFrameMatch(worldFrame);

      if (index >= yoGraphicVectors.size())
      {
         index = 0;
      }

      YoGraphicVector yoGraphicVector = yoGraphicVectors.get(index);
      yoGraphicVector.set(location, velocity);

      index++;
   }

   /**
    * Resets by placing all the balls at (0, 0, 0) and making the first ball be the next ball to place.
    */
   public void reset()
   {
      index = 0;

      for (int i = 0; i < yoGraphicVectors.size(); i++)
      {
         YoGraphicVector yoGraphicVector = yoGraphicVectors.get(i);
         yoGraphicVector.set(Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
      }
   }

   /**
    * Hides all the balls.
    */
   public void hideAll()
   {
      index = 0;
      for (int i = 0; i < yoGraphicVectors.size(); i++)
      {
         yoGraphicVectors.get(i).hide();
      }
   }

   public void setVisible(boolean visible)
   {
      index = 0;
      yoGraphicsList.setVisible(visible);
   }

   public int getNumberOfVectors()
   {
      return yoGraphicVectors.size();
   }
}
