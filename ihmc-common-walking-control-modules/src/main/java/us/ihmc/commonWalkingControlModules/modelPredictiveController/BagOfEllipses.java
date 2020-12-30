package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

/**
 * <p>
 * Title: BagOfVectors
 * </p>
 * <p>
 * Description: Class for displaying multiple vectors in the SCS GUI.
 * </p>
 */
public class BagOfEllipses
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ArrayList<YoGraphicEllipsoid> yoGraphicEllipsoids = new ArrayList<>();
   private int index;
   private boolean outOfVectorsWarning = false;
   private YoGraphicsList yoGraphicsList;

   /**
    * Creates a BagOfVectors with the given number of balls, and all the balls with the given Appearance.
    *
    * @param numberOfVectors int Number of balls to create.
    * @param vectorSizeInMeters double Size of each ball in meters.
    * @param name String Name of the BagOfVectors.
    * @param appearance Appearance for each of the balls.
    * @param parentRegistry YoRegistry to register the BagOfVectors with.
    * @param yoGraphicsListRegistry YoGraphicsListRegistry to register the BagOfVectors with.
    */
   public BagOfEllipses(int numberOfVectors,
                        String name,
                        AppearanceDefinition appearance,
                        YoRegistry parentRegistry,
                        YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(name, toList(appearance, numberOfVectors), parentRegistry, yoGraphicsListRegistry);
   }

   /**
    * Creates a BagOfVectors with the size being the same as the number of Appearances given.
    *
    * @param name String Name of the BagOfVectors
    * @param appearances ArrayList of the Appearance for each of the balls.
    * @param parentRegistry YoRegistry to register the BagOfVectors with.
    * @param yoGraphicsListRegistry YoGraphicsListRegistry to register the BagOfVectors with.
    */
   public BagOfEllipses(String name, List<AppearanceDefinition> appearances, YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoRegistry registry = new YoRegistry(name + "Vectors");

      for (int i = 0; i < appearances.size(); i++)
      {
         YoFramePoint3D yoFramePoint = new YoFramePoint3D(name + i, "Point", worldFrame, registry);
         YoFrameYawPitchRoll yoFrameOrientation = new YoFrameYawPitchRoll(name + i, "Orientation", worldFrame, registry);

         YoGraphicEllipsoid newVector = new YoGraphicEllipsoid(name + i, yoFramePoint, yoFrameOrientation, appearances.get(i), new Vector3D());

         yoGraphicEllipsoids.add(newVector);
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

   private void registerYoGraphics(String name, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (yoGraphicsListRegistry != null)
      {
         yoGraphicsList = new YoGraphicsList(name + "Balls");

         for (YoGraphicEllipsoid yoGraphicEllipsoid : yoGraphicEllipsoids)
            yoGraphicsList.add(yoGraphicEllipsoid);

         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      }
   }

   public void setEllipse(FramePoint3DReadOnly location, FrameQuaternionReadOnly orientation, Vector3D radii)
   {
      setEllipse(location, orientation, radii, index);
      index++;
   }

   public void setEllipse(FramePoint3DReadOnly location, FrameQuaternionReadOnly orientation, Vector3D radii, int ballIndex)
   {
      if (!location.getReferenceFrame().isWorldFrame())
         throw new RuntimeException(location + " must be in a World Frame!");

      if (ballIndex < yoGraphicEllipsoids.size())
      {
         YoGraphicEllipsoid yoGraphicVector = yoGraphicEllipsoids.get(ballIndex);
         yoGraphicVector.setPosition(location);
         yoGraphicVector.setOrientation(orientation);
         yoGraphicVector.setRadii(radii);
      }
      else
      {
         if (!outOfVectorsWarning)
         {
            outOfVectorsWarning = true;
         }
      }
   }

   public void setEllipseLoop(FramePoint3DReadOnly location, FrameQuaternionReadOnly orientation, Vector3D radii)
   {
      location.checkReferenceFrameMatch(worldFrame);
      orientation.checkReferenceFrameMatch(worldFrame);

      if (index >= yoGraphicEllipsoids.size())
      {
         index = 0;
      }

      YoGraphicEllipsoid yoGraphicEllipsoid = yoGraphicEllipsoids.get(index);
      yoGraphicEllipsoid.setPosition(location);
      yoGraphicEllipsoid.setOrientation(orientation);
      yoGraphicEllipsoid.setRadii(radii);

      index++;
   }

   /**
    * Resets by placing all the balls at (0, 0, 0) and making the first ball be the next ball to place.
    */
   public void reset()
   {
      index = 0;

      for (int i = 0; i < yoGraphicEllipsoids.size(); i++)
      {
         YoGraphicEllipsoid yoGraphicVector = yoGraphicEllipsoids.get(i);
         yoGraphicVector.setPosition(Double.NaN, Double.NaN, Double.NaN);
         yoGraphicVector.setYawPitchRoll(Double.NaN, Double.NaN, Double.NaN);
      }
   }

   /**
    * Hides all the balls.
    */
   public void hideAll()
   {
      index = 0;
      for (int i = 0; i < yoGraphicEllipsoids.size(); i++)
      {
         yoGraphicEllipsoids.get(i).hide();
      }
   }

   public void setVisible(boolean visible)
   {
      index = 0;
      yoGraphicsList.setVisible(visible);
   }

   public int getNumberOfVectors()
   {
      return yoGraphicEllipsoids.size();
   }
}
