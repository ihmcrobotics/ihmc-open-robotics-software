package us.ihmc.avatar.stepAdjustment;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class PlanarRegionsFilterForStepping
{
   private static final int regionsToViz = 0;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   /**
    * Minimum planar region area for a foothold
    */
   private final DoubleProvider minPlanarRegionArea = new DoubleParameter("minPlanarRegionAreaForStepping", registry, 0.05);

   /**
    * Maximum angle of planar region considered for a foothold.
    * Zero degrees corresponds to a vertical normal
    */
   private final DoubleProvider maxPlanarRegionAngle = new DoubleParameter("maxPlanarRegionAngleForStepping", registry, Math.toRadians(25.0));
   private final YoInteger totalPlanarRegions = new YoInteger("totalPlanarRegions", registry);
   private final YoInteger bigEnoughPlanarRegionsCounter = new YoInteger("bigEnoughPlanarRegionsCounter", registry);
   private final YoInteger filteredPlanarRegionsCounter = new YoInteger("filteredPlanarRegionsCounter", registry);
   private final YoInteger countsSinceReceivedPlanarRegions = new YoInteger("countsSinceReceivedPlanarRegions", registry);


   private final RecyclingArrayList<PlanarRegion> allPlanarRegionsList = new RecyclingArrayList<>(PlanarRegion::new);
   private final List<PlanarRegion> bigEnoughPlanarRegionsList = new ArrayList<>();
   private final List<PlanarRegion> filteredPlanarRegionsList = new ArrayList<>();

   private final List<YoFrameConvexPolygon2D> planarRegionHulls = new ArrayList<>();
   private final List<YoFramePose3D> planarRegionPoses = new ArrayList<>();

   private final FrameConvexPolygon2D tempPolygon = new FrameConvexPolygon2D();

   public PlanarRegionsFilterForStepping(YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      for (int i = 0; i < regionsToViz; i++)
      {
         YoFrameConvexPolygon2D planarRegionHull = new YoFrameConvexPolygon2D("planarRegionHull" + i, ReferenceFrame.getWorldFrame(), 20, registry);
         YoFramePose3D planarRegionPose = new YoFramePose3D("planarRegionPose" + i, ReferenceFrame.getWorldFrame(), registry);

         YoGraphicPolygon planarRegion = new YoGraphicPolygon("planarRegion" + i, planarRegionHull, planarRegionPose, 1.0, YoAppearance.Blue());

         planarRegionHulls.add(planarRegionHull);
         planarRegionPoses.add(planarRegionPose);

         graphicsListRegistry.registerYoGraphic("PlanarRegionsFilter", planarRegion);
      }
      parentRegistry.addChild(registry);
   }

   public void clear()
   {
      totalPlanarRegions.set(-1);
      bigEnoughPlanarRegionsCounter.set(-1);
      filteredPlanarRegionsCounter.set(-1);

      allPlanarRegionsList.clear();
      bigEnoughPlanarRegionsList.clear();
      filteredPlanarRegionsList.clear();
   }

   public double getMinPlanarRegionArea()
   {
      return minPlanarRegionArea.getValue();
   }

   public double getMaxPlanarRegionAngle()
   {
      return maxPlanarRegionAngle.getValue();
   }

   public void update()
   {
      countsSinceReceivedPlanarRegions.increment();
   }

   public void setPlanarRegions(PlanarRegionsListCommand planarRegions)
   {
      countsSinceReceivedPlanarRegions.set(-1);
      allPlanarRegionsList.clear();
      bigEnoughPlanarRegionsList.clear();
      filteredPlanarRegionsList.clear();

      for (int i = 0; i < planarRegions.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegionCommand candidateRegion = planarRegions.getPlanarRegionCommand(i);

         PlanarRegion planarRegion = allPlanarRegionsList.add();
         planarRegion.set(candidateRegion.getTransformToWorld(), candidateRegion.getConvexPolygons(), candidateRegion.getRegionId());

         if (planarRegion.getConvexHull().getArea() < getMinPlanarRegionArea())
            continue;

         bigEnoughPlanarRegionsList.add(planarRegion);

         if (Math.abs(planarRegion.getNormal().getZ()) < Math.cos(getMaxPlanarRegionAngle()))
            continue;

         filteredPlanarRegionsList.add(planarRegion);
      }

      totalPlanarRegions.set(allPlanarRegionsList.size());
      bigEnoughPlanarRegionsCounter.set(bigEnoughPlanarRegionsList.size());
      filteredPlanarRegionsCounter.set(filteredPlanarRegionsList.size());

      int i = 0;
      for (; i < Math.min(regionsToViz, bigEnoughPlanarRegionsCounter.getValue()); i++)
      {
         PlanarRegion region = bigEnoughPlanarRegionsList.get(i);
         tempPolygon.set(region.getConvexHull());
         tempPolygon.applyTransform(region.getTransformToWorld(), false);

         planarRegionHulls.get(i).set(tempPolygon);
         planarRegionPoses.get(i).getPosition().setToZero();
         planarRegionPoses.get(i).getOrientation().set(region.getTransformToWorld().getRotation());
      }

      for (; i < regionsToViz; i++)
      {
         planarRegionHulls.get(i).setToNaN();
         planarRegionPoses.get(i).setToNaN();
      }
   }

   public List<PlanarRegion> getPlanarRegions()
   {
      return allPlanarRegionsList;
   }

   public List<PlanarRegion> getBigEnoughPlanarRegions()
   {
      return bigEnoughPlanarRegionsList;
   }

   public List<PlanarRegion> getFilteredPlanarRegions()
   {
      return filteredPlanarRegionsList;
   }
}
