package us.ihmc.atlas.parameters;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.SettableFootstepPlannerParameters;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class AtlasFootstepPlannerParameters implements FootstepPlannerParameters
{
   private static volatile boolean printed = false;

   private double cliffClearance = 0.1;
   private double cliffHeight = 0.05;
   private double maxStepLength = 0.35;
   private double maxStepWidth = 0.4;
   private double maxStepYaw = 0.4;
   private double maxStepZ = 0.25;
   private double maxXYWiggle = 0.03;
   private double maxYawWiggle = 0.17;
   private double minFootholdPercent = 0.99;
   private double minStepLength = -0.4;
   private double minStepWidth = 0.15;
   private double minStepYaw = 0.0;
   private double minSurfaceIncline = 0.7853981633974483; // unused?
   private double minXClearance = 0.18;
   private double minYClearance = 0.2;
   private double wiggleInsideDelta = 0.02;
   private double idealFootstepWidth = 0.22; // unused?
   private double idealFootstepLength = 0.3; // unused?
   private boolean wiggleIntoConvexHull = true;
   private boolean rejectIfCannotFullyWiggleInside = false;

   public AtlasFootstepPlannerParameters()
   {
      Path parametersPath = Paths.get(SettableFootstepPlannerParameters.CONFIGURATION_FILE_NAME).toAbsolutePath().normalize();

      if (Files.exists(parametersPath))
      {
         File configurationFile = parametersPath.toFile();
         FilePropertyHelper filePropertyHelper = new FilePropertyHelper(configurationFile);

         maxStepLength = filePropertyHelper.loadDoubleProperty("maxStepLength", maxStepLength);
         maxStepWidth = filePropertyHelper.loadDoubleProperty("maxStepWidth", maxStepWidth);
         minStepWidth = filePropertyHelper.loadDoubleProperty("minStepWidth", minStepWidth);
         minStepLength = filePropertyHelper.loadDoubleProperty("minStepLength", minStepLength);
         maxStepZ = filePropertyHelper.loadDoubleProperty("maxStepZ", maxStepZ);
         minSurfaceIncline = filePropertyHelper.loadDoubleProperty("minSurfaceIncline", minSurfaceIncline);
         maxStepYaw = filePropertyHelper.loadDoubleProperty("maxStepYaw", maxStepYaw);
         minStepYaw = filePropertyHelper.loadDoubleProperty("minStepYaw", minStepYaw);
         minFootholdPercent = filePropertyHelper.loadDoubleProperty("minFootholdPercent", minFootholdPercent);
         minXClearance = filePropertyHelper.loadDoubleProperty("minXClearance", minXClearance);
         minYClearance = filePropertyHelper.loadDoubleProperty("minYClearance", minYClearance);
         cliffHeight = filePropertyHelper.loadDoubleProperty("cliffHeightSpinner", cliffHeight);
         cliffClearance = filePropertyHelper.loadDoubleProperty("cliffClearance", cliffClearance);
         maxXYWiggle = filePropertyHelper.loadDoubleProperty("maxXYWiggleSpinner", maxXYWiggle);
         maxYawWiggle = filePropertyHelper.loadDoubleProperty("maxYawWiggleSpinner", maxYawWiggle);
         wiggleInsideDelta = filePropertyHelper.loadDoubleProperty("wiggleInsideDeltaSpinner", wiggleInsideDelta);

         if (!printed)
         {
            printed = true;
            LogTools.info("Loaded footstep planner parameters from {}", parametersPath);
         }
      }
      else
      {
         if (!printed)
         {
            printed = true;
            LogTools.warn("Using defaults: Could not load parameters from {}", parametersPath);
         }
      }
   }

   @Override
   public double getIdealFootstepWidth()
   {
      return idealFootstepWidth;
   }

   @Override
   public double getIdealFootstepLength()
   {
      return idealFootstepLength;
   }

   @Override
   public double getMaximumStepReach()
   {
      return maxStepLength;
   }

   @Override
   public double getMaximumStepYaw()
   {
      return maxStepYaw;
   }

   @Override
   public double getMinimumStepWidth()
   {
      return minStepWidth;
   }

   @Override
   public double getMaximumStepZ()
   {
      return maxStepZ;
   }

   @Override
   public double getMaximumStepWidth()
   {
      return maxStepWidth;
   }

   @Override
   public double getCliffHeightToAvoid()
   {
      return cliffHeight;
   }

   @Override
   public double getMinimumDistanceFromCliffBottoms()
   {
      return cliffClearance;
   }

   @Override
   public double getWiggleInsideDelta()
   {
      return wiggleInsideDelta;
   }

   @Override
   public boolean getWiggleIntoConvexHullOfPlanarRegions()
   {
      return wiggleIntoConvexHull;
   }

   @Override
   public boolean getRejectIfCannotFullyWiggleInside()
   {
      return rejectIfCannotFullyWiggleInside;
   }

   @Override
   public double getMaximumXYWiggleDistance()
   {
      return maxXYWiggle;
   }

   @Override
   public double getMaximumYawWiggle()
   {
      return maxYawWiggle;
   }
}
