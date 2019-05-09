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

   private boolean wiggleIntoConvexHull = true;
   private boolean rejectIfCannotFullyWiggleInside = false;
   private boolean returnBestEffortPlan = false;
   private boolean checkForBodyBoxCollisions = false;
   private boolean performHeuristicSearchPolicies = true;
   private int minimumStepsForBestEffortPlan = 3;
   private double cliffClearance = 0.1;
   private double cliffHeight = 0.05;
   private double maxStepLength = 0.5;
   private double maxStepWidth = 0.4;
   private double maxStepYaw = 0.5;
   private double maxStepZ = 0.25;
   private double maxXYWiggle = 0.1;
   private double maxYawWiggle = 0.09;
   private double minFootholdPercent = 0.7;
   private double minStepLength = -0.6;
   private double minStepWidth = 0.0;
   private double minStepYaw = -0.5;
   private double minSurfaceIncline = 0.7853981633974483; // unused?
   private double minXClearance = 0.3;
   private double minYClearance = 0.22;
   private double wiggleInsideDelta = 0.02;
   private double stepUpHeight   = 1.5;
   private double stepDownHeight = 1.5;
   private double maxStepUpX     = 0.5;
   private double maxStepDownX   = 1.5;
   private double maxZPenetrationOnValleyRegions = Double.POSITIVE_INFINITY;
   private double idealFootstepWidth = 0.22; // unused?
   private double idealFootstepLength = 0.3; // unused?
   private double bodyGroundClearance = 0.25;
   private double bodyBoxWidth = 0.7;
   private double bodyBoxHeight = 1.5;
   private double bodyBoxDepth = 0.3;
   private double bodyBoxBaseX = 0.0;
   private double bodyBoxBaseY = 0.0;
   private double bodyBoxBaseZ = 0.25;
   private double finalTurnProximity = 1.0;

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

         printValues();

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
   public boolean checkForBodyBoxCollisions()
   {
      return checkForBodyBoxCollisions;
   }

   @Override
   public boolean performHeuristicSearchPolicies()
   {
      return performHeuristicSearchPolicies;
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
   public double getWiggleInsideDelta()
   {
      return wiggleInsideDelta;
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
   public double getMinimumStepLength()
   {
      return minStepLength;
   }

   @Override
   public double getMinimumStepYaw()
   {
      return minStepYaw;
   }

   @Override
   public double getMaximumStepReachWhenSteppingUp()
   {
      return maxStepUpX;
   }

   @Override
   public double getMaximumStepZWhenSteppingUp()
   {
      return stepUpHeight;
   }

   @Override
   public double getMaximumStepXWhenForwardAndDown()
   {
      return maxStepDownX;
   }

   @Override
   public double getMaximumStepZWhenForwardAndDown()
   {
      return stepDownHeight;
   }

   @Override
   public double getMaximumStepZ()
   {
      return maxStepZ;
   }

   @Override
   public double getMinimumFootholdPercent()
   {
      return minFootholdPercent;
   }

   @Override
   public double getMinimumSurfaceInclineRadians()
   {
      return minSurfaceIncline;
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

   @Override
   public double getMaximumZPenetrationOnValleyRegions()
   {
      return maxZPenetrationOnValleyRegions;
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
   public boolean getReturnBestEffortPlan()
   {
      return returnBestEffortPlan;
   }

   @Override
   public int getMinimumStepsForBestEffortPlan()
   {
      return minimumStepsForBestEffortPlan;
   }

   @Override
   public double getBodyGroundClearance()
   {
      return bodyGroundClearance;
   }

   @Override
   public double getBodyBoxHeight()
   {
      return bodyBoxHeight;
   }

   @Override
   public double getBodyBoxDepth()
   {
      return bodyBoxDepth;
   }

   @Override
   public double getBodyBoxWidth()
   {
      return bodyBoxWidth;
   }

   @Override
   public double getBodyBoxBaseX()
   {
      return bodyBoxBaseX;
   }

   @Override
   public double getBodyBoxBaseY()
   {
      return bodyBoxBaseY;
   }

   @Override
   public double getBodyBoxBaseZ()
   {
      return bodyBoxBaseZ;
   }

   @Override
   public double getMinXClearanceFromStance()
   {
      return minXClearance;
   }

   @Override
   public double getMinYClearanceFromStance()
   {
      return minYClearance;
   }

   @Override
   public double getFinalTurnProximity()
   {
      return finalTurnProximity;
   }
}
