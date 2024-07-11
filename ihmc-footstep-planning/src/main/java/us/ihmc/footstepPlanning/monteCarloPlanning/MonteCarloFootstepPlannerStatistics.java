package us.ihmc.footstepPlanning.monteCarloPlanning;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.WriteOption;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Date;

public class MonteCarloFootstepPlannerStatistics
{
   private File file;

   private String nodesPerLayerString;

   private int numberOfNodesVisited;
   private int numberOfNodesPruned;

   private double totalTime = 0;
   private double totalSimulationTime = 0;
   private double totalExpansionTime = 0;
   private double totalPruningTime = 0;
   private double totalPropagationTime = 0;
   private double totalSearchTime = 0;

   private double lastSearchTime = 0;
   private double lastSimulationTime = 0;
   private double lastExpansionTime = 0;
   private double lastPruningTime = 0;
   private double lastPropagationTime = 0;

   private long totalStartTime = 0;
   private long simulationStartTime = 0;
   private long expansionStartTime = 0;
   private long pruningStartTime = 0;
   private long propagationStartTime = 0;
   private long searchStartTime = 0;

   private double totalReward = 0;
   private double contactReward = 0;
   private double goalReward = 0;

   private boolean totalTimeStarted = false;
   private boolean simulationTimeStarted = false;
   private boolean expansionTimeStarted = false;
   private boolean pruningTimeStarted = false;
   private boolean propagationTimeStarted = false;
   private boolean searchTimeStarted = false;

   public MonteCarloFootstepPlannerStatistics()
   {
      SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
      String logFileName = dateFormat.format(new Date()) + "_" + "MonteCarloPlannerLog.txt";
      FileTools.ensureDirectoryExists(Paths.get(IHMCCommonPaths.MONTE_CARLO_FOOTSTEP_PLANNER_DIRECTORY_NAME), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      String filePath = IHMCCommonPaths.MONTE_CARLO_FOOTSTEP_PLANNER_DIRECTORY.resolve(logFileName).toString();

      try
      {
         if(!Files.exists(IHMCCommonPaths.MONTE_CARLO_FOOTSTEP_PLANNER_DIRECTORY))
         {
            Files.createDirectory(IHMCCommonPaths.MONTE_CARLO_FOOTSTEP_PLANNER_DIRECTORY);
         }
         if (!Files.exists(IHMCCommonPaths.MONTE_CARLO_FOOTSTEP_PLANNER_DIRECTORY.resolve(logFileName)))
         {
            Files.createFile(Paths.get(filePath));
            file = new File(filePath);
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public void logToFile(boolean logToFile, boolean printToConsole)
   {
      if (logToFile || printToConsole)
      {
         StringBuilder log = new StringBuilder();

         log.append("{");
         log.append("TotalTime: ").append(totalTime).append(", ");
         log.append("SimulationTime: ").append(lastSimulationTime).append(", ");
         log.append("ExpansionTime: ").append(lastExpansionTime).append(", ");
         log.append("PruningTime: ").append(lastPruningTime).append(", ");
         log.append("PropagationTime: ").append(lastPropagationTime).append(", ");
         log.append("SearchTime: ").append(lastSearchTime).append(", ");
         log.append("Visited: ").append(numberOfNodesVisited).append(", ");
         log.append("Pruned: ").append(numberOfNodesPruned).append(", ");
         log.append("TotalReward: ").append(totalReward).append(", ");
         log.append("ContactReward: ").append(contactReward).append(", ");
         log.append("GoalReward: ").append(goalReward).append(", ");
         log.append("NodesPerLayer: ").append(nodesPerLayerString).append(", ");
         log.append("}\n");

         if (printToConsole)
            System.out.println(log);

         if (logToFile)
            FileTools.write(file.getAbsoluteFile().toPath(), log.toString().getBytes(), WriteOption.APPEND, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      }
   }

   public void startTotalTime()
   {
      if (!totalTimeStarted)
      {
         totalStartTime = System.nanoTime();
         totalTimeStarted = true;
      }
   }

   public void stopTotalTime()
   {
      if (totalTimeStarted)
      {
         totalTime = (double) (System.nanoTime() - totalStartTime) / 1000000;
         totalTimeStarted = false;
      }
   }

   public void startSimulationTime()
   {
      if (!simulationTimeStarted)
      {
         simulationStartTime = System.nanoTime();
         simulationTimeStarted = true;
      }
   }

   public void stopSimulationTime()
   {
      if (simulationTimeStarted)
      {
         lastSimulationTime = (double) (System.nanoTime() - simulationStartTime) / 1000000;
         totalSimulationTime += lastSimulationTime;
         simulationTimeStarted = false;
      }
   }

   public void startExpansionTime()
   {
      if (!expansionTimeStarted)
      {
         expansionStartTime = System.nanoTime();
         expansionTimeStarted = true;
      }
   }

   public void stopExpansionTime()
   {
      if (expansionTimeStarted)
      {
         lastExpansionTime = (double) (System.nanoTime() - expansionStartTime) / 1000000;
         totalExpansionTime += lastExpansionTime;
         expansionTimeStarted = false;
      }
   }

   public void startPruningTime()
   {
      if (!pruningTimeStarted)
      {
         pruningStartTime = System.nanoTime();
         pruningTimeStarted = true;
      }
   }

   public void stopPruningTime()
   {
      if (pruningTimeStarted)
      {
         lastPruningTime = (double) (System.nanoTime() - pruningStartTime) / 1000000;
         totalPruningTime += lastPruningTime;
         pruningTimeStarted = false;
      }
   }

   public void startPropagationTime()
   {
      if (!propagationTimeStarted)
      {
         propagationStartTime = System.nanoTime();
         propagationTimeStarted = true;
      }
   }

   public void stopPropagationTime()
   {
      if (propagationTimeStarted)
      {
         lastPropagationTime = (double) (System.nanoTime() - propagationStartTime) / 1000000;
         totalPropagationTime += lastPropagationTime;
         propagationTimeStarted = false;
      }
   }

   public void startSearchTime()
   {
      if (!searchTimeStarted)
      {
         searchStartTime = System.nanoTime();
         searchTimeStarted = true;
      }
   }

   public void stopSearchTime()
   {
      if (searchTimeStarted)
      {
         lastSearchTime = (double) (System.nanoTime() - searchStartTime) / 1000000;
         totalSearchTime += lastSearchTime;
         searchTimeStarted = false;
      }
   }

   public void setLayerCountsString(String nodesPerLayerString)
   {
      this.nodesPerLayerString = nodesPerLayerString;
   }

   public void setNumberOfNodesVisited(int numberOfNodesVisited)
   {
      this.numberOfNodesVisited = numberOfNodesVisited;
   }

   public void setNumberOfNodesPruned(int numberOfNodesPruned)
   {
      this.numberOfNodesPruned = numberOfNodesPruned;
   }

   public void setTotalReward(float totalReward)
   {
      this.totalReward = totalReward;
   }

   public void setContactReward(float contactReward)
   {
      this.contactReward = contactReward;
   }

   public void setGoalReward(float goalReward)
   {
      this.goalReward = goalReward;
   }
}
