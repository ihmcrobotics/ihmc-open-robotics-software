package us.ihmc.simulationDispatcherExamples.gaSliderRobotTest;

import java.util.Random;

import us.ihmc.simulationconstructionset.Simulation;
import us.ihmc.simulationconstructionset.SimulationConstructor;
import us.ihmc.simulationconstructionset.simulationDispatcher.client.SimulationDispatcher;
import us.ihmc.utilities.parameterOptimization.OptimizationProblem;
import us.ihmc.utilities.parameterOptimization.geneticAlgorithm.GeneticAlgorithm;
import us.ihmc.utilities.parameterOptimization.geneticAlgorithm.PopulationParameters;

public class SliderGADispatcher
{
   public SliderGADispatcher()
   {
   }

   public static void main(String args[])
   {      
      // Need to put the jar file on the web, or something with a URL. 
      // Here is example for personal wiki page:
      // Put on the wiki at: http://ihmc.us/groups/jpratt/wiki/415b6/test.html
	     String myCodeBase = "http://ihmc.us/groups/jpratt/wiki/415b6/attachments/7835e/GASliderRobotDispatcher18.jar";
      
      // Here is example for SVN. Not sure if it works though since need to be logged in to svn
//      String myCodeBase = "https://bengal.ihmc.us/svn/RobotControl/SimulationDispatcher/GASliderRobotTest/GASliderRobotDispatcher13.jar";
      
      // Here is example for local file. But must have this file the same on all RemoteSimulationRunner computers.
//      String myCodeBase = "file:///S:/EclipseWorkspace/SimulationDispatcher/GASliderRobotTest/GASliderRobotDispatcher13.jar";
      
      // Here is example for getting it using resource locator. Note that the jar file must be created to be in the src directory, and then eclipse will
      // automatically copy it into the classes directory.
     
//      Class<SliderGADispatcher> myClass = SliderGADispatcher.class;
//      String myCodeBase = "file://" + myClass.getResource("GASliderRobotDispatcher15.jar").getFile();
      
      
      System.out.println("myCodeBase = " + myCodeBase);
	      
	      
      String[] hostNames = new String[]
      {
         "gazelle:2", 
         "cheetah:4",
//         "unknownpw-PC:8",
         "SylvainPC:2"
      };


      SimulationDispatcher dispatcher = new SimulationDispatcher(hostNames, myCodeBase);

      // Create Simulation
      SliderRobot sliderRobot = new SliderRobot(null, null);
      SliderController controller = new SliderController(sliderRobot);
      sliderRobot.setController(controller);
      Simulation sliderSim = new Simulation(sliderRobot, 1);

      SimulationConstructor sliderConstructor = new SliderSimulationConstructor();

      SliderIndividual sliderIndividual = new SliderIndividual(dispatcher, sliderSim, sliderConstructor);

      int populationSize = 50;
      double crossoverRate = 0.95;
      double mutationRate = 0.001;
      String name = "Slider";
      Random random = new Random(1789L);
      
      PopulationParameters populationParameters = new PopulationParameters(name, random, populationSize);
      GeneticAlgorithm geneticAlgorithm = new GeneticAlgorithm(populationParameters, crossoverRate, mutationRate);

      geneticAlgorithm.createGUI();

      double cutoffFitness = Double.POSITIVE_INFINITY;
      boolean maximize = true;
      
      int maximumNumberOfIndividualsToEvaluate = 1000;

      OptimizationProblem optimizationProblem = new OptimizationProblem(sliderIndividual, maximize, cutoffFitness, maximumNumberOfIndividualsToEvaluate);
      geneticAlgorithm.optimize(optimizationProblem);
      
//    geneticAlgorithm.save("ga/Slider.ga");
   }

}
