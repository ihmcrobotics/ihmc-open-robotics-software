package us.ihmc.simulationconstructionset;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class SimulationConstructionSetProcessDataCallTest
{   
   private static final boolean DEBUG = false;
   private YoVariableRegistry registry;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=1000)
   public void testForwardCount()
   {
      Robot robot = new Robot("testRobot");
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(); 
      parameters.setCreateGUI(false);
      parameters.setDataBufferSize(8192);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);

      registry = new YoVariableRegistry("testRegustry");
      DoubleYoVariable dataSet = new DoubleYoVariable("dataSet", registry);
      scs.addYoVariableRegistry(registry);
      
      int startValue = 5;
      int maxValue = 15;
      //Generate and populate data
      for(int i = startValue; i < maxValue; i++)
      {
         dataSet.set(i);
         scs.tickAndUpdate();
      }
 
      //Crop data to get rid of the first data point
      scs.setOutPoint();
      scs.setIndex(1);
      scs.setInPoint();
      scs.cropBuffer();
      
      //Apply the Data Processing Class
      DataProcessingFunction counterProcessingFunction = new CounterProcessingFunction(registry);
      scs.applyDataProcessingFunction(counterProcessingFunction);
      
      //Exact data from Data Processing Class
      DoubleYoVariable counterVariable = ((CounterProcessingFunction) counterProcessingFunction).getCountVariable();

      if (DEBUG)
      {
         System.out.println("===testForwardCount===");
         for (int i = 0; i < maxValue-startValue; i++)
         {
            scs.setIndex(i);
            System.out.println(dataSet.getDoubleValue() + "\t" + counterVariable.getDoubleValue());
         }     
      }

      //Test
      double testNum1, testNum2;
      for (int i = 0; i < maxValue-startValue; i++)
      {
         scs.setIndex(i);
         testNum1 = dataSet.getDoubleValue();
         testNum2 = counterVariable.getDoubleValue();
         
         assertEquals(testNum1-testNum2, startValue, 0);
      }    
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=1000)
   public void testBackwardCount()
   {
      Robot robot = new Robot("testRobot");
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(); 
      parameters.setCreateGUI(false);
      parameters.setDataBufferSize(8192);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);

      registry = new YoVariableRegistry("testRegustry");
      DoubleYoVariable dataSet = new DoubleYoVariable("dataSet", registry);
      scs.addYoVariableRegistry(registry);
      
      int startValue = 5;
      int maxValue = 15;
      //Generate and populate data
      for(int i = startValue; i < maxValue; i++)
      {
         dataSet.set(i);
         scs.tickAndUpdate();
      }
 
      //Crop data to get rid of the first data point
      scs.setOutPoint();
      scs.setIndex(1);
      scs.setInPoint();
      scs.cropBuffer();
      
      //Apply the Data Processing Class
      DataProcessingFunction counterProcessingFunction = new CounterProcessingFunction(registry);
      scs.applyDataProcessingFunctionBackward(counterProcessingFunction);
      
      //Exact data from Data Processing Class
      DoubleYoVariable counterVariable = ((CounterProcessingFunction) counterProcessingFunction).getCountVariable();

      if (DEBUG)
      {
         System.out.println("===testBackwardCount===");

         for (int i = 0; i < maxValue-startValue; i++)
         {
            scs.setIndex(i);
            System.out.println(dataSet.getDoubleValue() + "\t" + counterVariable.getDoubleValue());
         }
      }

      //Test
      double testNum1, testNum2;
      for (int i = 0; i < maxValue-startValue; i++)
      {
         scs.setIndex(i);
         testNum1 = dataSet.getDoubleValue();
         testNum2 = counterVariable.getDoubleValue();
         
         assertEquals(testNum1+testNum2, maxValue-1, 0);
      }    
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=1000)
   public void testForwardCopy()
   {
      Robot robot = new Robot("testRobot");
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(); 
      parameters.setCreateGUI(false);
      parameters.setDataBufferSize(8192);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      
      registry = new YoVariableRegistry("testRegustry");
      DoubleYoVariable dataSet = new DoubleYoVariable("dataSet", registry);
      scs.addYoVariableRegistry(registry);
      
      int startValue = 5;
      int maxValue = 15;
      //Generate and populate data
      for(int i = startValue; i < maxValue; i++)
      {
         dataSet.set(i);
         scs.tickAndUpdate();
      }
 
      //Crop data to get rid of the first data point
      scs.setOutPoint();
      scs.setIndex(1);
      scs.setInPoint();
      scs.cropBuffer();
      
      //Apply the Data Processing Class
      DataProcessingFunction copierProcessingFunction = new CopierProcessingFunction(dataSet, registry);
      scs.applyDataProcessingFunction(copierProcessingFunction);

      //Exact data from Data Processing Class
      DoubleYoVariable copierVariable = ((CopierProcessingFunction) copierProcessingFunction).getCopyVariable();

      if (DEBUG)
      {
         System.out.println("===testForwardCopy===");
         for (int i = 0; i < maxValue-startValue; i++)
         {
            scs.setIndex(i);
            System.out.println(dataSet.getDoubleValue() + "\t" + copierVariable.getDoubleValue());
         }
      }

      //Test
      double testNum1, testNum2;
      for (int i = 0; i < maxValue-startValue; i++)
      {
         scs.setIndex(i);
         testNum1 = dataSet.getDoubleValue();
         testNum2 = copierVariable.getDoubleValue();
         
         assertEquals(testNum1-testNum2, 0, 0);
      }    
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=1000)
   public void testBackwardCopy()
   {
      Robot robot = new Robot("testRobot");
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(); 
      parameters.setCreateGUI(false);
      parameters.setDataBufferSize(8192);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      
      registry = new YoVariableRegistry("testRegustry");
      DoubleYoVariable dataSet = new DoubleYoVariable("dataSet", registry);
      scs.addYoVariableRegistry(registry);
      
      int startValue = 5;
      int maxValue = 15;
      //Generate and populate data
      for(int i = startValue; i < maxValue; i++)
      {
         dataSet.set(i);
         scs.tickAndUpdate();
      }
 
      //Crop data to get rid of the first data point
      scs.setOutPoint();
      scs.setIndex(1);
      scs.setInPoint();
      scs.cropBuffer();
      
      //Apply the Data Processing Class
      DataProcessingFunction copierProcessingFunction = new CopierProcessingFunction(dataSet, registry);
      scs.applyDataProcessingFunctionBackward(copierProcessingFunction);
      
      //Exact data from Data Processing Class
      DoubleYoVariable copierVariable = ((CopierProcessingFunction) copierProcessingFunction).getCopyVariable();
      
      if (DEBUG)
      {
         System.out.println("===testBackwardCopy===");

         for (int i = 0; i < maxValue-startValue; i++)
         {
            scs.setIndex(i);
            System.out.println(dataSet.getDoubleValue() + "\t" + copierVariable.getDoubleValue());
         }
      }

      //Test     
      double testNum1, testNum2;
      for (int i = 0; i < maxValue-startValue; i++)
      {
         scs.setIndex(i);
         testNum1 = dataSet.getDoubleValue();
         testNum2 = copierVariable.getDoubleValue();
         
         assertEquals(testNum1-testNum2, 0, 0);
      }    

   }
   
   public static class CopierProcessingFunction implements DataProcessingFunction
   {
      private final DoubleYoVariable copyVariable;
      private final DoubleYoVariable testVariable;
      
      public CopierProcessingFunction(DoubleYoVariable inputData, YoVariableRegistry registry)
      {
         testVariable = inputData;
         copyVariable = new DoubleYoVariable("copyVariable", registry);
      }
      
      @Override
      public void processData()
      {
         double holderDouble;
         
         holderDouble = testVariable.getDoubleValue();
         copyVariable.set(holderDouble);
      }
      
      public DoubleYoVariable getCopyVariable()
      {
         return copyVariable;
      }

      @Override
      public void initializeProcessing()
      {
         // TODO Auto-generated method stub
         
      }
   }
   
   public static class CounterProcessingFunction implements DataProcessingFunction
   {
      private final DoubleYoVariable countVariable;
      private int count = 0;
      
      public CounterProcessingFunction(YoVariableRegistry registry)
      {
         countVariable = new DoubleYoVariable("countVariable", registry);
      }
      
      @Override
      public void processData()
      {
         countVariable.set(count);         
         count++;
      }
      
      public DoubleYoVariable getCountVariable()
      {
         return countVariable;
      }

      @Override
      public void initializeProcessing()
      {
         // TODO Auto-generated method stub
         
      }
   }   
}
