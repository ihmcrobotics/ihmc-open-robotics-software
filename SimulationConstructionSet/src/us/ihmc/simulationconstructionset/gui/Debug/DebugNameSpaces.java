package us.ihmc.simulationconstructionset.gui.Debug;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

/**
 * <p>Title: SimulationConstructionSet</p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2000</p>
 *
 * <p>Company: Yobotics, Inc.</p>
 *
 * @author not attributable
 * @version 1.0
 */
public class DebugNameSpaces
{
   public DebugNameSpaces()
   {
      DebugNameSpacesRobot robot = new DebugNameSpacesRobot();

      DebugNameSpacesController controller = new DebugNameSpacesController();
      robot.setController(controller);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);

      DoubleYoVariable variable1 = (DoubleYoVariable)scs.getVariable("variable1");

      Thread thread = new Thread(scs);
      thread.start();

      while (true)
      {
         System.out.println("variable1 = " + variable1.getDoubleValue());

         try
         {
            Thread.sleep(1000);
         }
         catch (InterruptedException ex)
         {
         }
      }
   }

   private class DebugNameSpacesRobot extends Robot
   {
      @SuppressWarnings("unused")
      private final DoubleYoVariable variable1;

//    private final YoVariable variable1_2 = new YoVariable("variable1", this);
      @SuppressWarnings("unused")
      private final DoubleYoVariable variable2;

      public DebugNameSpacesRobot()
      {
         super("DebugNameSpacesRobot");


         YoVariableRegistry registry = new YoVariableRegistry("DebugNameSpacesRobot");

         variable1 = new DoubleYoVariable("variable1", registry);
         variable2 = new DoubleYoVariable("variable2", registry);

         this.addYoVariableRegistry(registry);

      }
   }


   private class DebugNameSpacesController implements RobotController
   {
      private final YoVariableRegistry registry = new YoVariableRegistry("DebugNameSpacesController");
      @SuppressWarnings("unused")
      private final DoubleYoVariable variable1 = new DoubleYoVariable("variable1", registry);

//    private final YoVariable variable1_2 = new YoVariable("variable1", registry);
      @SuppressWarnings("unused")
      private final DoubleYoVariable variable2 = new DoubleYoVariable("variable2", registry);

      public DebugNameSpacesController()
      {
      }

      @Override
      public void doControl()
      {
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return "debugNameSpacesController";
      }
      
      @Override
      public void initialize()
      {      
      }

      @Override
      public String getDescription()
      {
         return getName();
      }

   }


   public static void main(String[] args)
   {
      @SuppressWarnings("unused")
      DebugNameSpaces debugNameSpaces = new DebugNameSpaces();
   }
}
