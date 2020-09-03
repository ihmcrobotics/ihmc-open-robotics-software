package us.ihmc.simulationConstructionSetTools.gui.Debug;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;

public class DebugNamespaces
{
   public DebugNamespaces()
   {
      DebugNamespacesRobot robot = new DebugNamespacesRobot();

      DebugNamespacesController controller = new DebugNamespacesController();
      robot.setController(controller);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);

      YoDouble variable1 = (YoDouble)scs.findVariable("variable1");

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

   private class DebugNamespacesRobot extends Robot
   {
      @SuppressWarnings("unused")
      private final YoDouble variable1;

//    private final YoVariable variable1_2 = new YoVariable("variable1", this);
      @SuppressWarnings("unused")
      private final YoDouble variable2;

      public DebugNamespacesRobot()
      {
         super("DebugNamespacesRobot");


         YoRegistry registry = new YoRegistry("DebugNamespacesRobot");

         variable1 = new YoDouble("variable1", registry);
         variable2 = new YoDouble("variable2", registry);

         this.addYoRegistry(registry);

      }
   }


   private class DebugNamespacesController implements RobotController
   {
      private final YoRegistry registry = new YoRegistry("DebugNamespacesController");
      @SuppressWarnings("unused")
      private final YoDouble variable1 = new YoDouble("variable1", registry);

//    private final YoVariable variable1_2 = new YoVariable("variable1", registry);
      @SuppressWarnings("unused")
      private final YoDouble variable2 = new YoDouble("variable2", registry);

      public DebugNamespacesController()
      {
      }

      @Override
      public void doControl()
      {
      }

      @Override
      public YoRegistry getYoRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return "debugNamespacesController";
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
      DebugNamespaces debugNamespaces = new DebugNamespaces();
   }
}
