package us.ihmc.simulationConstructionSetTools.util.visualizers;

import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;

public class RobotFreezeFramer implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("RobotFreezeFramer");
   private final YoBoolean doFreezeFrame = new YoBoolean("doFreezeFrame", registry);
   private final YoDouble freezeInterval = new YoDouble("freezeInterval", registry);
   private final YoDouble nextFreezeTime = new YoDouble("nextFreezeTime", registry);

   private final Robot robot;
   private final GraphicsRobot graphicsRobot;
   private final Graphics3DAdapter graphics3dAdapter;
   private final Graphics3DNode rootJoint;

   public RobotFreezeFramer(Robot robot, SimulationConstructionSet scs)
   {
      this(robot, null, scs);
   }

   public RobotFreezeFramer(Robot robot, Joint rootJoint, SimulationConstructionSet scs)
   {
      this.robot = robot;
      this.graphicsRobot = scs.getStandardSimulationGUI().getGraphicsRobot(robot);
      this.graphics3dAdapter = scs.getGraphics3dAdapter();
      if(rootJoint != null)
      {
         this.rootJoint = graphicsRobot.getGraphicsJoint(rootJoint);
      }
      else
      {
         this.rootJoint = graphicsRobot.getRootNode();
      }
      
      doFreezeFrame.set(false);
      freezeInterval.set(1.0);
      nextFreezeTime.set(1.0);
   }
   
   public void setFreezeInterval(double freezeInterval)
   {
      this.freezeInterval.set(freezeInterval);
   }
   
   public void setNextFreezeTime(double nextFreezeTime)
   {
      this.nextFreezeTime.set(nextFreezeTime);
   }
   
   public void setDoFreezeFrame(boolean doFreezeFrame)
   {
      this.doFreezeFrame.set(doFreezeFrame);
   }

   @Override
   public void doControl()
   {
      if (doFreezeFrame.getBooleanValue() && (robot.getTime() > nextFreezeTime.getDoubleValue()))
      {
         nextFreezeTime.set(robot.getTime() + freezeInterval.getDoubleValue());
         graphics3dAdapter.freezeFrame(rootJoint);

      }
   }



   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return "RobotFreezeFramer";
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
