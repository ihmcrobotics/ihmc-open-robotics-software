package us.ihmc.robotDataVisualizer.logger;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

import javax.vecmath.Point3d;
import java.io.IOException;

public class LogVisualizerListenerDemo implements YoVariableLogPlaybackListener
{

   public static final void main(String args[]) throws IOException
   {
      LogVisualizer logVisualizer = new LogVisualizer();
      logVisualizer.addLogPlaybackListener(new LogVisualizerListenerDemo());
      logVisualizer.run();
   }

   private OneDegreeOfFreedomJoint[] joints;
   private FloatingJoint origin;
   private DoubleYoVariable desiredCoMHeight;

   @Override
   public void setRobot(FloatingRootJointRobot robot)
   {
      joints = robot.getOneDegreeOfFreedomJoints();
      origin = robot.getRootJoint();
   }

   @Override
   public void updated(long timestamp)
   {
      System.out.print(timestamp + ": ");

      Point3d position = new Point3d();
      origin.getPosition(position);
      System.out.println("pos: " + position.getX() + " " + position.getY() + " " + position.getZ() + " - {");
      for(OneDegreeOfFreedomJoint joint : joints)
      {
         System.out.print(joint.getQYoVariable().getDoubleValue() + ",");
      }
      System.out.println("}. height: " + desiredCoMHeight.getDoubleValue());

   }

   @Override
   public void setYoVariableRegistry(YoVariableRegistry registry)
   {
      // To get the full name of a variable, right click on a variable in the logger and select "Copy Full Name to Clipboard".
      // Remove the "root.loggedMain"
      //
      // You only have to give the last part of the namespace, more might break if we rename parts of the name.

      desiredCoMHeight = (DoubleYoVariable) registry.getVariable("LookAheadCoMHeightTrajectoryGenerator.desiredCoMHeight");
   }

}
