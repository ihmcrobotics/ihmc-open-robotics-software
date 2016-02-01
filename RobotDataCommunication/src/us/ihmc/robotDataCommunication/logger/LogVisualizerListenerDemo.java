package us.ihmc.robotDataCommunication.logger;

import java.io.IOException;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

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
   public void setRobot(SDFHumanoidRobot robot)
   {
      joints = robot.getOneDoFJoints();
      origin = robot.getRootJoint();
   }

   @Override
   public void updated(long timestamp)
   {
      System.out.print(timestamp + ": ");
      
      Point3d position = new Point3d();
      origin.getPosition(position);
      System.out.println("pos: " + position.x + " " + position.y + " " + position.z + " - {");
      for(OneDegreeOfFreedomJoint joint : joints)
      {
         System.out.print(joint.getQ().getDoubleValue() + ",");
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
