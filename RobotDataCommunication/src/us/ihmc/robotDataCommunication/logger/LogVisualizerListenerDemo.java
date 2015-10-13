package us.ihmc.robotDataCommunication.logger;

import java.io.IOException;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFHumanoidRobot;
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
      System.out.println("}");
      
   }

}
