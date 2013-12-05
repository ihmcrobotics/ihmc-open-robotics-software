package us.ihmc.darpaRoboticsChallenge.calib;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import javax.vecmath.Quat4d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class CalibUtil
{
   public static Quat4d quat0 = new Quat4d(0,0,0,1);
   public static ReferenceFrame world = ReferenceFrame.getWorldFrame();
   public static Map<String, Double>  addQ(Map<String, Double> q1, Map<String, Double> q2)
   {
      Map<String, Double> qbuffer = new HashMap<>();
      addQ(q1,q2,qbuffer);
      return qbuffer;
   }
   
   
   public static void addQ(Map<String, Double> q1, Map<String, Double> q2, Map<String, Double> qout)
   {
      
      assert(q1.size() == q2.size());
      for(String jointName: q1.keySet())
      {
   //      System.out.println("addQ:"+jointName);
         double v=0;
         if (q1.containsKey(jointName))
            v+=q1.get(jointName);
         if(q2.containsKey(jointName))
            v+=q2.get(jointName);         
         qout.put(jointName, new Double(v));
      }
   }
   
   public static ArrayList<String> toStringArrayList(ArrayList<OneDoFJoint> joints)
   {
      ArrayList<String> jointNames = new ArrayList<>();
      for(OneDoFJoint joint: joints)
         jointNames.add(joint.getName());
      return jointNames;
   }
   
   public static void setRobotModelFromData(SDFFullRobotModel fullRobotModel, Map<String, Double> qmap)
   {      
      OneDoFJoint[] joints = fullRobotModel.getOneDoFJoints();
      
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         if (qmap.containsKey(joint.getName()))
         {
            joint.setQ(qmap.get(joint.getName()));
         }
         else if (AtlasKinematicCalibrator.DEBUG)
         {
            System.out.println("model contain joints not in data " + joint.getName());
            joint.setQ(0);
         }
      }

   }
}