package us.ihmc.SdfLoader;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;

import javax.media.j3d.Transform3D;

import us.ihmc.commonWalkingControlModules.referenceFrames.ReferenceFrames;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.Twist;

import com.yobotics.simulationconstructionset.PinJoint;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RawOutputWriter;
import com.yobotics.simulationconstructionset.robotController.RawSensorReader;

public class SDFPerfectSimulatedSensorReaderAndWriter implements RawSensorReader, RawOutputWriter
{
   private static final long serialVersionUID = 7667765732502448400L;
   private final String name;   
   private final SDFRobot robot;
   private final SDFFullRobotModel sdfFullRobotModel;
   private final ReferenceFrames referenceFrames;
   
   private final ArrayList<Pair<PinJoint, RevoluteJoint>> revoluteJoints = new ArrayList<Pair<PinJoint, RevoluteJoint>>();
   
   public SDFPerfectSimulatedSensorReaderAndWriter(SDFRobot robot, SDFFullRobotModel sdfFullRobotModel, ReferenceFrames referenceFrames)
   {
      this.name = robot.getName() + "SimulatedSensorReader";
      this.robot = robot;
      this.sdfFullRobotModel = sdfFullRobotModel;
      this.referenceFrames = referenceFrames;
      
      HashMap<String, RevoluteJoint> joints = sdfFullRobotModel.getRevoluteJointsMap();
      
      for(Entry<String, RevoluteJoint> jointEntry : joints.entrySet())
      {
         String name = jointEntry.getKey();
         RevoluteJoint revoluteJoint = jointEntry.getValue();
         PinJoint pinJoint = robot.getJoint(name);
         
         Pair<PinJoint, RevoluteJoint> jointPair = new Pair<PinJoint, RevoluteJoint>(pinJoint, revoluteJoint);
         revoluteJoints.add(jointPair);
      }
      
   }

   public void initialize()
   {
      read();      
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   private Transform3D temporaryRootToWorldTransform = new Transform3D();
   public void read()
   {
    
      for(Pair<PinJoint, RevoluteJoint> jointPair : revoluteJoints)
      {
         PinJoint pinJoint = jointPair.first();
         RevoluteJoint revoluteJoint = jointPair.second();
         
         revoluteJoint.setQ(pinJoint.getQ().getDoubleValue());
         revoluteJoint.setQd(pinJoint.getQD().getDoubleValue());
         revoluteJoint.setQdd(pinJoint.getQDD().getDoubleValue());
         
      }
      
      SixDoFJoint rootJoint = sdfFullRobotModel.getRootJoint();
      robot.getRootJointToWorldTransform(temporaryRootToWorldTransform);
      rootJoint.setPositionAndRotation(temporaryRootToWorldTransform);
      
      referenceFrames.updateFrames();
      
      
      ReferenceFrame elevatorFrame = rootJoint.getFrameBeforeJoint();
      ReferenceFrame pelvisFrame = rootJoint.getFrameAfterJoint();

      FrameVector linearVelocity = robot.getRootJointVelocity();
      linearVelocity.changeFrame(pelvisFrame);

      FrameVector angularVelocity = robot.getPelvisAngularVelocityInPelvisFrame(pelvisFrame);
      angularVelocity.changeFrame(pelvisFrame);

      Twist bodyTwist = new Twist(pelvisFrame, elevatorFrame, pelvisFrame, linearVelocity.getVector(), angularVelocity.getVector());
      rootJoint.setJointTwist(bodyTwist);
      
      // Think about adding root body acceleration to the fullrobotmodel
      
   }

   public void write()
   {
      for(Pair<PinJoint, RevoluteJoint> jointPair : revoluteJoints)
      {
         PinJoint pinJoint = jointPair.first();
         RevoluteJoint revoluteJoint = jointPair.second();
         
         pinJoint.setTau(revoluteJoint.getTau());
      }
   }

}
