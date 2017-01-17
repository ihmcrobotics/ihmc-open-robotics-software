package us.ihmc.simulationconstructionset;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class JointAxisVisualizer implements RobotController
{
   private final  String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final List<YoGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<YoGraphicReferenceFrame>();
   
   public JointAxisVisualizer(RigidBody rootBody, YoGraphicsListRegistry yoGraphicsListRegistry, double length)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(name);
      List<InverseDynamicsJoint> jointStack = new ArrayList<InverseDynamicsJoint>(rootBody.getChildrenJoints());
      while (!jointStack.isEmpty())
      {
         InverseDynamicsJoint joint = jointStack.get(0);
         if(joint instanceof OneDoFJoint)
         {
            FrameVector jAxis=((OneDoFJoint)joint).getJointAxis();
            ReferenceFrame referenceFrame = ReferenceFrame.constructReferenceFrameFromPointAndZAxis(joint.getName()+"JointAxis", new FramePoint(jAxis.getReferenceFrame()), new FrameVector(jAxis.getReferenceFrame(),jAxis.getVector()));
            YoGraphicReferenceFrame dynamicGraphicReferenceFrame = new YoGraphicReferenceFrame(referenceFrame , registry, length, YoAppearance.Gold());
            yoGraphicsList.add(dynamicGraphicReferenceFrame);
            dynamicGraphicReferenceFrames.add(dynamicGraphicReferenceFrame);
         }
         List<InverseDynamicsJoint> childrenJoints = joint.getSuccessor().getChildrenJoints();
         jointStack.addAll(childrenJoints);
         jointStack.remove(joint);
      }
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
   }
   
   public void initialize()
   {
      doControl();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   public void doControl()
   {
      for (int i = 0; i < dynamicGraphicReferenceFrames.size(); i++)
      {
         dynamicGraphicReferenceFrames.get(i).update();
      } 
   }

}
