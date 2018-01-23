package us.ihmc.simulationconstructionset;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class JointAxisVisualizer implements RobotController
{
   private final  String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final List<YoGraphicReferenceFrame> yoGraphicReferenceFrames = new ArrayList<YoGraphicReferenceFrame>();
   
   public JointAxisVisualizer(RigidBody rootBody, YoGraphicsListRegistry yoGraphicsListRegistry, double length)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(name);
      List<InverseDynamicsJoint> jointStack = new ArrayList<InverseDynamicsJoint>(rootBody.getChildrenJoints());
      while (!jointStack.isEmpty())
      {
         InverseDynamicsJoint joint = jointStack.get(0);
         if(joint instanceof OneDoFJoint)
         {
            FrameVector3D jAxis=((OneDoFJoint)joint).getJointAxis();
            ReferenceFrame referenceFrame = GeometryTools.constructReferenceFrameFromPointAndZAxis(joint.getName()+"JointAxis", new FramePoint3D(jAxis.getReferenceFrame()), new FrameVector3D(jAxis.getReferenceFrame(),jAxis));
            YoGraphicReferenceFrame yoGraphicReferenceFrame = new YoGraphicReferenceFrame(referenceFrame , registry, length, YoAppearance.Gold());
            yoGraphicsList.add(yoGraphicReferenceFrame);
            yoGraphicReferenceFrames.add(yoGraphicReferenceFrame);
         }
         List<InverseDynamicsJoint> childrenJoints = joint.getSuccessor().getChildrenJoints();
         jointStack.addAll(childrenJoints);
         jointStack.remove(joint);
      }
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
   }
   
   @Override
   public void initialize()
   {
      doControl();
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doControl()
   {
      for (int i = 0; i < yoGraphicReferenceFrames.size(); i++)
      {
         yoGraphicReferenceFrames.get(i).update();
      } 
   }

}
