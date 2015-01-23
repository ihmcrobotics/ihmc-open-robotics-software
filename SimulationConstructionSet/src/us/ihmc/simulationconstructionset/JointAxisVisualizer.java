package us.ihmc.simulationconstructionset;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

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
