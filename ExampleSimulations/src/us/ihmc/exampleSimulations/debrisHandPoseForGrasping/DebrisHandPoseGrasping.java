package us.ihmc.exampleSimulations.debrisHandPoseForGrasping;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.inputdevices.SliderBoardConfigurationManager;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphic;
import us.ihmc.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFramePose;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class DebrisHandPoseGrasping
{

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final PoseReferenceFrame handFrameBeforeRotation = new PoseReferenceFrame("handFrameBeforeRotation", worldFrame);
   private final YoGraphicReferenceFrame handFrameBeforeRotViz;

   public DebrisHandPoseGrasping()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

      final YoFramePose debrisPose = new YoFramePose("debris", worldFrame, registry);
      final YoFramePose initialHandPose = new YoFramePose("initialHand", worldFrame, registry);
      final YoFramePose graspingPose = new YoFramePose("grasping", worldFrame, registry);

      debrisPose.setXYZ(0.5, 0.1, 0.5);
      debrisPose.setYawPitchRoll(0.2, 0.2, 0.2);

      initialHandPose.setXYZ(0.34, -0.34, 0.72);
      initialHandPose.setYawPitchRoll(0.13, 0.59, -0.29);

      final ReferenceFrame handFrame = new ReferenceFrame("handFrame", worldFrame)
      {
         private static final long serialVersionUID = 1L;
         private FramePose localFramePose = new FramePose();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            initialHandPose.getFramePoseIncludingFrame(localFramePose);
            localFramePose.getPose(transformToParent);
         }
      };

      final ReferenceFrame debrisFrame = new ReferenceFrame("debrisFrame", worldFrame)
      {
         private static final long serialVersionUID = 1L;
         private FramePose localFramePose = new FramePose();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            debrisPose.getFramePoseIncludingFrame(localFramePose);

            localFramePose.getPose(transformToParent);
         }
      };

      final YoFrameVector graspVectorInWorldFrame = new YoFrameVector("graspVector", worldFrame, registry);
      final YoFramePoint graspVectorStartPointInWorldFrame = new YoFramePoint("graspVectorStartPoint", worldFrame, registry);
      
      
      final ReferenceFrame desiredGraspFrame = new ReferenceFrame("desiredGraspFrame", worldFrame)
      {
         private static final long serialVersionUID = 1L;
         private FramePose localFramePose = new FramePose();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            graspingPose.getFramePoseIncludingFrame(localFramePose);
            localFramePose.getPose(transformToParent);
         }
      };

      final YoGraphicReferenceFrame handFrameViz = new YoGraphicReferenceFrame(handFrame, registry, 0.3, YoAppearance.Black());
      final YoGraphicReferenceFrame debrisFrameViz = new YoGraphicReferenceFrame(debrisFrame, registry, 0.3, YoAppearance.Blue());
      final YoGraphicReferenceFrame desiredGraspFrameViz = new YoGraphicReferenceFrame(desiredGraspFrame, registry, 0.4, YoAppearance.Yellow());
      handFrameBeforeRotViz = new YoGraphicReferenceFrame(handFrameBeforeRotation, registry, 0.3, YoAppearance.Red());
      final YoGraphicVector graspVectorViz = new YoGraphicVector("graspVectorViz", graspVectorStartPointInWorldFrame, graspVectorInWorldFrame, 0.3, YoAppearance.Darkorange());

      graphicsListRegistry.registerYoGraphic("Frames", handFrameViz);
      graphicsListRegistry.registerYoGraphic("Frames", debrisFrameViz);
      graphicsListRegistry.registerYoGraphic("Frames", desiredGraspFrameViz);
      graphicsListRegistry.registerYoGraphic("Frames", graspVectorViz);
      graphicsListRegistry.registerYoGraphic("ComputedFrames", handFrameBeforeRotViz);

      setupListener(debrisPose, debrisFrame, debrisFrameViz);
 
      final FrameVector graspVectorInDebrisFrame = new FrameVector(debrisFrame);
      graspVectorInDebrisFrame.set(-1.0,0.0,0.0);
      
      final FramePoint graspVectorStartPointInDebrisFrame = new FramePoint(debrisFrame);
      graspVectorStartPointInDebrisFrame.setToZero();

      graspVectorInWorldFrame.setAndMatchFrame(graspVectorInDebrisFrame);
      
      graspVectorStartPointInWorldFrame.setAndMatchFrame(graspVectorStartPointInDebrisFrame);
      
      VariableChangedListener variableChangedListener = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            graspVectorInWorldFrame.setAndMatchFrame(graspVectorInDebrisFrame);
            
            graspVectorStartPointInWorldFrame.setAndMatchFrame(graspVectorStartPointInDebrisFrame);
            
         }
      };
      debrisPose.attachVariableChangedListener(variableChangedListener );

      setupListener(initialHandPose, handFrame, handFrameViz);
      setupListener(graspingPose, desiredGraspFrame, desiredGraspFrameViz);

      final BooleanYoVariable computeDesiredGraspPose = new BooleanYoVariable("computeDesiredGraspPose", registry);

      computeDesiredGraspPose.addVariableChangedListener(new VariableChangedListener()
      {
         private FramePose localFramePose = new FramePose();
         private Vector3d localGraspVector = new Vector3d();

         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (!computeDesiredGraspPose.getBooleanValue())
               return;

            computeDesiredGraspPose.set(false, false);

            RigidBodyTransform debrisTransform = new RigidBodyTransform();
            debrisPose.getFramePose(localFramePose);
            localFramePose.getPose(debrisTransform);
            Quat4d desiredGraspOrientationToPack = new Quat4d();
            graspVectorInWorldFrame.get(localGraspVector);

            computeDesiredGraspOrientation(debrisTransform, handFrame, desiredGraspOrientationToPack, localGraspVector);
            Point3d position = new Point3d();
            debrisPose.getPosition().get(position);
            graspingPose.setPosition(position);
            graspingPose.setOrientation(desiredGraspOrientationToPack);
         }
      });

      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);

      setupSliderBoard(scs, debrisPose, computeDesiredGraspPose);

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }


   private void setupSliderBoard(SimulationConstructionSet scs, YoFramePose debrisPose, BooleanYoVariable computeDesiredGraspPose)
   {
      final SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);

      int i = 1;
      sliderBoardConfigurationManager.setSlider(i++, debrisPose.getOrientation().getYaw(), -Math.PI, Math.PI);
      sliderBoardConfigurationManager.setSlider(i++, debrisPose.getOrientation().getPitch(),-Math.PI, Math.PI);
      sliderBoardConfigurationManager.setSlider(i++, debrisPose.getOrientation().getRoll(), -Math.PI, Math.PI);
      sliderBoardConfigurationManager.setSlider(i++, debrisPose.getPosition().getYoX(), 0.0, 3.0);
      sliderBoardConfigurationManager.setSlider(i++, debrisPose.getPosition().getYoY(), -1.0, 1.0);
      sliderBoardConfigurationManager.setSlider(i++, debrisPose.getPosition().getYoZ(), 0.0, 1.0);
      
      sliderBoardConfigurationManager.setButton(1, computeDesiredGraspPose);
   }

   
   private void setupListener(final YoFramePose pose, final ReferenceFrame frameToUpdate, final YoGraphic yoGraphicToUpdate)
   {
      VariableChangedListener variableChangedListener = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            frameToUpdate.update();
            yoGraphicToUpdate.update();
         }
      };
      pose.attachVariableChangedListener(variableChangedListener);
      variableChangedListener.variableChanged(null);
   }

   private void computeDesiredGraspOrientation(RigidBodyTransform debrisTransform, ReferenceFrame handFrame, Quat4d desiredGraspOrientationToPack,
         Vector3d graspVector)
   {
      handFrameBeforeRotation.setPoseAndUpdate(debrisTransform);
      handFrameBeforeRotViz.update();
      
      FramePose handPoseSolution1 = new FramePose(handFrameBeforeRotation);
      handPoseSolution1.changeFrame(handFrame);

      FramePose handPoseSolution2 = new FramePose(handFrameBeforeRotation);
      handPoseSolution2.setOrientation(0.0, 0.0, Math.PI);
      handPoseSolution2.changeFrame(handFrame);

      double rollOfSolution1 = handPoseSolution1.getRoll();
      double rollOfSolution2 = handPoseSolution2.getRoll();

      FramePose handPose = new FramePose(handFrameBeforeRotation);
      if (Math.abs(rollOfSolution1) <= Math.abs(rollOfSolution2))
      {
         handPose.setPoseIncludingFrame(handPoseSolution1);
      }
      else
      {
         handPose.setPoseIncludingFrame(handPoseSolution2);
      }
      
      handPose.changeFrame(worldFrame);
      handPose.getOrientation(desiredGraspOrientationToPack);
   }

   public static void main(String[] args)
   {
      new DebrisHandPoseGrasping();
   }
}
