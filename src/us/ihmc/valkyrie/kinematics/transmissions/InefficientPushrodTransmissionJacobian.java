package us.ihmc.valkyrie.kinematics.transmissions;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.TransformReferenceFrame;
import us.ihmc.utilities.math.geometry.TranslationReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;

public class InefficientPushrodTransmissionJacobian
{
   private double h;    // height of pitch axis above roll axis in meters (m)

   private double length;    // futek link length (m)
   private double lengthSquared = length * length;

   private final Vector3d rod5 = new Vector3d();    // position where rod 5 passes through bone frame plane. x is forward. y is to the left. z is up. (m)
   private final Vector3d rod6 = new Vector3d();    // position where rod 6 passes through bone frame plane. x is forward. y is to the left. z is up. (m)

   private final Vector3d rodBottom5 = new Vector3d();  // position vector of futek link base for actuator 5 side in foot frame (m)
   private final Vector3d rodBottom6 = new Vector3d();   // position vector of futek link base for actuator 6 side in foot frame (m)
   
   private boolean useFuteks = true; 
   
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final TranslationReferenceFrame boneFrame = new TranslationReferenceFrame("boneFrame", worldFrame);
   private final TransformReferenceFrame afterPitchFrame = new TransformReferenceFrame("afterPitchFrame", boneFrame);
   private final TranslationReferenceFrame beforeRollFrame = new TranslationReferenceFrame("beforeRollFrame", afterPitchFrame);
   private final TransformReferenceFrame footFrame = new TransformReferenceFrame("footFrame", beforeRollFrame);

   private final Transform3D pitchTransform3D = new Transform3D();
   private final Transform3D rollTransform3D = new Transform3D();

   private final FramePoint b5InFootFrame = new FramePoint();   
   private final FramePoint b6InFootFrame = new FramePoint();   

   private final FramePoint b5InBoneFrame = new FramePoint();
   private final FramePoint b6InBoneFrame = new FramePoint();

   private final FramePoint t5InBoneFrame = new FramePoint();
   private final FramePoint t6InBoneFrame = new FramePoint();
   
   private final FramePoint t5InFootFrame = new FramePoint();
   private final FramePoint t6InFootFrame = new FramePoint();

   private final FrameVector f5VectorInBoneFrame = new FrameVector(boneFrame);
   private final FrameVector f6VectorInBoneFrame = new FrameVector(boneFrame);

   private final FrameVector f5VectorInFootFrame = new FrameVector(footFrame);
   private final FrameVector f6VectorInFootFrame = new FrameVector(footFrame);

   private final FrameVector tempRVector = new FrameVector();
   private final FrameVector tempCrossVector = new FrameVector();

   private boolean visualize = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable jPitch5 = new DoubleYoVariable("jPitch5", registry);
   private final DoubleYoVariable jPitch6 = new DoubleYoVariable("jPitch6", registry);
   private final DoubleYoVariable jRoll5 = new DoubleYoVariable("jRoll5", registry);
   private final DoubleYoVariable jRoll6 = new DoubleYoVariable("jRoll6", registry);

   private final DynamicGraphicPosition b5Viz, b6Viz, t5Viz, t6Viz;
   private final DynamicGraphicReferenceFrame boneFrameViz, afterPitchFrameViz, beforeRollFrameViz, footFrameViz;

   private final boolean DEBUG = false;

   public InefficientPushrodTransmissionJacobian(PushRodTransmissionJoint pushRodTransmissionJoint, YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      switch (pushRodTransmissionJoint)
      {
      case ANKLE:
      {
         setupForAnkleActuators();
         break;
      }
      case WAIST:
      {
         setupForWaistActuators();
         break;
      }
      }
      boneFrame.updateTranslation(new FrameVector(worldFrame, 0.0, 0.0, 1.0));    // Arbitrary. Just put it in the air. If we wanted to have things align with the real robot, then this should be at the ankle.
      beforeRollFrame.updateTranslation(new FrameVector(afterPitchFrame, 0.0, 0.0, -h));

      if (dynamicGraphicObjectsListRegistry == null)
      {
         visualize = false;
      }

      if (visualize)
      {
         DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList(getClass().getSimpleName());

         double ballRadius = 0.005;

         t5Viz = new DynamicGraphicPosition("t5Viz", "", registry, ballRadius, YoAppearance.Blue());
         t6Viz = new DynamicGraphicPosition("t6Viz", "", registry, ballRadius, YoAppearance.Green());

         b5Viz = new DynamicGraphicPosition("b5Viz", "", registry, ballRadius, YoAppearance.Red());
         b6Viz = new DynamicGraphicPosition("b6Viz", "", registry, ballRadius, YoAppearance.Gold());

         double frameScale = 0.05;

         boneFrameViz = new DynamicGraphicReferenceFrame(boneFrame, registry, frameScale);
         afterPitchFrameViz = new DynamicGraphicReferenceFrame(afterPitchFrame, registry, frameScale * 0.8);
         beforeRollFrameViz = new DynamicGraphicReferenceFrame(beforeRollFrame, registry, frameScale * 0.6);
         footFrameViz = new DynamicGraphicReferenceFrame(footFrame, registry, frameScale * 0.4);

         dynamicGraphicObjectsList.add(b5Viz);
         dynamicGraphicObjectsList.add(b6Viz);
         dynamicGraphicObjectsList.add(t5Viz);
         dynamicGraphicObjectsList.add(t6Viz);

         dynamicGraphicObjectsList.add(boneFrameViz);
         dynamicGraphicObjectsList.add(afterPitchFrameViz);
         dynamicGraphicObjectsList.add(beforeRollFrameViz);
         dynamicGraphicObjectsList.add(footFrameViz);

         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);
      }
      else
      {
         b5Viz = b6Viz = t5Viz = t6Viz = null;
         boneFrameViz = afterPitchFrameViz = beforeRollFrameViz = footFrameViz = null;
      }

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }
   
   public void setupForAnkleActuators()
   {
      h = 0.0127;

      length = 0.1049655;
      lengthSquared = length * length;

      rod5.set(-0.0215689, -0.04128855, 0.0);    
      rod6.set(-0.0215689, 0.04128855, 0.0);     

      rodBottom5.set(-0.0364, -0.0355, 0.0176);     
      rodBottom6.set(-0.0364, 0.0355, 0.0176);         
   }
   
   public void setupForWaistActuators()
   {
      // TODO: Add Waist parameters. Right now they are ankle parameters.
      h = 0.0127;

      length = 0.1049655;
      lengthSquared = length * length;

      rod5.set(-0.0215689, -0.04128855, 0.0);    
      rod6.set(-0.0215689, 0.04128855, 0.0);     

      rodBottom5.set(-0.0364, -0.0355, 0.0176);     
      rodBottom6.set(-0.0364, 0.0355, 0.0176);         
   }

   public void setUseFuteks(boolean useFuteks)
   {
      this.useFuteks = useFuteks;
   }
   
   public void computeJacobian(double[][] jacobianToPack, double pitch, double roll)
   {
      // Update forward kinematics reference frames using roll and pitch.
      pitchTransform3D.setIdentity();
      pitchTransform3D.rotY(pitch);

      rollTransform3D.setIdentity();
      rollTransform3D.rotX(roll);

      afterPitchFrame.updateTransform(pitchTransform3D);
      footFrame.updateTransform(rollTransform3D);

      b5InFootFrame.setIncludingFrame(footFrame, rodBottom5);   
      b6InFootFrame.setIncludingFrame(footFrame, rodBottom6);   
      
      b5InBoneFrame.setIncludingFrame(b5InFootFrame);
      b5InBoneFrame.changeFrame(boneFrame);

      b6InBoneFrame.setIncludingFrame(b6InFootFrame);
      b6InBoneFrame.changeFrame(boneFrame);

      printIfDebug("b5InBoneFrame = " + b5InBoneFrame);
      printIfDebug("b6InBoneFrame = " + b6InBoneFrame);

      // Solve for t5, t6 in bone frame:

      double xDiff = rod5.getX() - b5InBoneFrame.getX();
      double yDiff = rod5.getY() - b5InBoneFrame.getY();
      double t5zInBoneFrame = b5InBoneFrame.getZ() + Math.sqrt(lengthSquared - xDiff * xDiff - yDiff * yDiff);

      xDiff = rod6.getX() - b6InBoneFrame.getX();
      yDiff = rod6.getY() - b6InBoneFrame.getY();
      double t6zInBoneFrame = b6InBoneFrame.getZ() + Math.sqrt(lengthSquared - xDiff * xDiff - yDiff * yDiff);

      printIfDebug("t5zInBoneFrame = " + t5zInBoneFrame);
      printIfDebug("t6zInBoneFrame = " + t6zInBoneFrame);

      t5InBoneFrame.setIncludingFrame(boneFrame, rod5);
      t6InBoneFrame.setIncludingFrame(boneFrame, rod6);

      t5InBoneFrame.setZ(t5zInBoneFrame);
      t6InBoneFrame.setZ(t6zInBoneFrame);

      t5InFootFrame.setIncludingFrame(t5InBoneFrame);
      t6InFootFrame.setIncludingFrame(t6InBoneFrame);
      
      t5InFootFrame.changeFrame(footFrame);
      t6InFootFrame.changeFrame(footFrame);
      
      if (DEBUG)
      {
         System.out.println("t5InFootFrame = " + t5InFootFrame);
         System.out.println("t6InFootFrame = " + t6InFootFrame);
      }


      // Do R cross F to get Jacobian elements:

      if (useFuteks)
      {
         f5VectorInBoneFrame.sub(b5InBoneFrame, t5InBoneFrame);
         f6VectorInBoneFrame.sub(b6InBoneFrame, t6InBoneFrame);

         f5VectorInBoneFrame.normalize();
         f6VectorInBoneFrame.normalize();
         
         printIfDebug("f5VectorInBoneFrame = " + f5VectorInBoneFrame);
         printIfDebug("f6VectorInBoneFrame = " + f6VectorInBoneFrame);
      }
      else
      {
         f5VectorInBoneFrame.setIncludingFrame(boneFrame, 0.0, 0.0, -1.0);
         f6VectorInBoneFrame.setIncludingFrame(boneFrame, 0.0, 0.0, -1.0);
      }
      
      f5VectorInFootFrame.setIncludingFrame(f5VectorInBoneFrame);
      f5VectorInFootFrame.changeFrame(footFrame);
      f6VectorInFootFrame.setIncludingFrame(f6VectorInBoneFrame);
      f6VectorInFootFrame.changeFrame(footFrame);

      tempRVector.setIncludingFrame(t5InBoneFrame); //kjb5InBoneFrame);
      tempCrossVector.setToZero(tempRVector.getReferenceFrame());
      tempCrossVector.cross(tempRVector, f5VectorInBoneFrame);
      jPitch5.set(tempCrossVector.getY());

      tempRVector.setIncludingFrame(t6InBoneFrame); //b6InBoneFrame);
      tempCrossVector.cross(tempRVector, f6VectorInBoneFrame);
      jPitch6.set(tempCrossVector.getY());


      tempRVector.setIncludingFrame(t5InFootFrame); //b5InFootFrame);
      tempCrossVector.setToZero(tempRVector.getReferenceFrame());
      tempCrossVector.cross(tempRVector, f5VectorInFootFrame);
      jRoll5.set(tempCrossVector.getX());

      tempRVector.setIncludingFrame(t6InFootFrame); //b6InFootFrame);
      tempCrossVector.cross(tempRVector, f6VectorInFootFrame);
      jRoll6.set(tempCrossVector.getX());

      jacobianToPack[0][0] = jPitch5.getDoubleValue();
      jacobianToPack[0][1] = jPitch6.getDoubleValue();
      jacobianToPack[1][0] = jRoll5.getDoubleValue();
      jacobianToPack[1][1] = jRoll6.getDoubleValue();

      if (visualize)
      {
         t5Viz.setPosition(t5InBoneFrame.changeFrameCopy(worldFrame));
         t6Viz.setPosition(t6InBoneFrame.changeFrameCopy(worldFrame));

         b5Viz.setPosition(b5InFootFrame.changeFrameCopy(worldFrame));
         b6Viz.setPosition(b6InFootFrame.changeFrameCopy(worldFrame));

         boneFrameViz.update();
         afterPitchFrameViz.update();
         beforeRollFrameViz.update();
         footFrameViz.update();

      }

   }

   private void printIfDebug(String message)
   {
      if (DEBUG)
         System.out.println(message);
   }

}
