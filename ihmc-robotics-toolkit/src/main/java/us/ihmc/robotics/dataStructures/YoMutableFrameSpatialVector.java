package us.ihmc.robotics.dataStructures;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.util.YoFrameVariableNameTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFrameObject;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFramePoint3D;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFrameQuaternion;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFrameVector3D;

/**
 * TODO: this should probably live in mecano?
 */
public class YoMutableFrameSpatialVector extends YoMutableFrameObject implements SpatialVectorBasics
{
   private final YoMutableFrameVector3D angularPart;
   private final YoMutableFrameVector3D linearPart;

   public YoMutableFrameSpatialVector(YoMutableFrameVector3D angularPart, YoMutableFrameVector3D linearPart)
   {
      super(angularPart.getYoFrameIndex(), angularPart.getFrameIndexMap());
      this.angularPart = angularPart;
      this.linearPart = linearPart;
      checkFrameConsistency();
   }

   public YoMutableFrameSpatialVector(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, registry);

      YoDouble xAngular = new YoDouble(YoFrameVariableNameTools.createXName(namePrefix, "Angular" + nameSuffix), registry);
      YoDouble yAngular = new YoDouble(YoFrameVariableNameTools.createYName(namePrefix, "Angular" + nameSuffix), registry);
      YoDouble zAngular = new YoDouble(YoFrameVariableNameTools.createZName(namePrefix, "Angular" + nameSuffix), registry);
      angularPart = new YoMutableFrameVector3D(xAngular, yAngular, zAngular, getYoFrameIndex(), getFrameIndexMap());

      YoDouble xLinear = new YoDouble(YoFrameVariableNameTools.createXName(namePrefix, "Linear" + nameSuffix), registry);
      YoDouble yLinear = new YoDouble(YoFrameVariableNameTools.createYName(namePrefix, "Linear" + nameSuffix), registry);
      YoDouble zLinear = new YoDouble(YoFrameVariableNameTools.createZName(namePrefix, "Linear" + nameSuffix), registry);
      linearPart = new YoMutableFrameVector3D(xLinear, yLinear, zLinear, getYoFrameIndex(), getFrameIndexMap());
   }

   @Override
   public FixedFrameVector3DBasics getAngularPart()
   {
      checkFrameConsistency();
      return angularPart;
   }

   @Override
   public FixedFrameVector3DBasics getLinearPart()
   {
      checkFrameConsistency();
      return linearPart;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      checkFrameConsistency();
      return super.getReferenceFrame();
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      checkFrameConsistency();
      super.setReferenceFrame(referenceFrame);
      // When constructing this with two YoMutableFrameVector3D objects the angular part is updated only by the super implementation.
      linearPart.setReferenceFrame(referenceFrame);
   }

   /**
    * This is a check that should be called every time this object is interacted with. If this
    * failes it likely means that you created this pose using
    * {@link #YoMutableFramePose3D(YoMutableFramePoint3D, YoMutableFrameQuaternion)} and changed the
    * reference frame of one of the passed objects without modifying the other one from outside this
    * class. This will make the data structure in here inconsistent.
    */
   private void checkFrameConsistency()
   {
      angularPart.checkReferenceFrameMatch(linearPart);
   }
}
