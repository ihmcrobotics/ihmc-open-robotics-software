package us.ihmc.robotics.dataStructures;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorBasics;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.FrameIndexMap;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.YoMutableFrameObject;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * TODO: this should probably live in mecano?
 */
public class YoMutableFrameSpatialVector implements SpatialVectorBasics, YoMutableFrameObject
{
   private final YoLong frameId;
   private final FrameIndexMap frameIndexMap;

   private final YoMutableFrameVector3D angularPart;
   private final YoMutableFrameVector3D linearPart;

   public YoMutableFrameSpatialVector(YoMutableFrameVector3D angularPart, YoMutableFrameVector3D linearPart)
   {
      this.frameId = angularPart.getYoFrameIndex();
      this.frameIndexMap = angularPart.getFrameIndexMap();
      this.angularPart = angularPart;
      this.linearPart = linearPart;
      checkFrameConsistency();
   }

   public YoMutableFrameSpatialVector(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      frameId = new YoLong(YoGeometryNameTools.assembleName(namePrefix, "frame", nameSuffix), registry);
      frameIndexMap = new FrameIndexMap.FrameIndexHashMap();

      YoDouble xAngular = new YoDouble(YoGeometryNameTools.createXName(namePrefix, "Angular" + nameSuffix), registry);
      YoDouble yAngular = new YoDouble(YoGeometryNameTools.createYName(namePrefix, "Angular" + nameSuffix), registry);
      YoDouble zAngular = new YoDouble(YoGeometryNameTools.createZName(namePrefix, "Angular" + nameSuffix), registry);
      angularPart = new YoMutableFrameVector3D(xAngular, yAngular, zAngular, getYoFrameIndex(), getFrameIndexMap());

      YoDouble xLinear = new YoDouble(YoGeometryNameTools.createXName(namePrefix, "Linear" + nameSuffix), registry);
      YoDouble yLinear = new YoDouble(YoGeometryNameTools.createYName(namePrefix, "Linear" + nameSuffix), registry);
      YoDouble zLinear = new YoDouble(YoGeometryNameTools.createZName(namePrefix, "Linear" + nameSuffix), registry);
      linearPart = new YoMutableFrameVector3D(xLinear, yLinear, zLinear, getYoFrameIndex(), getFrameIndexMap());
   }

   @Override
   public FrameIndexMap getFrameIndexMap()
   {
      return frameIndexMap;
   }

   @Override
   public YoLong getYoFrameIndex()
   {
      return frameId;
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
      return YoMutableFrameObject.super.getReferenceFrame();
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      checkFrameConsistency();
      YoMutableFrameObject.super.setReferenceFrame(referenceFrame);
      // When constructing this with two YoMutableFrameVector3D objects the angular part is updated only by the super implementation.
      linearPart.setReferenceFrame(referenceFrame);
   }

   /**
    * This is a check that should be called every time this object is interacted with. If this failes
    * it likely means that you created this pose using
    * {@link #YoMutableFramePose3D(YoMutableFramePoint3D, YoMutableFrameQuaternion)} and changed the
    * reference frame of one of the passed objects without modifying the other one from outside this
    * class. This will make the data structure in here inconsistent.
    */
   private void checkFrameConsistency()
   {
      angularPart.checkReferenceFrameMatch(linearPart);
   }
}
