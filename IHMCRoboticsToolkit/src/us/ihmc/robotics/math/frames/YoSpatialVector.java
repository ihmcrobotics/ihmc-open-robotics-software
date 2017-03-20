package us.ihmc.robotics.math.frames;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameTuple;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoSpatialVector
{
   protected final String namePrefix;
   protected final String nameSuffix;

   /** This is where the data is stored. All operations must act on these numbers. */
   protected final YoFrameVector linearPart;
   protected final YoFrameVector angularPart;
   /** Redundant but allows to make sure the frame isn't changed. */
   protected final ReferenceFrame expressedInFrame;

   public YoSpatialVector(String namePrefix, String nameSuffix, ReferenceFrame expressedInFrame, YoVariableRegistry registry)
   {
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;
      
      linearPart = new YoFrameVector(namePrefix + "Linear", nameSuffix, expressedInFrame, registry);
      angularPart = new YoFrameVector(namePrefix + "Angular", nameSuffix, expressedInFrame, registry);
      this.expressedInFrame = expressedInFrame;
   }
   
   public YoSpatialVector(String namePrefix, ReferenceFrame expressedInFrame, YoVariableRegistry registry)
   {
      this(namePrefix, "", expressedInFrame, registry);
   }
   
   public YoSpatialVector(YoFrameVector yoLinearPart, YoFrameVector yoAngularPart, ReferenceFrame expressedInFrame)
   {
      this.namePrefix = StringUtils.getCommonPrefix(yoLinearPart.getNamePrefix(), yoAngularPart.getNamePrefix());
      this.nameSuffix = YoFrameVariableNameTools.getCommonSuffix(yoLinearPart.getNameSuffix(), yoAngularPart.getNameSuffix());
      
      this.linearPart = yoLinearPart;
      this.angularPart = yoAngularPart;
      this.expressedInFrame = expressedInFrame;
   }
   
   public void set(YoSpatialVector yoSpacialVector)
   {
      linearPart.set(yoSpacialVector.getExpressedInFrame(), yoSpacialVector.getLinearPartX(), yoSpacialVector.getLinearPartY(), yoSpacialVector.getLinearPartZ());
      angularPart.set(yoSpacialVector.getExpressedInFrame(), yoSpacialVector.getAngularPartX(), yoSpacialVector.getAngularPartY(), yoSpacialVector.getAngularPartZ());
   }
   
   public void setLinearPart(Vector3DReadOnly vector3d)
   {
      linearPart.set(vector3d);
   }
   
   public void setLinearPart(FrameVector frameVector)
   {
      linearPart.set(frameVector);
   }
   
   public void setLinearPart(YoFrameVector yoFrameVector)
   {
      linearPart.set(yoFrameVector);
   }
   
   public void setAngularPart(Vector3DReadOnly vector3d)
   {
      angularPart.set(vector3d);
   }
   
   public void setAngularPart(FrameVector frameVector)
   {
      angularPart.set(frameVector);
   }
   
   public void setAngularPart(YoFrameVector yoFrameVector)
   {
      angularPart.set(yoFrameVector);
   }
   
   public void scale(double scaleFactor)
   {
      scaleLinearPart(scaleFactor);
      scaleAngularPart(scaleFactor);
   }

   public void scaleLinearPart(double scaleFactor)
   {
      linearPart.scale(scaleFactor);
   }

   public void scaleAngularPart(double scaleFactor)
   {
      angularPart.scale(scaleFactor);
   }
   
   public void setToZero()
   {
      linearPart.setToZero();
      angularPart.setToZero();
   }

   public YoFrameVector getYoLinearPart()
   {
      return linearPart;
   }

   public YoFrameVector getYoAngularPart()
   {
      return angularPart;
   }
   
   public FrameVector getLinearPart()
   {
      return linearPart.getFrameTuple();
   }

   public FrameVector getAngularPart()
   {
      return angularPart.getFrameTuple();
   }
   
   public void setAndMatchFrameLinearPart(FrameTuple<?, ?> frameVector)
   {
      linearPart.setAndMatchFrame(frameVector);
   }
   
   public void setAndMatchFrameAngularPart(FrameTuple<?, ?> frameVector)
   {
      angularPart.setAndMatchFrame(frameVector);
   }
   
   public double getLinearPartX()
   {
      return linearPart.getX();
   }
   
   public double getLinearPartY()
   {
      return linearPart.getY();
   }
   
   public double getLinearPartZ()
   {
      return linearPart.getZ();
   }
   
   public double getAngularPartX()
   {
      return angularPart.getX();
   }
   
   public double getAngularPartY()
   {
      return angularPart.getY();
   }
   
   public double getAngularPartZ()
   {
      return angularPart.getZ();
   }

   public ReferenceFrame getExpressedInFrame()
   {
      return expressedInFrame;
   }

   public String getNamePrefix()
   {
      return namePrefix;
   }

   public String getNameSuffix()
   {
      return nameSuffix;
   }
}
