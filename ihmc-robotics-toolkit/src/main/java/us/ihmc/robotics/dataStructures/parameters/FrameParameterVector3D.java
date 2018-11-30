package us.ihmc.robotics.dataStructures.parameters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FrameParameterVector3D extends ParameterVector3D implements FrameVector3DReadOnly
{
   private final ReferenceFrame referenceFrame;

   public FrameParameterVector3D(String prefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this(prefix, referenceFrame, null, registry);
   }

   public FrameParameterVector3D(String prefix, ReferenceFrame referenceFrame, Vector3DReadOnly defaults, YoVariableRegistry registry)
   {
      super(prefix, defaults, registry);
      this.referenceFrame = referenceFrame;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

}
