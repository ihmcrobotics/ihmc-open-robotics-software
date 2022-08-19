package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.mecano.spatial.interfaces.FixedFrameMomentumBasics;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;
import us.ihmc.mecano.tools.MecanoIOTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialForce;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoFixedFrameMomentum implements FixedFrameMomentumBasics, Settable<YoFixedFrameMomentum>
{
   /** This is where we store the internal data. */
   private final YoFixedFrameSpatialForce spatialForceVector;

   /**
    * Creates a new momentum with its components set to zero and initializes its reference frame.
    * <p>
    * Note that the reference frame is final.
    * </p>
    *
    * @param namePrefix       a unique name string to use as the prefix for child variable names.
    * @param expressedInFrame in which reference frame the momentum is expressed.
    * @param registry         the registry to register child variables to.
    */
   public YoFixedFrameMomentum(String namePrefix, ReferenceFrame expressedInFrame, YoRegistry registry)
   {
      this(new YoFixedFrameSpatialVector(namePrefix, expressedInFrame, registry));
   }

   /**
    * Creates a new momentum with its components set to zero and initializes its reference frame.
    * <p>
    * Note that the reference frame is final.
    * </p>
    *
    * @param namePrefix       a unique name string to use as the prefix for child variable names.
    * @param nameSuffix       a string to use as the suffix for child variable names.
    * @param expressedInFrame in which reference frame the momentum is expressed.
    * @param registry         the registry to register child variables to.
    */
   public YoFixedFrameMomentum(String namePrefix, String nameSuffix, ReferenceFrame expressedInFrame, YoRegistry registry)
   {
      this(new YoFixedFrameSpatialVector(namePrefix, nameSuffix, expressedInFrame, registry));
   }

   /**
    * Creates a new momentum given its angular and linear parts and initializes its reference frame.
    * <p>
    * Note that the reference frame is final.
    * </p>
    *
    * @param angularPart the vector to use for the angular part.
    * @param linearPart  the vector to use for the linear part.
    */
   public YoFixedFrameMomentum(YoFrameVector3D angularPart, YoFrameVector3D linearPart)
   {
      this(new YoFixedFrameSpatialVector(angularPart, linearPart));
   }

   /**
    * Creates a new momentum given the spatial vector holding data and initializes its
    * reference frames.
    * <p>
    * Note that the reference frames are final.
    * </p>
    *
    * @param spatialVector the spatial vector to use for holding data.
    */
   public YoFixedFrameMomentum(YoFixedFrameSpatialVector spatialVector)
   {
      this.spatialForceVector = new YoFixedFrameSpatialForce(spatialVector.getAngularPart(), spatialVector.getLinearPart());
   }

   /** {@inheritDoc} */
   @Override
   public void set(YoFixedFrameMomentum other)
   {
      FixedFrameMomentumBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return spatialForceVector.getReferenceFrame();
   }

   /** {@inheritDoc} */
   @Override
   public YoFrameVector3D getAngularPart()
   {
      return spatialForceVector.getAngularPart();
   }

   /** {@inheritDoc} */
   @Override
   public YoFrameVector3D getLinearPart()
   {
      return spatialForceVector.getLinearPart();
   }

   /** {@inheritDoc} */
   @Override
   public void applyTransform(Transform transform)
   {
      spatialForceVector.applyTransform(transform);
   }

   /** {@inheritDoc} */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      spatialForceVector.applyInverseTransform(transform);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof MomentumReadOnly)
         return equals((MomentumReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this momentum vector as follows:<br>
    * Momentum: [angular = (x, y, z), linear = (x, y, z)] - expressedInFrame
    *
    * @return the {@code String} representing this momentum.
    */
   @Override
   public String toString()
   {
      return MecanoIOTools.getMomentumString(this);
   }
}
