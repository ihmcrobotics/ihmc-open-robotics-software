package us.ihmc.robotics.geometry.interfaces;

import us.ihmc.robotics.EpsilonComparable;
import us.ihmc.robotics.Settable;
import us.ihmc.robotics.geometry.transformables.Transformable;

public interface GeometryObject<T> extends Transformable, EpsilonComparable<T>, Settable<T>
{

}
