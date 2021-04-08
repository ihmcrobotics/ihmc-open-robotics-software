package us.ihmc.gdx.ui;

/**
 * Created with IntelliJ IDEA.
 * User: teskridge
 * Date: 2/4/13
 * Time: 3:53 PM
 * To change this template use File | Settings | File Templates.
 */
public interface RRTStateInterface<T>
{
   public int numDimensions();

   public double getDimension(int i);

   public T stateFromValues(double[] vals);
}
