package us.ihmc.rdx.simulation.environment.isaac;

import java.util.List;

/**
 * An environment object that {@link RDXIsaacSimEnvironmentExporter} writes as a fixed-base USD
 * articulation instead of one rigid body.
 */
public interface RDXIsaacSimArticulatedObject
{
   /** Root first, then each link after its parent. Joint positions are the current ones. */
   List<RDXIsaacSimLink> getIsaacSimLinks();

   /** Link name pairs that must not collide with each other, beyond the jointed pairs. */
   List<String[]> getIsaacSimExcludedLinkPairs();
}
