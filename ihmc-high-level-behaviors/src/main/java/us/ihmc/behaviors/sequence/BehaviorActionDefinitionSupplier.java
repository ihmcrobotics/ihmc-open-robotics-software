package us.ihmc.behaviors.sequence;

import us.ihmc.communication.packets.Packet;

public interface BehaviorActionDefinitionSupplier<T extends Packet<T>>
{
   BehaviorActionDefinition<T> getDefinition();
}
