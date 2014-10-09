package us.ihmc.communication.packets;

import us.ihmc.utilities.net.ComparableDataObject;

import java.util.Random;

public interface Packet<T> extends ComparableDataObject<T>
{
    Packet<T> createRandomPacket(Random random);
}
