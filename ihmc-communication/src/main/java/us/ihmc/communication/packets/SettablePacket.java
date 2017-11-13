package us.ihmc.communication.packets;

import us.ihmc.euclid.interfaces.Settable;

public abstract class SettablePacket<T extends SettablePacket<T>> extends Packet<T> implements Settable<T>
{
}
