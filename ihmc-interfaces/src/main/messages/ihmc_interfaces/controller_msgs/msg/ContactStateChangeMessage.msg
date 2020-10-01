# This message is part of the IHMC multi-contact controller API.
# Provides a way for a user or module external to the controller to signal a change in contact state

# Time relative to when the message is received that the change in contact will be processed
float64 time
# Hash code of the rigid body that will have a contact change
int32 rigid_body_hash_code
# Flag to indicate whether this rigid body is making or breaking contact.
# If true, the controller will try to add a contact point at the given time. If the rigid body is already in contact, this request is ignored.
# Otherwise, the controller will add a contact point at the rigid body's lowest point in world frame as defined by its collision mesh.
# If false, the controller will remove all contact points associated with this rigid body.
bool add_contact_point
